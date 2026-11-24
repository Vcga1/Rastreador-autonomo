/*
 * Nodo GPS solar - nRF52840 + RA-02 (SX1278) + NEO-6M
 * ------------------------------------------------------
 * Enciende/lee el GPS, transmite por LoRa un mensaje con ID + posición,
 * y vuelve a dormir. Reintenta cada 3 minutos si no obtiene fix.
 *
 * Requiere la librería TinyGPS++ (Mikal Hart) - instálala desde el
 * Library Manager del Arduino IDE.
 *
 * AJUSTA:
 *  - DEVICE_ID: identificador de este nodo
 *  - GPS_RX_PIN / GPS_TX_PIN: pines libres en tu placa
 *  - GPS_POWER_PIN (opcional): si tienes un MOSFET/load switch para
 *    cortar la alimentación del GPS entre lecturas, descomenta y
 *    define el pin. Esto ahorra mucha más energía que solo dejarlo
 *    conectado al 3V3 fijo, a costa de que cada lectura sea un
 *    "cold start" (puede tardar más en obtener fix).
 */

#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>

// SoftwareSerial no está disponible de forma nativa en el core nRF52;
// si tu core no lo trae, instala la librería "SoftwareSerial for nRF52"
// o dime tu core exacto y te lo adapto a Serial1 por hardware.
#include <SoftwareSerial.h>

// --- Identificador de este nodo ---
#define DEVICE_ID  "GPS01"

// --- Pines SPI custom (nRF52840) - LoRa ---
#define LORA_MISO   P0_02
#define LORA_MOSI   P1_15
#define LORA_SCK    P1_11
#define LORA_NSS    P1_13
#define LORA_RST    P0_09
#define LORA_DIO0   P0_29
#define LORA_FREQUENCY  433E6

// --- Pines GPS (AJUSTA a pines libres reales de tu placa) ---
#define GPS_RX_PIN  P0_08   // <- a este pin del MCU llega el TX del GPS
#define GPS_TX_PIN  P0_06   // <- este pin del MCU va al RX del GPS
#define GPS_BAUD    9600

// Descomenta si tienes control de alimentación por MOSFET/load switch
#define GPS_POWER_PIN  P1_06
#define GPS_POWER_ON   LOW  // ajusta segun tu circuito (activo alto/bajo)
#define GPS_POWER_OFF  1

#define TX_INTERVAL_MS   (15UL * 60UL * 1000UL)  // intervalo normal: 15 minutos
#define GPS_RETRY_INTERVAL_MS (3UL * 60UL * 1000UL)  // reintento tras NOFIX
#define GPS_FIX_TIMEOUT_MS  120000UL             // maximo tiempo esperando fix (cold start)

// Pon en 1 solo mientras depuras
#define DEBUG_MODE  0

#if DEBUG_MODE
  #define DBG_BEGIN(x)    Serial.begin(x)
  #define DBG_PRINT(x)    Serial.print(x)
  #define DBG_PRINTLN(x)  Serial.println(x)
#else
  #define DBG_BEGIN(x)
  #define DBG_PRINT(x)
  #define DBG_PRINTLN(x)
#endif

SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;

void setup() {
  #ifdef LED_BUILTIN
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
  #endif
  #ifdef LED_RED
    pinMode(LED_RED, OUTPUT);
    digitalWrite(LED_RED, LOW);
  #endif
  #ifdef LED_BLUE
    pinMode(LED_BLUE, OUTPUT);
    digitalWrite(LED_BLUE, LOW);
  #endif
  #ifdef LED_CONN
    pinMode(LED_CONN, OUTPUT);
    digitalWrite(LED_CONN, LOW);
  #endif

  #ifdef GPS_POWER_PIN
    pinMode(GPS_POWER_PIN, OUTPUT);
    digitalWrite(GPS_POWER_PIN, GPS_POWER_OFF);
  #endif

  DBG_BEGIN(115200);
#if DEBUG_MODE
  while (!Serial && millis() < 3000);
#endif

  SPI.setPins(LORA_MISO, LORA_SCK, LORA_MOSI);
  SPI.begin();
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQUENCY)) {
    DBG_PRINTLN("Error: no se pudo iniciar el modulo LoRa.");
    while (1) { delay(1000); }
  }

  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setTxPower(17);
  LoRa.sleep();

  DBG_PRINTLN("Nodo GPS listo.");
}

void loop() {
  String payload;
  bool fixValido = obtenerPosicionGPS(payload);
  transmitir(payload);

  // Reintenta mas seguido mientras no haya fix; vuelve al intervalo normal al obtenerlo.
  delay(fixValido ? TX_INTERVAL_MS : GPS_RETRY_INTERVAL_MS);
}

bool obtenerPosicionGPS(String &msg) {
  #ifdef GPS_POWER_PIN
    digitalWrite(GPS_POWER_PIN, GPS_POWER_ON);
    delay(100);  // tiempo de arranque del modulo
  #endif

  gpsSerial.begin(GPS_BAUD);

  unsigned long start = millis();
  bool fixValido = false;

  while (millis() - start < GPS_FIX_TIMEOUT_MS) {
    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read());
    }
    if (gps.location.isUpdated() && gps.location.isValid()) {
      fixValido = true;
      break;
    }
  }

  if (fixValido) {
    msg = String(DEVICE_ID) + ","
        + String(gps.location.lat(), 6) + ","
        + String(gps.location.lng(), 6) + ","
        + String(gps.satellites.value()) + ","
        + String(gps.hdop.hdop(), 1);
    DBG_PRINT("Fix obtenido: ");
    DBG_PRINTLN(msg);
  } else {
    msg = String(DEVICE_ID) + ",NOFIX";
    DBG_PRINTLN("Timeout esperando fix GPS.");
  }

  gpsSerial.end();
  #ifdef GPS_POWER_PIN
    digitalWrite(GPS_POWER_PIN, GPS_POWER_OFF);
  #endif

  return fixValido;
}

void transmitir(const String &payload) {
  LoRa.idle();

  DBG_PRINT("Transmitiendo: ");
  DBG_PRINTLN(payload);

  LoRa.beginPacket();
  LoRa.print(payload);
  LoRa.endPacket();

  LoRa.sleep();
}