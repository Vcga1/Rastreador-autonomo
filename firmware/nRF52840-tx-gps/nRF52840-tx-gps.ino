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
#include <InternalFileSystem.h>
using namespace Adafruit_LittleFS_Namespace;

// SoftwareSerial no está disponible de forma nativa en el core nRF52;
// si tu core no lo trae, instala la librería "SoftwareSerial for nRF52"
// o dime tu core exacto y te lo adapto a Serial1 por hardware.
#include <SoftwareSerial.h>

// --- Identificador de este nodo ---
#define DEVICE_ID  1

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

// Protocolo compatible con el receptor ESP32 de referencias.
#define PKT_DATA 0x01
#define PKT_REQ  0x02
#define LISTEN_WINDOW_MS 1200UL
#define BURST_GAP_MS 350UL
#define MAX_BURST 15
#define BUF_SIZE 512
#define LOG_CAPACITY 480  // 19.2 KB de historial en LittleFS
#define LOG_FILE "/gps_log.bin"

typedef struct __attribute__((packed)) {
  uint8_t pkt_type;
  uint16_t device_id;
  uint16_t seq;
  char timestamp[20];
  int32_t lat;
  int32_t lon;
  uint16_t speed;
  uint8_t flags;
} data_pkt_t;

typedef struct __attribute__((packed)) {
  uint8_t pkt_type;
  uint16_t device_id;
  uint16_t from_seq;
  uint16_t to_seq;
} req_pkt_t;

typedef struct __attribute__((packed)) {
  data_pkt_t pkt;
  uint8_t crc;
  uint8_t pad[3];
} log_rec_t;

static_assert(sizeof(data_pkt_t) == 36, "data_pkt_t size mismatch");
static_assert(sizeof(req_pkt_t) == 7, "req_pkt_t size mismatch");
static_assert(sizeof(log_rec_t) == 40, "log_rec_t size mismatch");

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
data_pkt_t txBuffer[BUF_SIZE];
uint16_t currentSeq = 0;
uint16_t oldestSeq = 0;
uint16_t logWriteIdx = 0;

uint8_t crc8(const uint8_t *data, size_t length);
bool validLogRecord(const log_rec_t &record);
bool loadHistory();
bool persistPacket(const data_pkt_t &packet);
bool obtenerPosicionGPS(data_pkt_t &packet);
bool seqInBuffer(uint16_t seq);
bool enviarPaquete(const data_pkt_t &packet);
void escucharSolicitudes();
void enviarRafaga(uint16_t fromSeq, uint16_t toSeq);

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

  if (!InternalFS.begin() || !loadHistory()) {
    DBG_PRINTLN("Error: no se pudo montar o leer el historial GPS.");
    while (1) { delay(1000); }
  }

  SPI.setPins(LORA_MISO, LORA_SCK, LORA_MOSI);
  SPI.begin();
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQUENCY)) {
    DBG_PRINTLN("Error: no se pudo iniciar el modulo LoRa.");
    while (1) { delay(1000); }
  }

  LoRa.setSpreadingFactor(8);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(6);
  LoRa.setTxPower(17);
  LoRa.enableCrc();
  LoRa.sleep();

  DBG_PRINTLN("Nodo GPS listo.");
}

void loop() {
  data_pkt_t packet;
  bool fixValido = obtenerPosicionGPS(packet);

  if (!persistPacket(packet)) {
    DBG_PRINTLN("Error: no se pudo guardar la medicion; no se transmite.");
    delay(fixValido ? TX_INTERVAL_MS : GPS_RETRY_INTERVAL_MS);
    return;
  }

  txBuffer[packet.seq % BUF_SIZE] = packet;
  currentSeq++;
  if ((uint16_t)(currentSeq - oldestSeq) > BUF_SIZE) oldestSeq++;

  if (!enviarPaquete(packet)) {
    DBG_PRINTLN("Error al transmitir por LoRa; se conserva en el historial.");
  }
  escucharSolicitudes();

  // Reintenta mas seguido mientras no haya fix; vuelve al intervalo normal al obtenerlo.
  delay(fixValido ? TX_INTERVAL_MS : GPS_RETRY_INTERVAL_MS);
}

uint8_t crc8(const uint8_t *data, size_t length) {
  uint8_t crc = 0;
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

bool validLogRecord(const log_rec_t &record) {
  return record.pkt.pkt_type == PKT_DATA
      && record.pkt.device_id == DEVICE_ID
      && record.crc == crc8((const uint8_t *)&record.pkt, sizeof(record.pkt));
}

bool loadHistory() {
  File file(InternalFS);
  if (!file.open(LOG_FILE, FILE_O_READ)) {
    return !InternalFS.exists(LOG_FILE);
  }

  uint32_t slotCount = file.size() / sizeof(log_rec_t);
  if (slotCount > LOG_CAPACITY) slotCount = LOG_CAPACITY;

  bool found = false;
  uint16_t newestSeq = 0;
  uint16_t newestSlot = 0;
  for (uint32_t slot = 0; slot < slotCount; slot++) {
    log_rec_t record;
    if (file.read(&record, sizeof(record)) != sizeof(record)) break;
    if (!validLogRecord(record)) continue;
    if (!found || (int16_t)(record.pkt.seq - newestSeq) > 0) {
      found = true;
      newestSeq = record.pkt.seq;
      newestSlot = (uint16_t)slot;
    }
  }
  file.close();

  if (!found) return true;

  File history(InternalFS);
  if (!history.open(LOG_FILE, FILE_O_READ)) return false;

  uint16_t maxAge = 0;
  for (uint32_t slot = 0; slot < slotCount; slot++) {
    log_rec_t record;
    if (history.read(&record, sizeof(record)) != sizeof(record)) break;
    if (!validLogRecord(record)) continue;

    uint16_t age = (uint16_t)(newestSeq - record.pkt.seq);
    if (age >= BUF_SIZE) continue;

    txBuffer[record.pkt.seq % BUF_SIZE] = record.pkt;
    if (age > maxAge) maxAge = age;
  }
  history.close();

  currentSeq = (uint16_t)(newestSeq + 1);
  oldestSeq = (uint16_t)(newestSeq - maxAge);
  logWriteIdx = (uint16_t)((newestSlot + 1) % LOG_CAPACITY);
  return true;
}

bool persistPacket(const data_pkt_t &packet) {
  log_rec_t record;
  memset(&record, 0xFF, sizeof(record));
  record.pkt = packet;
  record.crc = crc8((const uint8_t *)&packet, sizeof(packet));

  File file(InternalFS);
  if (!file.open(LOG_FILE, FILE_O_WRITE)) return false;

  uint32_t offset = (uint32_t)logWriteIdx * sizeof(log_rec_t);
  if (!file.seek(offset)) {
    file.close();
    return false;
  }

  size_t written = file.write((const uint8_t *)&record, sizeof(record));
  file.flush();
  file.close();
  if (written != sizeof(record)) return false;

  logWriteIdx = (uint16_t)((logWriteIdx + 1) % LOG_CAPACITY);
  return true;
}

bool obtenerPosicionGPS(data_pkt_t &packet) {
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
    if (gps.location.isUpdated() && gps.location.isValid()
        && gps.location.age() < GPS_FIX_TIMEOUT_MS) {
      fixValido = true;
      break;
    }
  }

  memset(&packet, 0, sizeof(packet));
  packet.pkt_type = PKT_DATA;
  packet.device_id = DEVICE_ID;
  packet.seq = currentSeq;

  if (fixValido) {
    packet.lat = (int32_t)(gps.location.lat() * 1e7);
    packet.lon = (int32_t)(gps.location.lng() * 1e7);
    packet.flags = 0x01;

    if (gps.speed.isValid() && gps.speed.age() < GPS_FIX_TIMEOUT_MS) {
      packet.speed = (uint16_t)(gps.speed.kmph() * 10);
      if (gps.speed.kmph() >= 2.0) packet.flags |= 0x04;
    }
    if (gps.hdop.isValid() && gps.hdop.age() < GPS_FIX_TIMEOUT_MS
        && gps.hdop.hdop() <= 3.0) {
      packet.flags |= 0x02;
    }

    if (gps.date.isValid() && gps.time.isValid()
        && gps.date.age() < GPS_FIX_TIMEOUT_MS
        && gps.time.age() < GPS_FIX_TIMEOUT_MS) {
      snprintf(packet.timestamp, sizeof(packet.timestamp),
               "%04d-%02d-%02dT%02d:%02d:%02d",
               gps.date.year(), gps.date.month(), gps.date.day(),
               gps.time.hour(), gps.time.minute(), gps.time.second());
    } else {
      snprintf(packet.timestamp, sizeof(packet.timestamp), "NO-TIME");
    }
    DBG_PRINTLN("Fix GPS guardado.");
  } else {
    snprintf(packet.timestamp, sizeof(packet.timestamp), "NOFIX");
    DBG_PRINTLN("Timeout esperando fix GPS.");
  }

  gpsSerial.end();
  #ifdef GPS_POWER_PIN
    digitalWrite(GPS_POWER_PIN, GPS_POWER_OFF);
  #endif

  return fixValido;
}

bool seqInBuffer(uint16_t seq) {
  uint16_t age = (uint16_t)(currentSeq - seq);
  return age >= 1 && age <= BUF_SIZE;
}

bool enviarPaquete(const data_pkt_t &packet) {
  LoRa.idle();
  LoRa.beginPacket();
  if (LoRa.write((const uint8_t *)&packet, sizeof(packet)) != sizeof(packet)) {
    LoRa.endPacket();
    return false;
  }
  return LoRa.endPacket(false) == 1;
}

void escucharSolicitudes() {
  LoRa.receive();
  unsigned long start = millis();

  while (millis() - start < LISTEN_WINDOW_MS) {
    int packetSize = LoRa.parsePacket();
    if (packetSize == 0) continue;

    if (packetSize != sizeof(req_pkt_t)) {
      while (LoRa.available()) LoRa.read();
      continue;
    }

    req_pkt_t request;
    if (LoRa.readBytes((uint8_t *)&request, sizeof(request)) != sizeof(request)) continue;
    if (request.pkt_type != PKT_REQ || request.device_id != DEVICE_ID) continue;

    uint16_t requested = (uint16_t)(request.to_seq - request.from_seq + 1);
    if (requested == 0 || requested > MAX_BURST) continue;

    LoRa.idle();
    enviarRafaga(request.from_seq, request.to_seq);
    LoRa.receive();
    start = millis();
  }

  LoRa.sleep();
}

void enviarRafaga(uint16_t fromSeq, uint16_t toSeq) {
  uint16_t count = (uint16_t)(toSeq - fromSeq + 1);
  if (count == 0 || count > MAX_BURST) return;

  for (uint16_t i = 0; i < count; i++) {
    uint16_t seq = (uint16_t)(fromSeq + i);
    if (!seqInBuffer(seq)) continue;

    const data_pkt_t &packet = txBuffer[seq % BUF_SIZE];
    if (packet.pkt_type != PKT_DATA || packet.seq != seq) continue;

    enviarPaquete(packet);
    delay(BURST_GAP_MS);
  }
}