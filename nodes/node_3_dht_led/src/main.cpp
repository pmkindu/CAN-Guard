#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <mcp2515.h>
#include <DHT.h>
#include <Adafruit_NeoPixel.h>
#include <U8x8lib.h>
#include <util/atomic.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

// ============================================================
// CAN-Guard - Node 3
// Klassischer Arduino Nano V3 / ATmega328P
//
// Vorhandene Pins bleiben erhalten:
//   MCP2515 CS: D10
//   NeoPixel:   D6
//   DHT11 #1:   D3
//   DHT11 #2:   D4
//   DHT11 #3:   D5
//
// Zusaetzlich:
//   MCP2515 INT: D2
//   OLED SDA:    A4
//   OLED SCL:    A5
//
// DHT-Nachrichten:
//   0x103 / 0x104 / 0x105, DLC 5
//   B0-B1: Temperatur x10, int16, Little Endian
//   B2-B3: Feuchte x10, uint16, Little Endian
//   B4:    0 = gueltig, 1 = ungueltig
//
// Diagnose:
//   0x1F3, DLC 4
//   B0: 1-3 = DHT11, 4 = fehlende Eingangsdaten 0x101
//   B1: 1 = aktiv, 0 = behoben
//   B2: 1 = Lesefehler, 2 = Timeout, 3 = Wertebereich
//   B3: Ereigniszaehler
//
// Serieller Monitor: 115200 Baud
//   s = Status und momentane RAM-Reserve
//
// Hinweis:
// DHT-Library und I2C-Aufrufe sind nicht vollstaendig
// nichtblockierend. Kein garantierter Frameverlust-Nachweis.
// ============================================================


// -------------------- Pins und Hardware --------------------

const int PIN_CAN_CS = 10;
const int PIN_NEOPIXEL = 6;
const int NUM_PIXELS = 4;

const int DHT_PIN_1 = 3;
const int DHT_PIN_2 = 4;
const int DHT_PIN_3 = 5;

constexpr uint8_t PIN_CAN_INT = 2;

DHT dht1(DHT_PIN_1, DHT11);
DHT dht2(DHT_PIN_2, DHT11);
DHT dht3(DHT_PIN_3, DHT11);

DHT* const dhtSensors[3] = { &dht1, &dht2, &dht3 };

Adafruit_NeoPixel pixels(
    NUM_PIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

MCP2515 mcp2515(PIN_CAN_CS);

// SH1106, 128x64, Hardware-I2C, kein Resetpin.
U8X8_SH1106_128X64_NONAME_HW_I2C oled(U8X8_PIN_NONE);


// -------------------- Konfiguration --------------------

constexpr uint32_t DHT_MEASURE_INTERVAL_MS = 1000;
constexpr uint32_t DHT_SEND_INTERVAL_MS = 1000;
constexpr uint32_t DHT_FRESHNESS_MS = 2000;

constexpr uint32_t LDR_TIMEOUT_MS = 1000;

constexpr uint32_t DIAG_REPEAT_MS = 5000;
constexpr uint32_t DIAG_MIN_GAP_MS = 200;
constexpr uint8_t DIAG_QUEUE_SIZE = 4;

constexpr uint8_t INVALID_LIMIT = 3;
constexpr uint8_t VALID_LIMIT = 3;

constexpr uint32_t CAN_RETRY_MS = 2000;

// Deine bisherige Interpretation bleibt:
// kleinerer LDR-Wert -> mehr LEDs.
constexpr uint8_t FALLBACK_LED_COUNT = 2;
constexpr uint8_t LED_INTENSITY = 40;

// OLED-Adresse bei Bedarf auf 0x3D aendern.
constexpr uint8_t OLED_ADDRESS = 0x3C;

constexpr uint8_t EVENT_CLEARED = 0;
constexpr uint8_t EVENT_ACTIVE = 1;

constexpr uint8_t ERROR_READ = 0x01;
constexpr uint8_t ERROR_TIMEOUT = 0x02;
constexpr uint8_t ERROR_RANGE = 0x03;


// -------------------- DHT-Zustand --------------------

struct DhtState {
  int16_t temperature10;
  uint16_t humidity10;

  uint32_t lastSuccessMs;
  uint32_t nextMeasureMs;
  uint32_t nextSendMs;

  bool lastMeasurementValid;

  uint8_t invalidCount;
  uint8_t validCount;
};

DhtState sensors[3] = {};


// -------------------- Diagnosezustand --------------------

// Index 0-2: DHT11, Index 3: Eingang 0x101.
struct FaultState {
  bool active;
  uint8_t code;
  uint8_t eventCounter;
  uint32_t lastRepeatMs;
};

FaultState faults[4] = {};

struct DiagnosticEvent {
  uint8_t source;
  uint8_t eventType;
  uint8_t errorCode;
  uint8_t counter;
};

DiagnosticEvent diagQueue[DIAG_QUEUE_SIZE];

uint8_t queueHead = 0;
uint8_t queueCount = 0;
uint8_t globalEventCounter = 0;
uint8_t repeatCursor = 0;

uint32_t queueOverflows = 0;

bool diagAttemptMade = false;
uint32_t lastDiagAttemptMs = 0;


// -------------------- CAN / LDR --------------------

volatile bool canInterruptPending = false;

bool canReady = false;
uint32_t lastCanInitAttemptMs = 0;

bool receivedLdr = false;
uint32_t lastLdrRxMs = 0;
uint32_t ldrWatchStartedMs = 0;

uint16_t ldrValue = 0;
bool fallbackActive = true;

uint8_t requestedLedCount = FALLBACK_LED_COUNT;
uint8_t shownLedCount = 255;

uint32_t sensorTxAccepted = 0;
uint32_t sensorTxErrors = 0;
uint32_t diagTxAccepted = 0;
uint32_t diagTxErrors = 0;

uint32_t validLdrFrames = 0;
uint32_t invalidLdrFrames = 0;


// -------------------- OLED --------------------

bool oledReady = false;

char oledLines[3][17];

bool oledUpdating = false;
uint8_t oledSensorIndex = 0;
uint8_t oledColumn = 0;

uint32_t lastOledRefreshMs = 0;


// -------------------- Zeitfunktionen --------------------

bool elapsed(uint32_t now, uint32_t previous, uint32_t interval) {
  return static_cast<uint32_t>(now - previous) >= interval;
}

bool deadlineReached(uint32_t now, uint32_t deadline) {
  return static_cast<int32_t>(now - deadline) >= 0;
}


// -------------------- CAN-Interrupt --------------------

void onCanInterrupt() {
  canInterruptPending = true;
}


// -------------------- RAM-Momentaufnahme --------------------

extern int __heap_start;
extern int* __brkval;

int freeRamNow() {
  int stackVariable;

  return reinterpret_cast<char*>(&stackVariable)
      - (__brkval
          ? reinterpret_cast<char*>(__brkval)
          : reinterpret_cast<char*>(&__heap_start));
}


// -------------------- CAN initialisieren --------------------

bool initializeCan() {
  if (mcp2515.reset() != MCP2515::ERROR_OK) {
    return false;
  }

  if (mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ)
      != MCP2515::ERROR_OK) {
    return false;
  }

  // Nur 0x101 in die Empfangspuffer aufnehmen.
  // Software validiert zusaetzlich ID, Flags, DLC und Inhalt.
  if (mcp2515.setFilterMask(
          MCP2515::MASK0, false, 0x7FF) != MCP2515::ERROR_OK) {
    return false;
  }

  if (mcp2515.setFilterMask(
          MCP2515::MASK1, false, 0x7FF) != MCP2515::ERROR_OK) {
    return false;
  }

  for (uint8_t i = 0; i < 6; i++) {
    if (mcp2515.setFilter(
            static_cast<MCP2515::RXF>(i), false, 0x101)
        != MCP2515::ERROR_OK) {
      return false;
    }
  }

  return mcp2515.setNormalMode() == MCP2515::ERROR_OK;
}


// -------------------- Diagnoseereignis vormerken --------------------

void enqueueDiagnostic(uint8_t source,
                       uint8_t eventType,
                       uint8_t errorCode,
                       uint8_t counter) {
  if (queueCount == DIAG_QUEUE_SIZE) {
    queueHead = (queueHead + 1) % DIAG_QUEUE_SIZE;
    queueCount--;
    queueOverflows++;
  }

  const uint8_t tail =
      (queueHead + queueCount) % DIAG_QUEUE_SIZE;

  diagQueue[tail].source = source;
  diagQueue[tail].eventType = eventType;
  diagQueue[tail].errorCode = errorCode;
  diagQueue[tail].counter = counter;

  queueCount++;
}


// -------------------- Fehlerquelle umschalten --------------------

void setFault(uint8_t source, bool active, uint8_t code) {
  if (faults[source].active == active) {
    return;
  }

  faults[source].active = active;

  if (active) {
    // Ausloesende Ursache bis zur Behebung speichern.
    faults[source].code = code;
  }

  faults[source].eventCounter = ++globalEventCounter;
  faults[source].lastRepeatMs = millis();

  enqueueDiagnostic(
      source,
      active ? EVENT_ACTIVE : EVENT_CLEARED,
      faults[source].code,
      faults[source].eventCounter);
}


// -------------------- LDR -> LED-Stufe --------------------

uint8_t ledCountFromLdr(uint16_t value) {
  if (value < 300) return 4;
  if (value < 450) return 3;
  if (value < 650) return 2;
  if (value < 800) return 1;

  return 0;
}


// -------------------- CAN empfangen --------------------

void serviceCanReceive() {
  if (!canReady) {
    return;
  }

  bool notified;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    notified = canInterruptPending;
    canInterruptPending = false;
  }

  // LOW-Abfrage erfasst auch einen bereits anstehenden INT.
  if (!notified &&
      digitalRead(PIN_CAN_INT) != LOW &&
      !mcp2515.checkReceive()) {
    return;
  }

  struct can_frame frame;

  // Begrenztes Budget, beide RX-Puffer regelmaessig leeren.
  for (uint8_t i = 0; i < 4; i++) {
    if (mcp2515.readMessage(&frame) != MCP2515::ERROR_OK) {
      break;
    }

    // Exakte ID schliesst RTR-/Extended-Flags mit aus.
    if (frame.can_id != 0x101) {
      continue;
    }

    if (frame.can_dlc != 3 || frame.data[0] > 1) {
      invalidLdrFrames++;
      continue;
    }

    const uint16_t value =
        static_cast<uint16_t>(frame.data[1]) |
        (static_cast<uint16_t>(frame.data[2]) << 8);

    if (value > 1023) {
      invalidLdrFrames++;
      continue;
    }

    ldrValue = value;
    lastLdrRxMs = millis();
    receivedLdr = true;
    validLdrFrames++;

    fallbackActive = false;
    requestedLedCount = ledCountFromLdr(ldrValue);

    // Timeout wird durch den naechsten gueltigen Frame behoben.
    setFault(3, false, ERROR_TIMEOUT);
  }
}


// -------------------- LDR-Timeout --------------------

void updateLdrTimeout(uint32_t now) {
  const uint32_t reference =
      receivedLdr ? lastLdrRxMs : ldrWatchStartedMs;

  if (elapsed(now, reference, LDR_TIMEOUT_MS)) {
    fallbackActive = true;
    requestedLedCount = FALLBACK_LED_COUNT;

    // Timeout direkt nach 1 s, keine zusaetzliche
    // Drei-Messungen-Verzoegerung.
    setFault(3, true, ERROR_TIMEOUT);
  }
}


// -------------------- NeoPixel aktualisieren --------------------

void updatePixels() {
  if (requestedLedCount == shownLedCount) {
    return;
  }

  pixels.clear();

  for (uint8_t i = 0; i < requestedLedCount; i++) {
    pixels.setPixelColor(
        i, pixels.Color(
            LED_INTENSITY, LED_INTENSITY, LED_INTENSITY));
  }

  pixels.show();
  shownLedCount = requestedLedCount;
}


// -------------------- DHT-Gueligkeit --------------------

bool dhtValueValid(uint8_t index, uint32_t now) {
  return sensors[index].lastMeasurementValid &&
      !elapsed(now,
               sensors[index].lastSuccessMs,
               DHT_FRESHNESS_MS + 1UL);
}


// -------------------- Einen DHT messen --------------------

void measureDht(uint8_t index) {
  serviceCanReceive();

  // Genau eine neue physische Transaktion erzwingen.
  const float temperature =
      dhtSensors[index]->readTemperature(false, true);

  // Feuchte aus derselben Transaktion auslesen.
  const float humidity =
      dhtSensors[index]->readHumidity(false);

  const uint32_t now = millis();

  const bool finiteValues =
      !isnan(temperature) && !isinf(temperature) &&
      !isnan(humidity) && !isinf(humidity);

  const bool plausible =
      finiteValues &&
      temperature >= 0.0f && temperature <= 50.0f &&
      humidity >= 0.0f && humidity <= 100.0f;

  sensors[index].lastMeasurementValid = plausible;

  if (plausible) {
    sensors[index].temperature10 =
        static_cast<int16_t>(temperature * 10.0f + 0.5f);

    sensors[index].humidity10 =
        static_cast<uint16_t>(humidity * 10.0f + 0.5f);

    sensors[index].lastSuccessMs = now;
    sensors[index].invalidCount = 0;

    if (sensors[index].validCount < VALID_LIMIT) {
      sensors[index].validCount++;
    }

    if (sensors[index].validCount >= VALID_LIMIT) {
      setFault(index, false, 0);
    }

  } else {
    sensors[index].validCount = 0;

    if (sensors[index].invalidCount < INVALID_LIMIT) {
      sensors[index].invalidCount++;
    }

    if (sensors[index].invalidCount >= INVALID_LIMIT) {
      setFault(index, true,
               finiteValues ? ERROR_RANGE : ERROR_READ);
    }
  }

  serviceCanReceive();
}


// -------------------- Messplan --------------------

void serviceDhtMeasurements() {
  const uint32_t now = millis();

  for (uint8_t i = 0; i < 3; i++) {
    if (deadlineReached(now, sensors[i].nextMeasureMs)) {
      // Kein Nachholen mehrerer Messungen in schneller Folge.
      sensors[i].nextMeasureMs =
          now + DHT_MEASURE_INTERVAL_MS;

      measureDht(i);
      return;
    }
  }
}


// -------------------- DHT-CAN-Frames --------------------

void sendDhtFrames() {
  const uint32_t now = millis();

  for (uint8_t i = 0; i < 3; i++) {
    if (!deadlineReached(now, sensors[i].nextSendMs)) {
      continue;
    }

    sensors[i].nextSendMs = now + DHT_SEND_INTERVAL_MS;

    if (!canReady) {
      continue;
    }

    struct can_frame frame = {};

    frame.can_id = 0x103 + i;
    frame.can_dlc = 5;

    const bool valid = dhtValueValid(i, now);

    // Bei Status 1 sind die Wertefelder nicht auszuwerten.
    const uint16_t tBits = valid
        ? static_cast<uint16_t>(sensors[i].temperature10)
        : 0;

    const uint16_t hBits = valid
        ? sensors[i].humidity10
        : 0;

    frame.data[0] = static_cast<uint8_t>(tBits & 0xFF);
    frame.data[1] = static_cast<uint8_t>(tBits >> 8);

    frame.data[2] = static_cast<uint8_t>(hBits & 0xFF);
    frame.data[3] = static_cast<uint8_t>(hBits >> 8);

    frame.data[4] = valid ? 0 : 1;

    if (mcp2515.sendMessage(&frame) == MCP2515::ERROR_OK) {
      sensorTxAccepted++;
    } else {
      sensorTxErrors++;
    }
  }
}


// -------------------- Diagnosen senden --------------------

void processDiagnostics() {
  if (!canReady) {
    return;
  }

  const uint32_t now = millis();

  if (diagAttemptMade &&
      !elapsed(now, lastDiagAttemptMs, DIAG_MIN_GAP_MS)) {
    return;
  }

  const bool queued = queueCount > 0;
  DiagnosticEvent event;

  if (queued) {
    event = diagQueue[queueHead];

  } else {
    bool found = false;

    for (uint8_t n = 0; n < 4; n++) {
      const uint8_t source = (repeatCursor + n) % 4;

      if (faults[source].active &&
          elapsed(now,
                  faults[source].lastRepeatMs,
                  DIAG_REPEAT_MS)) {
        event.source = source;
        event.eventType = EVENT_ACTIVE;
        event.errorCode = faults[source].code;
        event.counter = faults[source].eventCounter;

        repeatCursor = (source + 1) % 4;
        found = true;
        break;
      }
    }

    if (!found) {
      return;
    }
  }

  struct can_frame frame = {};

  frame.can_id = 0x1F3;
  frame.can_dlc = 4;

  frame.data[0] = event.source + 1;
  frame.data[1] = event.eventType;
  frame.data[2] = event.errorCode;
  frame.data[3] = event.counter;

  lastDiagAttemptMs = now;
  diagAttemptMade = true;

  if (mcp2515.sendMessage(&frame) != MCP2515::ERROR_OK) {
    diagTxErrors++;
    return;
  }

  diagTxAccepted++;

  // Ein altes Queue-Ereignis darf den Wiederholungstimer
  // eines neueren Zustands nicht verschieben.
  if (event.counter == faults[event.source].eventCounter) {
    faults[event.source].lastRepeatMs = now;
  }

  if (queued) {
    queueHead = (queueHead + 1) % DIAG_QUEUE_SIZE;
    queueCount--;
  }
}


// -------------------- OLED vorbereiten --------------------

void prepareOledSnapshot(uint32_t now) {
  for (uint8_t i = 0; i < 3; i++) {
    char text[24];

    if (dhtValueValid(i, now)) {
      const int t = sensors[i].temperature10;
      const unsigned int h = sensors[i].humidity10;

      snprintf(
          text, sizeof(text),
          "%u %d.%dC %u.%u%%",
          static_cast<unsigned int>(i + 1),
          t / 10,
          t % 10,
          h / 10,
          h % 10);

    } else {
      snprintf(
          text, sizeof(text),
          "%u -- C -- %% ERR",
          static_cast<unsigned int>(i + 1));
    }

    memset(oledLines[i], ' ', 16);
    oledLines[i][16] = '\0';

    size_t length = strlen(text);
    if (length > 16) length = 16;

    memcpy(oledLines[i], text, length);
  }

  oledSensorIndex = 0;
  oledColumn = 0;
  oledUpdating = true;
}


// -------------------- OLED schrittweise aktualisieren --------------------

void serviceOled() {
  if (!oledReady) {
    return;
  }

  const uint32_t now = millis();

  if (!oledUpdating &&
      elapsed(now, lastOledRefreshMs, 1000)) {
    lastOledRefreshMs = now;
    prepareOledSnapshot(now);
  }

  if (!oledUpdating) {
    return;
  }

  // Pro Aufruf nur ein Zeichen uebertragen.
  const uint8_t row = 2 + oledSensorIndex * 2;

  oled.drawGlyph(
      oledColumn,
      row,
      static_cast<uint8_t>(
          oledLines[oledSensorIndex][oledColumn]));

  oledColumn++;

  if (oledColumn >= 16) {
    oledColumn = 0;
    oledSensorIndex++;

    if (oledSensorIndex >= 3) {
      oledUpdating = false;
    }
  }
}


// -------------------- Serielle Statusausgabe --------------------

void printStatus() {
  Serial.print(F("[NODE3] LDR="));
  Serial.print(ldrValue);

  Serial.print(F(" Ersatzbetrieb="));
  Serial.println(fallbackActive ? F("AN") : F("AUS"));

  for (uint8_t i = 0; i < 3; i++) {
    Serial.print(F("DHT"));
    Serial.print(i + 1);

    Serial.print(F(": gueltig="));
    Serial.print(dhtValueValid(i, millis()) ? 1 : 0);

    Serial.print(F(" Diagnose="));
    Serial.println(faults[i].active ? F("AKTIV") : F("NEIN"));
  }

  Serial.print(F("[TX] Sensor OK/Fehler="));
  Serial.print(sensorTxAccepted);
  Serial.print('/');
  Serial.print(sensorTxErrors);

  Serial.print(F(" Diagnose OK/Fehler="));
  Serial.print(diagTxAccepted);
  Serial.print('/');
  Serial.println(diagTxErrors);

  Serial.print(F("[RX] 0x101 gueltig/ungueltig="));
  Serial.print(validLdrFrames);
  Serial.print('/');
  Serial.println(invalidLdrFrames);

  Serial.print(F("[QUEUE] Ueberlaeufe="));
  Serial.println(queueOverflows);

  Serial.print(F("[RAM] Momentan frei="));
  Serial.println(freeRamNow());

  if (canReady) {
    Serial.print(F("[CAN] EFLG=0x"));
    Serial.print(mcp2515.getErrorFlags(), HEX);

    Serial.print(F(" TEC="));
    Serial.print(mcp2515.errorCountTX());

    Serial.print(F(" REC="));
    Serial.println(mcp2515.errorCountRX());
  }
}


// -------------------- Setup --------------------

void setup() {
  Serial.begin(115200);

  pinMode(PIN_CAN_INT, INPUT_PULLUP);

  dht1.begin();
  dht2.begin();
  dht3.begin();

  pixels.begin();
  updatePixels();

  SPI.begin();
  canReady = initializeCan();

  attachInterrupt(
      digitalPinToInterrupt(PIN_CAN_INT),
      onCanInterrupt,
      FALLING);

  Wire.begin();

  // Begrenzte I2C-Wartezeit, sofern vom AVR-Core unterstuetzt.
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);
#endif

  Wire.beginTransmission(OLED_ADDRESS);
  oledReady = Wire.endTransmission() == 0;

  if (oledReady) {
    oled.setI2CAddress(OLED_ADDRESS << 1);
    oled.setBusClock(100000);

    oled.begin();
    oled.setFont(u8x8_font_chroma48medium8_r);
    oled.drawString(0, 0, "CAN-Guard Node3");
  }

  const uint32_t now = millis();

  ldrWatchStartedMs = now;
  lastCanInitAttemptMs = now;
  lastOledRefreshMs = now;

  // Messungen nach 1000 / 1333 / 1666 ms.
  // CAN-Slots jeweils 50 ms spaeter.
  const uint16_t offsets[3] = {0, 333, 666};

  for (uint8_t i = 0; i < 3; i++) {
    sensors[i].nextMeasureMs = now + 1000UL + offsets[i];
    sensors[i].nextSendMs = now + 1050UL + offsets[i];
  }

  Serial.println(F("[NODE3] DHT D3/D4/D5, NeoPixel D6, CAN-CS D10"));
  Serial.println(F("[NODE3] CAN-INT D2, OLED A4/A5"));
  Serial.println(F("[NODE3] DHT: 0x103-0x105 / DLC 5"));
  Serial.println(F("[NODE3] Diagnose: 0x1F3 / DLC 4"));

  Serial.println(
      canReady ? F("[CAN] Bereit.") : F("[CAN] Initialisierung fehlgeschlagen."));

  Serial.println(
      oledReady ? F("[OLED] Gefunden.") : F("[OLED] Nicht gefunden."));

  Serial.println(F("[SERIAL] s = Status"));
}


// -------------------- Hauptschleife --------------------

void loop() {
  serviceCanReceive();

  uint32_t now = millis();

  updateLdrTimeout(now);
  updatePixels();

  if (!canReady &&
      elapsed(now, lastCanInitAttemptMs, CAN_RETRY_MS)) {
    lastCanInitAttemptMs = now;
    canReady = initializeCan();
  }

  serviceDhtMeasurements();

  serviceCanReceive();

  now = millis();
  updateLdrTimeout(now);
  updatePixels();

  sendDhtFrames();
  processDiagnostics();

  serviceCanReceive();
  serviceOled();
  serviceCanReceive();

  if (Serial.available() > 0) {
    const char command = static_cast<char>(Serial.read());

    if (command == 's' || command == 'S') {
      printStatus();
    }
  }
}