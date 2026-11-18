#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

// ============================================================
// CAN-Guard - Node 4
// Arduino Nano V3 + SR602 + HW-184
//
// Pins:
//   SR602 OUT: D3
//   LED blau:  D5, ueber 330 Ohm
//   LED gelb:  D6, ueber 330 Ohm
//   CAN CS:    D10
//   CAN INT:   D2, hier nicht ausgewertet
//   SPI:       D11 MOSI / D12 MISO / D13 SCK
//
// Sensornachricht:
//   ID 0x106, DLC 2
//   B0: Bewegung, 0 = nein, 1 = ja
//   B1: Status, 0 = bereit, 1 = Anlaufphase
//
// Diagnose:
//   ID 0x1F4, DLC 4
//   B0: Sensor-ID 1 = SR602
//   B1: 1 = nicht verfuegbar, 0 = wieder verfuegbar
//   B2: 2 = noch keine nutzbaren Daten
//   B3: Ereigniszaehler
//
// Diagnose betrifft hier ausschliesslich die Anlaufphase.
// Keine sichere Erkennung eines defekten digitalen Ausgangs.
//
// Serieller Monitor, 115200 Baud:
//   s = Status anzeigen
// ============================================================


// -------------------- Pins --------------------

constexpr uint8_t PIN_PIR = 3;
constexpr uint8_t PIN_LED_BLUE = 5;
constexpr uint8_t PIN_LED_YELLOW = 6;
constexpr uint8_t PIN_CAN_CS = 10;
constexpr uint8_t PIN_CAN_INT = 2;

MCP2515 mcp2515(PIN_CAN_CS);


// -------------------- CAN-Protokoll --------------------

constexpr uint16_t CAN_ID_SENSOR = 0x106;
constexpr uint16_t CAN_ID_DIAG = 0x1F4;

constexpr uint8_t SENSOR_ID = 1;

constexpr uint8_t EVENT_CLEARED = 0x00;
constexpr uint8_t EVENT_ACTIVE = 0x01;
constexpr uint8_t ERROR_NO_DATA = 0x02;


// -------------------- Zeitparameter --------------------

constexpr uint32_t WARMUP_MS = 60000;
constexpr uint32_t MOTION_HOLD_MS = 3000;

constexpr uint32_t SENSOR_SEND_INTERVAL_MS = 1000;
constexpr uint32_t SENSOR_RETRY_MS = 100;

constexpr uint32_t YELLOW_BLINK_HALF_PERIOD_MS = 500;

constexpr uint32_t DIAG_CHECK_INTERVAL_MS = 100;
constexpr uint32_t DIAG_REPEAT_MS = 5000;
constexpr uint32_t DIAG_MIN_GAP_MS = 200;

constexpr uint32_t CAN_INIT_RETRY_MS = 2000;

constexpr uint8_t INVALID_LIMIT = 3;
constexpr uint8_t VALID_LIMIT = 3;
constexpr uint8_t DIAG_QUEUE_SIZE = 4;

// true:
// Anlaufphase nach drei Checks diagnostizieren.
// false:
// Anlaufphase nur im Statusbyte von 0x106 anzeigen.
constexpr bool DIAGNOSE_WARMUP = true;


// -------------------- Lokaler Zustand --------------------

uint32_t startupMs = 0;

bool warmingUp = true;
bool rawMotion = false;
bool motionActive = false;

bool motionHighSeen = false;
uint32_t lastMotionHighMs = 0;


// -------------------- CAN-Zeitverwaltung --------------------

bool canReady = false;
uint32_t lastCanInitAttemptMs = 0;

bool sensorPending = true;
bool sensorAttemptMade = false;
bool sensorAcceptedOnce = false;

uint32_t lastSensorAttemptMs = 0;
uint32_t lastSensorAcceptedMs = 0;


// -------------------- Diagnosezustand --------------------

bool startupDiagnosticActive = false;

uint8_t invalidCount = 0;
uint8_t validCount = 0;
uint8_t eventCounter = 0;

uint32_t lastDiagCheckMs = 0;

bool diagAttemptMade = false;
uint32_t lastDiagAttemptMs = 0;
uint32_t lastDiagAcceptedMs = 0;


// -------------------- Diagnosewarteschlange --------------------

struct DiagnosticEvent {
  uint8_t eventType;
  uint8_t counter;
};

DiagnosticEvent diagQueue[DIAG_QUEUE_SIZE];

uint8_t queueHead = 0;
uint8_t queueCount = 0;

uint32_t queueOverflows = 0;


// -------------------- Statistik --------------------

// "Accepted": sendMessage() lieferte ERROR_OK.
// Kein Ende-zu-Ende-Nachweis fuer Gateway-Empfang.
uint32_t sensorTxAccepted = 0;
uint32_t sensorTxErrors = 0;

uint32_t diagTxAccepted = 0;
uint32_t diagTxErrors = 0;


// -------------------- Zeitvergleich --------------------

bool elapsed(uint32_t now,
             uint32_t previous,
             uint32_t interval) {
  return static_cast<uint32_t>(now - previous) >= interval;
}


// -------------------- CAN initialisieren --------------------

bool initializeCan() {
  if (mcp2515.reset() != MCP2515::ERROR_OK) {
    Serial.println(F("[CAN] Reset fehlgeschlagen."));
    return false;
  }

  if (mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ)
      != MCP2515::ERROR_OK) {
    Serial.println(F("[CAN] Bitrate fehlgeschlagen."));
    return false;
  }

  if (mcp2515.setNormalMode() != MCP2515::ERROR_OK) {
    Serial.println(F("[CAN] Normalmodus fehlgeschlagen."));
    return false;
  }

  Serial.println(F("[CAN] Bereit: 500 kbit/s, Quarz 8 MHz."));
  return true;
}


// -------------------- Diagnose vormerken --------------------

void enqueueDiagnostic(uint8_t eventType) {
  if (queueCount == DIAG_QUEUE_SIZE) {
    queueHead =
        static_cast<uint8_t>((queueHead + 1) % DIAG_QUEUE_SIZE);

    queueCount--;
    queueOverflows++;
  }

  const uint8_t tail =
      static_cast<uint8_t>(
          (queueHead + queueCount) % DIAG_QUEUE_SIZE);

  eventCounter++;

  diagQueue[tail].eventType = eventType;
  diagQueue[tail].counter = eventCounter;

  queueCount++;
}


// -------------------- Diagnosezustand aendern --------------------

void setStartupDiagnostic(bool active) {
  if (active == startupDiagnosticActive) {
    return;
  }

  startupDiagnosticActive = active;

  enqueueDiagnostic(active ? EVENT_ACTIVE : EVENT_CLEARED);
}


// -------------------- Lokalen Zustand aktualisieren --------------------

void updateLocalState(uint32_t now) {
  const bool previousWarmup = warmingUp;
  const bool previousMotion = motionActive;

  rawMotion = digitalRead(PIN_PIR) == HIGH;

  // Einmal abgeschlossene Anlaufphase bleibt abgeschlossen.
  if (warmingUp && elapsed(now, startupMs, WARMUP_MS)) {
    warmingUp = false;
  }

  if (warmingUp) {
    motionActive = false;
    motionHighSeen = false;

  } else {
    if (rawMotion) {
      motionHighSeen = true;
      lastMotionHighMs = now;
    }

    // Nachlaufzeit beginnt, wenn der Ausgang nicht mehr HIGH ist.
    motionActive =
        motionHighSeen &&
        (rawMotion ||
         !elapsed(now, lastMotionHighMs, MOTION_HOLD_MS));
  }

  if (previousWarmup != warmingUp ||
      previousMotion != motionActive) {
    sensorPending = true;
  }

  // Lokale LEDs arbeiten unabhaengig von CAN.
  digitalWrite(
      PIN_LED_BLUE,
      (!warmingUp && motionActive) ? HIGH : LOW);

  if (warmingUp) {
    const bool yellowOn =
        ((static_cast<uint32_t>(now - startupMs) /
          YELLOW_BLINK_HALF_PERIOD_MS) % 2UL) == 0;

    digitalWrite(PIN_LED_YELLOW, yellowOn ? HIGH : LOW);

  } else {
    digitalWrite(
        PIN_LED_YELLOW,
        motionActive ? LOW : HIGH);
  }
}


// -------------------- Diagnose entprellen --------------------

void updateDiagnosticState(uint32_t now) {
  if (!DIAGNOSE_WARMUP ||
      !elapsed(now, lastDiagCheckMs, DIAG_CHECK_INTERVAL_MS)) {
    return;
  }

  lastDiagCheckMs = now;

  if (warmingUp) {
    validCount = 0;

    if (invalidCount < INVALID_LIMIT) {
      invalidCount++;
    }

    if (invalidCount >= INVALID_LIMIT) {
      setStartupDiagnostic(true);
    }

  } else {
    invalidCount = 0;

    if (validCount < VALID_LIMIT) {
      validCount++;
    }

    if (validCount >= VALID_LIMIT) {
      setStartupDiagnostic(false);
    }
  }
}


// -------------------- Sensornachricht senden --------------------

void sendSensorFrame(uint32_t now) {
  if (!canReady) {
    return;
  }

  const bool cyclicDue =
      !sensorAcceptedOnce ||
      elapsed(
          now,
          lastSensorAcceptedMs,
          SENSOR_SEND_INTERVAL_MS);

  if (!sensorPending && !cyclicDue) {
    return;
  }

  // Fehlgeschlagene Versuche nicht in jeder Loop wiederholen.
  if (sensorAttemptMade &&
      elapsed(now, lastSensorAttemptMs, SENSOR_RETRY_MS) == false) {
    return;
  }

  struct can_frame frame = {};

  frame.can_id = CAN_ID_SENSOR;
  frame.can_dlc = 2;

  frame.data[0] = (!warmingUp && motionActive) ? 1 : 0;
  frame.data[1] = warmingUp ? 1 : 0;

  sensorAttemptMade = true;
  lastSensorAttemptMs = now;

  if (mcp2515.sendMessage(&frame) != MCP2515::ERROR_OK) {
    sensorTxErrors++;
    sensorPending = true;
    return;
  }

  sensorTxAccepted++;
  sensorAcceptedOnce = true;
  lastSensorAcceptedMs = now;
  sensorPending = false;
}


// -------------------- Diagnosen senden --------------------

void processDiagnostics(uint32_t now) {
  if (!canReady) {
    return;
  }

  if (diagAttemptMade &&
      !elapsed(now, lastDiagAttemptMs, DIAG_MIN_GAP_MS)) {
    return;
  }

  const bool queued = queueCount > 0;

  const bool repeatDue =
      startupDiagnosticActive &&
      elapsed(now, lastDiagAcceptedMs, DIAG_REPEAT_MS);

  if (!queued && !repeatDue) {
    return;
  }

  DiagnosticEvent event;

  if (queued) {
    event = diagQueue[queueHead];
  } else {
    event.eventType = EVENT_ACTIVE;
    event.counter = eventCounter;
  }

  struct can_frame frame = {};

  frame.can_id = CAN_ID_DIAG;
  frame.can_dlc = 4;

  frame.data[0] = SENSOR_ID;
  frame.data[1] = event.eventType;
  frame.data[2] = ERROR_NO_DATA;
  frame.data[3] = event.counter;

  diagAttemptMade = true;
  lastDiagAttemptMs = now;

  if (mcp2515.sendMessage(&frame) != MCP2515::ERROR_OK) {
    diagTxErrors++;
    return;
  }

  diagTxAccepted++;
  lastDiagAcceptedMs = now;

  if (queued) {
    queueHead =
        static_cast<uint8_t>((queueHead + 1) % DIAG_QUEUE_SIZE);

    queueCount--;
  }

  Serial.print(F("[DIAG] 0x1F4 Event=0x"));
  Serial.print(event.eventType, HEX);

  Serial.print(F(" Code=0x"));
  Serial.print(ERROR_NO_DATA, HEX);

  Serial.print(F(" Counter="));
  Serial.println(event.counter);
}


// -------------------- Empfangspuffer leeren --------------------

void serviceCanReceive() {
  if (!canReady) {
    return;
  }

  struct can_frame frame;

  // Node 4 verarbeitet derzeit keine Eingangssignale.
  for (uint8_t i = 0; i < 2; i++) {
    if (mcp2515.readMessage(&frame) != MCP2515::ERROR_OK) {
      break;
    }
  }
}


// -------------------- Status anzeigen --------------------

void printStatus() {
  Serial.print(F("[NODE4] Zustand="));

  if (warmingUp) {
    Serial.print(F("ANLAUFPHASE"));
  } else if (motionActive) {
    Serial.print(F("BEWEGUNG"));
  } else {
    Serial.print(F("RUHE"));
  }

  Serial.print(F(" PIR-Rohsignal="));
  Serial.println(rawMotion ? 1 : 0);

  Serial.print(F("[TX] Sensor akzeptiert/Fehler="));
  Serial.print(sensorTxAccepted);
  Serial.print('/');
  Serial.print(sensorTxErrors);

  Serial.print(F(" Diagnose akzeptiert/Fehler="));
  Serial.print(diagTxAccepted);
  Serial.print('/');
  Serial.println(diagTxErrors);

  Serial.print(F("[QUEUE] Belegt="));
  Serial.print(queueCount);

  Serial.print(F(" Ueberlaeufe="));
  Serial.println(queueOverflows);

  if (canReady) {
    Serial.print(F("[CAN] EFLG=0x"));
    Serial.print(mcp2515.getErrorFlags(), HEX);

    Serial.print(F(" TEC="));
    Serial.print(mcp2515.errorCountTX());

    Serial.print(F(" REC="));
    Serial.println(mcp2515.errorCountRX());

  } else {
    Serial.println(F("[CAN] Nicht initialisiert."));
  }
}


// -------------------- Setup --------------------

void setup() {
  Serial.begin(115200);

  pinMode(PIN_PIR, INPUT);
  pinMode(PIN_LED_BLUE, OUTPUT);
  pinMode(PIN_LED_YELLOW, OUTPUT);
  pinMode(PIN_CAN_INT, INPUT_PULLUP);

  digitalWrite(PIN_LED_BLUE, LOW);
  digitalWrite(PIN_LED_YELLOW, HIGH);

  startupMs = millis();
  lastDiagCheckMs = startupMs;

  SPI.begin();

  canReady = initializeCan();
  lastCanInitAttemptMs = millis();

  Serial.println(F("[NODE4] SR602 D3, LED blau D5, LED gelb D6"));
  Serial.println(F("[NODE4] 0x106 / DLC 2 / 1000 ms + Aenderung"));
  Serial.println(F("[NODE4] Diagnose 0x1F4 / DLC 4"));
  Serial.println(F("[NODE4] Anlaufzeit 60 s, Nachlaufzeit 3 s"));
  Serial.println(F("[SERIAL] s = Status"));
}


// -------------------- Hauptschleife --------------------

void loop() {
  uint32_t now = millis();

  // Lokale Funktionen zuerst, unabhaengig von CAN.
  updateLocalState(now);
  updateDiagnosticState(now);

  if (!canReady &&
      elapsed(now, lastCanInitAttemptMs, CAN_INIT_RETRY_MS)) {
    lastCanInitAttemptMs = now;
    canReady = initializeCan();
  }

  now = millis();

  serviceCanReceive();
  sendSensorFrame(now);
  processDiagnostics(now);

  if (Serial.available() > 0) {
    const char command = static_cast<char>(Serial.read());

    if (command == 's' || command == 'S') {
      printStatus();
    }
  }
}