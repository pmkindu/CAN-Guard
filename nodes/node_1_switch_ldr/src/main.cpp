#include <SPI.h>
#include <mcp2515.h>

// ============================================================
// CAN-Guard - Node 1
// Arduino Nano V3 + HW-184, MCP2515 mit 8-MHz-Quarz
//
// 0x101 / DLC 3 / 200 ms:
//   B0:    Schalter, 0 = frei, 1 = betaetigt
//   B1-B2: LDR-Rohwert, uint16, Little Endian, 0-1023
//
// 0x1F1 / DLC 4:
//   B0: Sensor-ID, 2 = LDR
//   B1: Ereignis, 1 = aktiv, 0 = behoben
//   B2: Fehlercode, 1 = Messwert ungueltig
//   B3: Ereigniszaehler, steigt nur bei Zustandswechseln
//
// Serieller Monitor, 115200 Baud:
//   f = LDR-Fehler simulieren
//   n = Simulation beenden
//   s = Status anzeigen
//
// Wichtig:
// Die Simulation testet die Diagnosekette.
// Sie ist KEINE reale Leitungsbrucherkennung des LDR.
// ============================================================


// -------------------- Hardware --------------------

constexpr uint8_t PIN_SWITCH = 3;
constexpr uint8_t PIN_LDR = A0;
constexpr uint8_t PIN_CS = 10;

MCP2515 mcp2515(PIN_CS);


// -------------------- CAN-Protokoll --------------------

constexpr uint16_t CAN_ID_SENSOR = 0x101;
constexpr uint16_t CAN_ID_DIAG = 0x1F1;

constexpr uint8_t SENSOR_ID_LDR = 2;

constexpr uint8_t EVENT_CLEARED = 0x00;
constexpr uint8_t EVENT_ACTIVE = 0x01;

constexpr uint8_t ERROR_INVALID_VALUE = 0x01;


// -------------------- Zeitparameter --------------------

constexpr uint32_t SEND_INTERVAL_MS = 200;
constexpr uint32_t LDR_SAMPLE_INTERVAL_MS = 100;
constexpr uint32_t SWITCH_DEBOUNCE_MS = 30;

constexpr uint32_t DIAG_REPEAT_INTERVAL_MS = 5000;
constexpr uint32_t DIAG_MIN_INTERVAL_MS = 200;

constexpr uint32_t CAN_INIT_RETRY_MS = 2000;
constexpr uint32_t STATUS_INTERVAL_MS = 5000;


// -------------------- Diagnosekonfiguration --------------------

constexpr uint8_t INVALID_LIMIT = 3;
constexpr uint8_t VALID_LIMIT = 3;

constexpr uint8_t DIAG_QUEUE_SIZE = 4;

// Fuer normalen Betrieb ohne Fehlertest auf false setzen.
constexpr bool ENABLE_FAULT_INJECTION = true;


// -------------------- Sensorzustand --------------------

bool switchPressed = false;
bool lastRawSwitchPressed = false;
uint32_t switchChangedAt = 0;

uint16_t ldrValue = 0;

bool simulateLdrError = false;
bool ldrErrorActive = false;

uint8_t invalidCount = 0;
uint8_t validCount = 0;

uint8_t diagEventCounter = 0;


// -------------------- Diagnosewarteschlange --------------------

struct DiagnosticEvent {
  uint8_t sensorId;
  uint8_t eventType;
  uint8_t errorCode;
  uint8_t eventCounter;
};

DiagnosticEvent diagQueue[DIAG_QUEUE_SIZE];

uint8_t diagQueueHead = 0;
uint8_t diagQueueCount = 0;

uint32_t diagQueueOverflows = 0;


// -------------------- CAN- und Zeitstatus --------------------

bool canReady = false;

uint32_t lastSendTime = 0;
uint32_t lastLdrSampleTime = 0;
uint32_t lastCanInitAttemptTime = 0;
uint32_t lastStatusTime = 0;

uint32_t lastDiagAttemptTime = 0;
uint32_t lastDiagAcceptedTime = 0;

bool diagAttemptMade = false;


// -------------------- Statistik --------------------

// "Accepted" bedeutet ERROR_OK von sendMessage().
// Kein Empfangsnachweis durch das Gateway.
uint32_t sensorTxAccepted = 0;
uint32_t sensorTxErrors = 0;

uint32_t diagTxAccepted = 0;
uint32_t diagTxErrors = 0;


// -------------------- Zeitvergleich --------------------

bool intervalElapsed(uint32_t now,
                     uint32_t previous,
                     uint32_t interval) {
  return static_cast<uint32_t>(now - previous) >= interval;
}


// -------------------- CAN initialisieren --------------------

bool initializeCan() {
  MCP2515::ERROR result = mcp2515.reset();

  if (result != MCP2515::ERROR_OK) {
    Serial.println(F("[CAN] Reset fehlgeschlagen."));
    return false;
  }

  result = mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);

  if (result != MCP2515::ERROR_OK) {
    Serial.println(F("[CAN] Bitrate konnte nicht gesetzt werden."));
    return false;
  }

  result = mcp2515.setNormalMode();

  if (result != MCP2515::ERROR_OK) {
    Serial.println(F("[CAN] Normalmodus fehlgeschlagen."));
    return false;
  }

  Serial.println(F("[CAN] Bereit: 500 kbit/s, 8-MHz-Quarz."));
  return true;
}


// -------------------- Schalter entprellen --------------------

void updateSwitch(uint32_t now) {
  const bool rawPressed = digitalRead(PIN_SWITCH) == LOW;

  if (rawPressed != lastRawSwitchPressed) {
    lastRawSwitchPressed = rawPressed;
    switchChangedAt = now;
  }

  if (rawPressed != switchPressed &&
      intervalElapsed(now, switchChangedAt, SWITCH_DEBOUNCE_MS)) {
    switchPressed = rawPressed;
  }
}


// -------------------- Diagnose vormerken --------------------

void enqueueDiagnostic(uint8_t eventType) {
  if (diagQueueCount == DIAG_QUEUE_SIZE) {
    // Bei Ueberlauf aeltestes Ereignis verwerfen.
    // Neuester Zustand bleibt dadurch erhalten.
    diagQueueHead =
        static_cast<uint8_t>((diagQueueHead + 1) % DIAG_QUEUE_SIZE);

    diagQueueCount--;
    diagQueueOverflows++;
  }

  const uint8_t tail =
      static_cast<uint8_t>(
          (diagQueueHead + diagQueueCount) % DIAG_QUEUE_SIZE);

  diagQueue[tail].sensorId = SENSOR_ID_LDR;
  diagQueue[tail].eventType = eventType;
  diagQueue[tail].errorCode = ERROR_INVALID_VALUE;
  diagQueue[tail].eventCounter = diagEventCounter;

  diagQueueCount++;
}


// -------------------- Fehlerzustand wechseln --------------------

void setLdrError(bool active) {
  if (active == ldrErrorActive) {
    return;
  }

  ldrErrorActive = active;

  // uint8_t laeuft nach 255 auf 0 ueber.
  diagEventCounter++;

  enqueueDiagnostic(active ? EVENT_ACTIVE : EVENT_CLEARED);

  Serial.print(F("[DIAG] LDR "));
  Serial.print(active ? F("FEHLER AKTIV") : F("FEHLER BEHOBEN"));
  Serial.print(F(", Ereignis="));
  Serial.println(diagEventCounter);
}


// -------------------- LDR erfassen --------------------

void updateLdr(uint32_t now) {
  if (!intervalElapsed(now,
                       lastLdrSampleTime,
                       LDR_SAMPLE_INTERVAL_MS)) {
    return;
  }

  lastLdrSampleTime = now;

  // Tatsächlichen ADC-Rohwert immer erfassen.
  ldrValue = static_cast<uint16_t>(analogRead(PIN_LDR));

  // Keine scheinbare Defekterkennung anhand von 0 oder 1023.
  const bool invalid =
      ENABLE_FAULT_INJECTION && simulateLdrError;

  if (invalid) {
    validCount = 0;

    if (invalidCount < INVALID_LIMIT) {
      invalidCount++;
    }

    if (invalidCount >= INVALID_LIMIT) {
      setLdrError(true);
    }
  } else {
    invalidCount = 0;

    if (validCount < VALID_LIMIT) {
      validCount++;
    }

    if (validCount >= VALID_LIMIT) {
      setLdrError(false);
    }
  }
}


// -------------------- Sensornachricht senden --------------------

void sendSensorFrame(uint32_t now) {
  if (!canReady ||
      !intervalElapsed(now, lastSendTime, SEND_INTERVAL_MS)) {
    return;
  }

  lastSendTime = now;

  struct can_frame frame = {};

  frame.can_id = CAN_ID_SENSOR;
  frame.can_dlc = 3;

  frame.data[0] = switchPressed ? 1 : 0;
  frame.data[1] = static_cast<uint8_t>(ldrValue & 0xFF);
  frame.data[2] = static_cast<uint8_t>((ldrValue >> 8) & 0xFF);

  const MCP2515::ERROR result = mcp2515.sendMessage(&frame);

  if (result == MCP2515::ERROR_OK) {
    sensorTxAccepted++;
  } else {
    sensorTxErrors++;
  }
}


// -------------------- Diagnose senden --------------------

void processDiagnostics(uint32_t now) {
  if (!canReady) {
    return;
  }

  // Mindestabstand gilt auch bei fehlgeschlagenen Versuchen.
  if (diagAttemptMade &&
      !intervalElapsed(now,
                       lastDiagAttemptTime,
                       DIAG_MIN_INTERVAL_MS)) {
    return;
  }

  const bool hasQueuedEvent = diagQueueCount > 0;

  const bool repeatDue =
      ldrErrorActive &&
      intervalElapsed(now,
                      lastDiagAcceptedTime,
                      DIAG_REPEAT_INTERVAL_MS);

  if (!hasQueuedEvent && !repeatDue) {
    return;
  }

  DiagnosticEvent event;

  if (hasQueuedEvent) {
    event = diagQueue[diagQueueHead];
  } else {
    // Wiederholung des aktuellen aktiven Zustands.
    // Ereigniszaehler bleibt unveraendert.
    event.sensorId = SENSOR_ID_LDR;
    event.eventType = EVENT_ACTIVE;
    event.errorCode = ERROR_INVALID_VALUE;
    event.eventCounter = diagEventCounter;
  }

  struct can_frame frame = {};

  frame.can_id = CAN_ID_DIAG;
  frame.can_dlc = 4;

  frame.data[0] = event.sensorId;
  frame.data[1] = event.eventType;
  frame.data[2] = event.errorCode;
  frame.data[3] = event.eventCounter;

  lastDiagAttemptTime = now;
  diagAttemptMade = true;

  const MCP2515::ERROR result = mcp2515.sendMessage(&frame);

  if (result != MCP2515::ERROR_OK) {
    diagTxErrors++;

    // Ereignis bleibt in der Warteschlange.
    return;
  }

  diagTxAccepted++;
  lastDiagAcceptedTime = now;

  if (hasQueuedEvent) {
    diagQueueHead =
        static_cast<uint8_t>((diagQueueHead + 1) % DIAG_QUEUE_SIZE);
    diagQueueCount--;
  }

  Serial.print(F("[DIAG TX] 0x1F1: Sensor="));
  Serial.print(event.sensorId);

  Serial.print(F(" Event=0x"));
  Serial.print(event.eventType, HEX);

  Serial.print(F(" Code=0x"));
  Serial.print(event.errorCode, HEX);

  Serial.print(F(" Counter="));
  Serial.println(event.eventCounter);
}


// -------------------- Empfangspuffer leeren --------------------

void serviceCanReceive() {
  if (!canReady) {
    return;
  }

  // Node 1 verarbeitet aktuell keine Eingangsnachrichten.
  // Trotzdem Empfangspuffer regelmaessig leeren.
  struct can_frame receivedFrame;

  for (uint8_t i = 0; i < 2; i++) {
    if (mcp2515.readMessage(&receivedFrame) != MCP2515::ERROR_OK) {
      break;
    }
  }
}


// -------------------- Status ausgeben --------------------

void printStatus() {
  Serial.print(F("[STATUS] Switch="));
  Serial.print(switchPressed ? 1 : 0);

  Serial.print(F(" LDR="));
  Serial.print(ldrValue);

  Serial.print(F(" Test="));
  Serial.print(simulateLdrError ? F("AN") : F("AUS"));

  Serial.print(F(" Fehler="));
  Serial.println(ldrErrorActive ? F("AKTIV") : F("NEIN"));

  Serial.print(F("[TX] Sensor akzeptiert/Fehler="));
  Serial.print(sensorTxAccepted);
  Serial.print('/');
  Serial.print(sensorTxErrors);

  Serial.print(F(" Diagnose akzeptiert/Fehler="));
  Serial.print(diagTxAccepted);
  Serial.print('/');
  Serial.println(diagTxErrors);

  Serial.print(F("[QUEUE] Belegt="));
  Serial.print(diagQueueCount);
  Serial.print(F(" Ueberlaeufe="));
  Serial.println(diagQueueOverflows);

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


// -------------------- Serielle Befehle --------------------

void processSerial() {
  // Begrenztes Arbeitsbudget je Schleifendurchlauf.
  for (uint8_t i = 0; i < 8 && Serial.available() > 0; i++) {
    const char command = static_cast<char>(Serial.read());

    switch (command) {
      case 'f':
      case 'F':
        if (ENABLE_FAULT_INJECTION) {
          simulateLdrError = true;
          Serial.println(F("[TEST] LDR-Fehlersimulation AN."));
        } else {
          Serial.println(F("[TEST] Fehlersimulation deaktiviert."));
        }
        break;

      case 'n':
      case 'N':
        simulateLdrError = false;
        Serial.println(F("[TEST] LDR-Fehlersimulation AUS."));
        break;

      case 's':
      case 'S':
        printStatus();
        break;

      default:
        break;
    }
  }
}


// -------------------- Setup --------------------

void setup() {
  Serial.begin(115200);

  pinMode(PIN_SWITCH, INPUT_PULLUP);
  pinMode(PIN_LDR, INPUT);

  const uint32_t now = millis();

  lastRawSwitchPressed = digitalRead(PIN_SWITCH) == LOW;
  switchPressed = lastRawSwitchPressed;
  switchChangedAt = now;

  ldrValue = static_cast<uint16_t>(analogRead(PIN_LDR));

  lastSendTime = now;
  lastLdrSampleTime = now;
  lastCanInitAttemptTime = now;
  lastStatusTime = now;

  SPI.begin();

  canReady = initializeCan();

  Serial.println(F("[NODE1] Schalter + LDR"));
  Serial.println(F("[NODE1] Sensor: 0x101, DLC 3, 200 ms"));
  Serial.println(F("[NODE1] Diagnose: 0x1F1, DLC 4"));
  Serial.println(F("[SERIAL] f=Fehler, n=Normal, s=Status"));
}


// -------------------- Hauptschleife --------------------

void loop() {
  processSerial();

  const uint32_t now = millis();

  updateSwitch(now);
  updateLdr(now);

  if (!canReady &&
      intervalElapsed(now,
                      lastCanInitAttemptTime,
                      CAN_INIT_RETRY_MS)) {
    lastCanInitAttemptTime = now;
    canReady = initializeCan();
  }

  serviceCanReceive();

  // Sensornachrichten vor Diagnosen bearbeiten.
  sendSensorFrame(now);
  processDiagnostics(now);

  if (intervalElapsed(now, lastStatusTime, STATUS_INTERVAL_MS)) {
    lastStatusTime = now;
    printStatus();
  }
}