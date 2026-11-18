#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <Adafruit_NeoPixel.h>

// ============================================================
// CAN-Guard - Node 2 (Roadmap v4.2)
// Arduino Nano V3 / ATmega328P + HW-184 MCP2515 (8 MHz)
// 
// Verkabelung:
//   HC-SR04 Trig: D4
//   HC-SR04 Echo: D5 (PCINT21)
//   WS2812 Ring:  D6 (16x 5050, integrierter Treiber)
//   MCP2515 CS:   D10
//   MCP2515 INT:  D2 (in dieser Firmware gepollt/optional)
//
// Sensornachricht 0x102, DLC 2, alle 250 ms:
//   B0-B1: Distanz in cm (uint16, Little Endian)
//   0xFFFF: kein gueltiger Distanzwert
//
// Diagnose 0x1F2, DLC 4:
//   B0: Sensor-ID = 1
//   B1: 1 = Fehler aktiv, 0 = Fehler behoben
//   B2: 2 = kein Echo/Timeout, 3 = ausserhalb Bereich
//   B3: Ereigniszaehler
//
// Serieller Monitor (115200 Baud):
//   s = Status (inkl. CAN-Infos und Ultraschallwert) anzeigen
// ============================================================

// -------------------- Hardware & Pins --------------------

constexpr uint8_t PIN_TRIG = 4;
constexpr uint8_t PIN_ECHO = 5;
constexpr uint8_t PIN_LED_RING = 6;
constexpr uint8_t PIN_CS = 10;

constexpr uint16_t NUM_LEDS = 16;
Adafruit_NeoPixel strip(NUM_LEDS, PIN_LED_RING, NEO_GRB + NEO_KHZ800);

MCP2515 mcp2515(PIN_CS);

// -------------------- CAN-Protokoll --------------------

constexpr uint16_t CAN_ID_SENSOR = 0x102;
constexpr uint16_t CAN_ID_DIAG = 0x1F2;

constexpr uint8_t SENSOR_ID = 1;

constexpr uint8_t EVENT_CLEARED = 0x00;
constexpr uint8_t EVENT_ACTIVE = 0x01;

constexpr uint8_t ERROR_TIMEOUT = 0x02;
constexpr uint8_t ERROR_RANGE = 0x03;

constexpr uint16_t INVALID_DISTANCE = 0xFFFF;

// -------------------- Messparameter --------------------

constexpr uint32_t MEASURE_INTERVAL_MS = 250;
constexpr uint32_t SEND_INTERVAL_MS = 250;
constexpr uint32_t ECHO_TIMEOUT_US = 30000;

constexpr uint16_t MIN_DISTANCE_CM = 2;
constexpr uint16_t MAX_DISTANCE_CM = 400;

// -------------------- Diagnoseparameter --------------------

constexpr uint8_t INVALID_LIMIT = 3;
constexpr uint8_t VALID_LIMIT = 3;

constexpr uint32_t DIAG_REPEAT_MS = 5000;
constexpr uint32_t DIAG_MIN_GAP_MS = 200;

constexpr uint8_t DIAG_QUEUE_SIZE = 4;

// -------------------- Weitere Zeitparameter --------------------

constexpr uint32_t INIT_RETRY_MS = 2000;
constexpr uint32_t STATUS_INTERVAL_MS = 5000;

// -------------------- Echo-Erfassung --------------------

enum EchoState : uint8_t {
  ECHO_IDLE,
  ECHO_WAIT_RISE,
  ECHO_WAIT_FALL,
  ECHO_COMPLETE
};

volatile uint8_t echoState = ECHO_IDLE;
volatile uint32_t echoRiseUs = 0;
volatile uint32_t echoWidthUs = 0;

bool measurementRunning = false;
uint32_t measurementStartedUs = 0;

// -------------------- Messwerte und Fehlerzustand --------------------

uint16_t distanceCm = INVALID_DISTANCE;
uint32_t lastPulseUs = 0;

uint8_t invalidCount = 0;
uint8_t validCount = 0;

bool sensorErrorActive = false;
uint8_t activeErrorCode = ERROR_TIMEOUT;
uint8_t eventCounter = 0;

// -------------------- Diagnosewarteschlange --------------------

struct DiagnosticEvent {
  uint8_t eventType;
  uint8_t errorCode;
  uint8_t counter;
};

DiagnosticEvent diagQueue[DIAG_QUEUE_SIZE];
uint8_t queueHead = 0;
uint8_t queueCount = 0;
uint32_t queueOverflows = 0;

// -------------------- CAN- und Zeitstatus --------------------

bool canReady = false;
bool diagAttemptMade = false;

uint32_t lastMeasureMs = 0;
uint32_t lastSendMs = 0;
uint32_t lastInitAttemptMs = 0;
uint32_t lastStatusMs = 0;

uint32_t lastDiagAttemptMs = 0;
uint32_t lastDiagAcceptedMs = 0;

uint32_t sensorTxAccepted = 0;
uint32_t sensorTxErrors = 0;
uint32_t diagTxAccepted = 0;
uint32_t diagTxErrors = 0;

// -------------------- ISR für Echo (D5) --------------------

ISR(PCINT2_vect) {
  const bool echoHigh = (PIND & _BV(PD5)) != 0;

  if (echoState == ECHO_WAIT_RISE && echoHigh) {
    echoRiseUs = micros();
    echoState = ECHO_WAIT_FALL;
  } else if (echoState == ECHO_WAIT_FALL && !echoHigh) {
    echoWidthUs = static_cast<uint32_t>(micros() - echoRiseUs);
    echoState = ECHO_COMPLETE;
  }
}

// -------------------- Hilfsfunktionen --------------------

bool elapsed(uint32_t now, uint32_t previous, uint32_t interval) {
  return static_cast<uint32_t>(now - previous) >= interval;
}

// -------------------- WS2812 LED-Ring Steuerung (v4.2) --------------------

void updateLedRing(uint16_t dist) {
  strip.clear();

  if (dist == INVALID_DISTANCE || dist > 50) {
    // > 50 cm oder ungültig: Alle LEDs aus
    strip.show();
    return;
  }

  uint8_t activeLeds = 0;
  uint32_t color = 0;

  if (dist <= 10) {
    activeLeds = 16;
    color = strip.Color(255, 0, 0);       // Rot (<= 10 cm)
  } else if (dist <= 20) {
    activeLeds = 12;
    color = strip.Color(255, 100, 0);     // Orange (<= 20 cm)
  } else if (dist <= 30) {
    activeLeds = 8;
    color = strip.Color(0, 255, 0);       // Grün (<= 30 cm)
  } else { // <= 40 cm
    activeLeds = 4;
    color = strip.Color(0, 255, 0);       // Grün (<= 40 cm)
  }

  // Iterativ im Uhrzeigersinn leuchten lassen
  for (uint8_t i = 0; i < activeLeds && i < NUM_LEDS; i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}

// -------------------- CAN initialisieren --------------------

bool initializeCan() {
  if (mcp2515.reset() != MCP2515::ERROR_OK) {
    Serial.println(F("[CAN] Reset fehlgeschlagen."));
    return false;
  }

  if (mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ) != MCP2515::ERROR_OK) {
    Serial.println(F("[CAN] Bitratenkonfiguration fehlgeschlagen."));
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

void enqueueDiagnostic(uint8_t eventType, uint8_t errorCode) {
  if (queueCount == DIAG_QUEUE_SIZE) {
    queueHead = static_cast<uint8_t>((queueHead + 1) % DIAG_QUEUE_SIZE);
    queueCount--;
    queueOverflows++;
  }

  const uint8_t tail = static_cast<uint8_t>((queueHead + queueCount) % DIAG_QUEUE_SIZE);
  diagQueue[tail].eventType = eventType;
  diagQueue[tail].errorCode = errorCode;
  eventCounter++;
  diagQueue[tail].counter = eventCounter;
  queueCount++;
}

// -------------------- Messergebnis auswerten --------------------

void applyMeasurement(bool valid, uint16_t measuredCm, uint8_t errorCode) {
  distanceCm = valid ? measuredCm : INVALID_DISTANCE;
  updateLedRing(distanceCm); // Lokale Aktorik sofort aktualisieren

  if (valid) {
    invalidCount = 0;
    if (validCount < VALID_LIMIT) validCount++;

    if (sensorErrorActive && validCount >= VALID_LIMIT) {
      sensorErrorActive = false;
      enqueueDiagnostic(EVENT_CLEARED, activeErrorCode);
    }
  } else {
    validCount = 0;
    if (invalidCount < INVALID_LIMIT) invalidCount++;

    if (!sensorErrorActive && invalidCount >= INVALID_LIMIT) {
      sensorErrorActive = true;
      activeErrorCode = errorCode;
      enqueueDiagnostic(EVENT_ACTIVE, activeErrorCode);
    }
  }
}

// -------------------- Messung starten --------------------

void startMeasurement() {
  measurementStartedUs = micros();
  measurementRunning = true;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    echoState = ECHO_IDLE;
    echoWidthUs = 0;
    if ((PIND & _BV(PD5)) == 0) {
      echoState = ECHO_WAIT_RISE;
    }
  }

  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
}

// -------------------- Messung bearbeiten --------------------

void serviceMeasurement(uint32_t nowMs) {
  if (measurementRunning) {
    bool completed = false;
    bool timedOut = false;
    uint32_t widthUs = 0;

    const bool timeoutDue = elapsed(micros(), measurementStartedUs, ECHO_TIMEOUT_US);

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      if (echoState == ECHO_COMPLETE) {
        widthUs = echoWidthUs;
        echoState = ECHO_IDLE;
        completed = true;
      } else if (timeoutDue) {
        echoState = ECHO_IDLE;
        timedOut = true;
      }
    }

    if (completed) {
      measurementRunning = false;
      lastPulseUs = widthUs;

      const bool inRange = widthUs >= static_cast<uint32_t>(MIN_DISTANCE_CM) * 58UL &&
                           widthUs <= static_cast<uint32_t>(MAX_DISTANCE_CM) * 58UL;

      if (inRange) {
        const uint16_t measuredCm = static_cast<uint16_t>((widthUs + 29UL) / 58UL);
        applyMeasurement(true, measuredCm, 0);
      } else {
        applyMeasurement(false, INVALID_DISTANCE, ERROR_RANGE);
      }
    } else if (timedOut) {
      measurementRunning = false;
      lastPulseUs = 0;
      applyMeasurement(false, INVALID_DISTANCE, ERROR_TIMEOUT);
    }
  }

  if (!measurementRunning && elapsed(nowMs, lastMeasureMs, MEASURE_INTERVAL_MS)) {
    lastMeasureMs = nowMs;
    startMeasurement();
  }
}

// -------------------- Sensornachricht senden --------------------

void sendSensorFrame(uint32_t nowMs) {
  if (!canReady || !elapsed(nowMs, lastSendMs, SEND_INTERVAL_MS)) {
    return;
  }
  lastSendMs = nowMs;

  struct can_frame frame = {};
  frame.can_id = CAN_ID_SENSOR;
  frame.can_dlc = 2;
  frame.data[0] = static_cast<uint8_t>(distanceCm & 0xFF);
  frame.data[1] = static_cast<uint8_t>((distanceCm >> 8) & 0xFF);

  if (mcp2515.sendMessage(&frame) == MCP2515::ERROR_OK) {
    sensorTxAccepted++;
  } else {
    sensorTxErrors++;
  }
}

// -------------------- Diagnose senden --------------------

void processDiagnostics(uint32_t nowMs) {
  if (!canReady) return;

  if (diagAttemptMade && !elapsed(nowMs, lastDiagAttemptMs, DIAG_MIN_GAP_MS)) {
    return;
  }

  const bool queued = queueCount > 0;
  const bool repeatDue = sensorErrorActive && elapsed(nowMs, lastDiagAcceptedMs, DIAG_REPEAT_MS);

  if (!queued && !repeatDue) return;

  DiagnosticEvent event;
  if (queued) {
    event = diagQueue[queueHead];
  } else {
    event.eventType = EVENT_ACTIVE;
    event.errorCode = activeErrorCode;
    event.counter = eventCounter;
  }

  struct can_frame frame = {};
  frame.can_id = CAN_ID_DIAG;
  frame.can_dlc = 4;
  frame.data[0] = SENSOR_ID;
  frame.data[1] = event.eventType;
  frame.data[2] = event.errorCode;
  frame.data[3] = event.counter;

  lastDiagAttemptMs = nowMs;
  diagAttemptMade = true;

  if (mcp2515.sendMessage(&frame) != MCP2515::ERROR_OK) {
    diagTxErrors++;
    return;
  }

  diagTxAccepted++;
  lastDiagAcceptedMs = nowMs;

  if (queued) {
    queueHead = static_cast<uint8_t>((queueHead + 1) % DIAG_QUEUE_SIZE);
    queueCount--;
  }
}

// -------------------- Status anzeigen (inkl. CAN-Info & Ultraschall) --------------------

void printStatus() {
  Serial.print(F("[NODE2] Distanz = "));
  if (distanceCm == INVALID_DISTANCE) {
    Serial.print(F("-- (kein Wert)"));
  } else {
    Serial.print(distanceCm);
    Serial.print(F(" cm"));
  }

  Serial.print(F(" | Puls = "));
  Serial.print(lastPulseUs);
  Serial.print(F(" us | Sensor-Fehler = "));
  Serial.println(sensorErrorActive ? F("AKTIV") : F("NEIN"));

  Serial.print(F("[TX] Sensor OK/Err = "));
  Serial.print(sensorTxAccepted);
  Serial.print(F("/"));
  Serial.print(sensorTxErrors);
  Serial.print(F(" | Diag OK/Err = "));
  Serial.print(diagTxAccepted);
  Serial.print(F("/"));
  Serial.println(diagTxErrors);

  if (canReady) {
    Serial.print(F("[CAN INFO] EFLG = 0x"));
    Serial.print(mcp2515.getErrorFlags(), HEX);
    Serial.print(F(" | TEC = "));
    Serial.print(mcp2515.errorCountTX());
    Serial.print(F(" | REC = "));
    Serial.println(mcp2515.errorCountRX());
  } else {
    Serial.println(F("[CAN INFO] Nicht initialisiert."));
  }
  Serial.println(F("--------------------------------------------------"));
}

// -------------------- Setup --------------------

void setup() {
  Serial.begin(115200);

  pinMode(PIN_TRIG, OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO, INPUT);

  // WS2812 LED-Ring initialisieren & Helligkeit auf sichere 35% drosseln
  strip.begin();
  strip.setBrightness(90); 
  strip.show();

  // Echo an D5 (PCINT21)
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    PCMSK2 |= _BV(PCINT21);
    PCIFR = _BV(PCIF2);
    PCICR |= _BV(PCIE2);
  }

  SPI.begin();
  canReady = initializeCan();

  const uint32_t now = millis();
  lastMeasureMs = now;
  lastSendMs = now;
  lastInitAttemptMs = now;
  lastStatusMs = now;

  Serial.println(F("[NODE2] HC-SR04 + WS2812 LED-Ring v4.2 gestartet"));
  Serial.println(F("[SERIAL] s = Status (Ultraschall & CAN-Info ausgeben)"));

  startMeasurement();
}

// -------------------- Hauptschleife --------------------

void loop() {
  const uint32_t now = millis();

  serviceMeasurement(now);

  if (!measurementRunning && !canReady && elapsed(now, lastInitAttemptMs, INIT_RETRY_MS)) {
    lastInitAttemptMs = now;
    canReady = initializeCan();
  }

  // Empfangspuffer leeren
  struct can_frame frame;
  if (canReady) {
    for (uint8_t i = 0; i < 2; i++) {
      if (mcp2515.readMessage(&frame) != MCP2515::ERROR_OK) break;
    }
  }

  sendSensorFrame(now);

  if (!measurementRunning) {
    processDiagnostics(now);

    if (Serial.available() > 0) {
      const char cmd = static_cast<char>(Serial.read());
      if (cmd == 's' || cmd == 'S') {
        printStatus();
      }
    }

    if (elapsed(now, lastStatusMs, STATUS_INTERVAL_MS)) {
      lastStatusMs = now;
      printStatus();
    }
  }
}