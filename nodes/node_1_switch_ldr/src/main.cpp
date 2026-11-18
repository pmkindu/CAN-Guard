/*
 * CAN-Guard - Node 1 (Arduino Nano V3): Mikroschalter + LDR
 * Roadmap v4.1
 *
 *  0x101  DLC 3  B0 = Schalter (0/1), B1..B2 = LDR uint16 (0..1023, Low-Byte zuerst), alle 200 ms
 *  0x1F1  DLC 4  Diagnosenachricht:
 *                B0 = Sensor-ID (1 = Schalter [reserviert], 2 = LDR)
 *                B1 = Ereignis  (0x01 Fehler aktiv, 0x00 Fehler behoben)
 *                B2 = Fehlercode (0x01 Wert ungueltig, 0x02 Timeout, 0x03 ausserhalb Wertebereich)
 *                B3 = Ereigniszaehler (zaehlt jede gesendete Diagnosenachricht, laeuft ueber)
 *
 * Bibliothek: autowp/arduino-mcp2515 (mcp2515.h)
 * Pins: D3 Schalter (Pullup), A0 LDR, D10 CS, D11/D12/D13 SPI. D13 = SCK, daher nicht als LED nutzen.
 */
#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

// ------------------------------------------------------------------ Debug
// 1 = Statusmeldungen ueber Serial (bei Ereignissen und jede Sekunde), 0 = aus
#define DEBUG_SERIAL 1
#if DEBUG_SERIAL
  #define DBG(...)   Serial.print(__VA_ARGS__)
  #define DBGLN(...) Serial.println(__VA_ARGS__)
#else
  #define DBG(...)
  #define DBGLN(...)
#endif

// ------------------------------------------------------------------ Konfiguration
constexpr uint8_t  PIN_SWITCH = 3;
constexpr uint8_t  PIN_LDR    = A0;
constexpr uint8_t  PIN_CS     = 10;

constexpr uint32_t ID_DATA = 0x101;
constexpr uint32_t ID_DIAG = 0x1F1;

constexpr uint16_t SEND_INTERVAL_MS   = 200;   // Zyklus 0x101
constexpr uint16_t DIAG_REPEAT_MS     = 5000;  // aktive Fehler alle 5 s wiederholen
constexpr uint16_t DIAG_MIN_GAP_MS    = 200;   // Mindestabstand zwischen zwei Diagnosenachrichten
constexpr uint16_t DIAG_RETRY_MS      = 50;    // erneuter Sendeversuch, wenn der MCP2515 ausgelastet ist
constexpr uint16_t CAN_INIT_RETRY_MS  = 2000;  // erneute Initialisierung, falls das Modul nicht antwortet
constexpr uint16_t STATUS_PRINT_MS    = 1000;   // Debug-Statuszeile

constexpr uint8_t  DEBOUNCE_COUNT     = 3;     // 3 ungueltige Messungen -> aktiv, 3 gueltige -> behoben
constexpr uint8_t  SWITCH_DEBOUNCE_MS = 20;

// LDR-Plausibilitaet: analogRead liefert immer 0..1023, ein Wert ausserhalb ist nicht moeglich.
// Ungueltig heisst hier: Messwert klebt an einer Schiene (offene Leitung oder Kurzschluss).
// Ein echter LDR 5528 am 10-kOhm-Teiler erreicht die Schienen praktisch nie.
// Grenzen an der realen Schaltung pruefen (dunkel / hell abdecken) und ggf. anpassen.
constexpr uint16_t LDR_RAIL_LOW  = 2;
constexpr uint16_t LDR_RAIL_HIGH = 1021;

constexpr uint8_t  SENSOR_SWITCH = 1;
constexpr uint8_t  SENSOR_LDR    = 2;
constexpr uint8_t  EVT_CLEARED   = 0x00;
constexpr uint8_t  EVT_ACTIVE    = 0x01;
constexpr uint8_t  ERR_INVALID   = 0x01;       // Wert ungueltig (an Schiene)

// ------------------------------------------------------------------ Zustand
MCP2515 mcp2515(PIN_CS);
can_frame dataMsg;
can_frame diagMsg;

bool     canReady = false;
uint32_t lastCanInitTry = 0;
uint32_t nextSend = 0;
uint32_t lastStatusPrint = 0;
uint16_t txFailures = 0;
uint16_t lastLdr = 0;           // zuletzt gemessener LDR-Wert (fuer die Debug-Ausgabe)
uint8_t  lastErrorFlags = 0;

// Schalter
bool     swRaw = false;
bool     swStable = false;
bool     swLatched = false;     // kurzer Tastendruck zwischen zwei Frames bleibt erhalten
uint32_t swChangedAt = 0;

// LDR
bool     ldrErr = false;
uint8_t  ldrBad = 0;
uint8_t  ldrGood = 0;
uint32_t ldrNextRepeat = 0;

// Diagnose-Warteschlange (max. 4 Eintraege)
struct DiagEvent { uint8_t sensorId; uint8_t event; uint8_t code; };
constexpr uint8_t DIAG_QUEUE_SIZE = 4;
DiagEvent diagQueue[DIAG_QUEUE_SIZE];
uint8_t  qHead = 0;
uint8_t  qCount = 0;
uint8_t  diagEventCounter = 0;
uint32_t lastDiagTx = 0;
uint32_t lastDiagTry = 0;

// ------------------------------------------------------------------ CAN
bool initCan() {
  if (mcp2515.reset() != MCP2515::ERROR_OK) return false;
  if (mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ) != MCP2515::ERROR_OK) return false;
  if (mcp2515.setNormalMode() != MCP2515::ERROR_OK) return false;
  return true;
}

bool diagEnqueue(uint8_t sensorId, uint8_t event, uint8_t code) {
  if (qCount >= DIAG_QUEUE_SIZE) return false;
  uint8_t tail = (qHead + qCount) % DIAG_QUEUE_SIZE;
  diagQueue[tail] = {sensorId, event, code};
  qCount++;
  return true;
}

// Sendet hoechstens eine Diagnosenachricht, haelt den Mindestabstand ein, geht nie verloren
// (bleibt in der Warteschlange, bis der MCP2515 sie angenommen hat) und blockiert nicht.
void processDiagQueue(uint32_t now) {
  if (!canReady || qCount == 0) return;
  if ((uint32_t)(now - lastDiagTx) < DIAG_MIN_GAP_MS) return;
  if ((uint32_t)(now - lastDiagTry) < DIAG_RETRY_MS) return;
  lastDiagTry = now;

  const DiagEvent &e = diagQueue[qHead];
  diagMsg.can_id  = ID_DIAG;
  diagMsg.can_dlc = 4;
  diagMsg.data[0] = e.sensorId;
  diagMsg.data[1] = e.event;
  diagMsg.data[2] = e.code;
  diagMsg.data[3] = diagEventCounter;

  if (mcp2515.sendMessage(&diagMsg) == MCP2515::ERROR_OK) {
    diagEventCounter++;
    lastDiagTx = now;
    DBG(F("Diag 0x1F1 Sensor=")); DBG(e.sensorId);
    DBG(F(" Event=")); DBG(e.event);
    DBG(F(" Code=")); DBG(e.code);
    DBG(F(" Zaehler=")); DBGLN(diagMsg.data[3]);
    qHead = (qHead + 1) % DIAG_QUEUE_SIZE;
    qCount--;
  }
}

// ------------------------------------------------------------------ Eingaenge
// Wird in jedem loop()-Durchlauf aufgerufen (nicht nur alle 200 ms).
void sampleSwitch(uint32_t now) {
  bool raw = (digitalRead(PIN_SWITCH) == LOW);   // INPUT_PULLUP: gedrueckt = LOW
  if (raw != swRaw) {
    swRaw = raw;
    swChangedAt = now;
  } else if (raw != swStable && (uint32_t)(now - swChangedAt) >= SWITCH_DEBOUNCE_MS) {
    swStable = raw;
    if (swStable) swLatched = true;
  }
}

void evaluateLdr(uint16_t value, uint32_t now) {
  bool invalid = (value <= LDR_RAIL_LOW || value >= LDR_RAIL_HIGH);
  if (invalid) {
    ldrGood = 0;
    if (ldrBad < 255) ldrBad++;
  } else {
    ldrBad = 0;
    if (ldrGood < 255) ldrGood++;
  }

  if (!ldrErr && ldrBad >= DEBOUNCE_COUNT) {
    ldrErr = true;
    ldrNextRepeat = now + DIAG_REPEAT_MS;
    diagEnqueue(SENSOR_LDR, EVT_ACTIVE, ERR_INVALID);
  } else if (ldrErr && ldrGood >= DEBOUNCE_COUNT) {
    ldrErr = false;
    diagEnqueue(SENSOR_LDR, EVT_CLEARED, ERR_INVALID);
  }
}

// ------------------------------------------------------------------ Zyklische Aufgabe (200 ms)
void cyclicTask(uint32_t now) {
  uint16_t ldr = (uint16_t)analogRead(PIN_LDR);   // 10 Bit: 0..1023
  lastLdr = ldr;
  evaluateLdr(ldr, now);

  uint8_t sw = (swStable || swLatched) ? 1 : 0;
  swLatched = false;

  if (!canReady) return;

  // Der gemessene Wert wird immer unverfaelscht gesendet (kein Ersatzwert 0 bei Fehler).
  dataMsg.can_id  = ID_DATA;
  dataMsg.can_dlc = 3;
  dataMsg.data[0] = sw;
  dataMsg.data[1] = (uint8_t)(ldr & 0xFF);        // Low-Byte
  dataMsg.data[2] = (uint8_t)(ldr >> 8);          // High-Byte

  if (mcp2515.sendMessage(&dataMsg) != MCP2515::ERROR_OK) {
    txFailures++;                                  // z. B. alle Sendepuffer belegt (kein ACK am Bus)
  }
}

// ------------------------------------------------------------------ Arduino
void setup() {
#if DEBUG_SERIAL
  Serial.begin(115200);
#endif
  pinMode(PIN_SWITCH, INPUT_PULLUP);
  swRaw = swStable = (digitalRead(PIN_SWITCH) == LOW);

  uint32_t now = millis();
  swChangedAt = now;
  nextSend = now + SEND_INTERVAL_MS;
  lastDiagTx = now - DIAG_MIN_GAP_MS;              // erste Diagnose darf sofort raus
  lastDiagTry = now - DIAG_RETRY_MS;
  lastStatusPrint = now;
  lastCanInitTry = now;

  canReady = initCan();
  DBGLN(canReady ? F("Node 1: CAN bereit (500 kbit/s, 8 MHz)") : F("Node 1: CAN-Init FEHLER, Wiederholung alle 2 s"));
}

void loop() {
  const uint32_t now = millis();

  sampleSwitch(now);

  // MCP2515 nicht erreichbar: regelmaessig neu versuchen, ohne zu blockieren
  if (!canReady && (uint32_t)(now - lastCanInitTry) >= CAN_INIT_RETRY_MS) {
    lastCanInitTry = now;
    canReady = initCan();
    DBGLN(canReady ? F("Node 1: CAN-Init erfolgreich") : F("Node 1: CAN-Init weiterhin FEHLER"));
  }

  // Fester Takt ohne Drift: der naechste Zeitpunkt wird addiert, nicht neu gemessen
  if ((int32_t)(now - nextSend) >= 0) {
    nextSend += SEND_INTERVAL_MS;
    if ((int32_t)(now - nextSend) >= 0) nextSend = now + SEND_INTERVAL_MS;   // Schutz nach langer Pause
    cyclicTask(now);
  }

  // Aktiven Fehler alle 5 s wiederholen
  if (ldrErr && (int32_t)(now - ldrNextRepeat) >= 0) {
    if (diagEnqueue(SENSOR_LDR, EVT_ACTIVE, ERR_INVALID)) ldrNextRepeat = now + DIAG_REPEAT_MS;
  }

  processDiagQueue(now);

#if DEBUG_SERIAL
  if ((uint32_t)(now - lastStatusPrint) >= STATUS_PRINT_MS) {
    lastStatusPrint = now;
    DBG(F("Status: sw=")); DBG(swStable);
    DBG(F(" ldr=")); DBG(lastLdr);
    DBG(F(" ldrErr=")); DBG(ldrErr);
    DBG(F(" txFehler=")); DBG(txFailures);
    if (canReady) {
      uint8_t ef = mcp2515.getErrorFlags();
      DBG(F(" EFLG=0x")); DBG(ef, HEX);
      if (ef & MCP2515::EFLG_RX0OVR) DBG(F(" RX0OVR"));
      if (ef & MCP2515::EFLG_TXBO) DBG(F(" BUS-OFF"));
      else if (ef & MCP2515::EFLG_TXEP) DBG(F(" Error-Passive"));
      lastErrorFlags = ef;
    }
    DBGLN();
  }
#endif
}