#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

// Pin-Definitionen für Node 2
const int PIN_TRIG = 4;   // Trig-Pin des Ultraschall-Sensors an D4
const int PIN_ECHO = 5;   // Echo-Pin des Ultraschall-Sensors an D5
const int PIN_CS   = 10;  // SPI Chip Select für das MCP2515 Modul

MCP2515 mcp2515(PIN_CS);
struct can_frame canMsg;

unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 500; // Sende-Intervall in Millisekunden

void setup() {
  Serial.begin(115200);

  // Ultraschall-Pins konfigurieren
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  // MCP2515-Bus-Initialisierung (500 kbit/s, 8 MHz Quarz)
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  // CAN-ID (0x102) und Payload-Länge (2 Bytes für den 16-Bit-Abstandswert) festlegen
  canMsg.can_id = 0x102;
  canMsg.can_dlc = 2;

  Serial.println("Node 2 (Ultraschall) erfolgreich initialisiert.");
}

void loop() {
  unsigned long currentMillis = millis();

  // Zyklisches Sende-Intervall (500 ms)
  if (currentMillis - lastSendTime >= SEND_INTERVAL) {
    lastSendTime = currentMillis;

    // 1. Ultraschall-Messung auslösen (10 Mikrosekunden Impuls)
    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);

    // 2. Laufzeit messen und in Zentimeter umrechnen (Schallgeschwindigkeit ca. 0.0343 cm/µs)
    long duration = pulseIn(PIN_ECHO, HIGH, 30000); // Timeout nach 30 ms
    uint16_t distance = (duration * 0.0343) / 2;

    // 3. Abstandswert aufteilen und in den CAN-Frame packen (High-Byte und Low-Byte)
    canMsg.data[0] = (distance >> 8) & 0xFF;
    canMsg.data[1] = distance & 0xFF;

    // 4. CAN-Nachricht senden
    MCP2515::ERROR status = mcp2515.sendMessage(&canMsg);
    if (status == MCP2515::ERROR_OK) {
      Serial.print("CAN-Frame 0x102 gesendet -> Abstand: ");
      Serial.print(distance);
      Serial.println(" cm");
    } else {
      Serial.println("Fehler beim Senden des CAN-Frames (0x102)!");
    }
  }
}