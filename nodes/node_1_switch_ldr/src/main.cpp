#include <SPI.h>
#include <mcp2515.h>

// Pin-Definitionen für Node 1
const int PIN_SWITCH = 3;  // Schalter an Pin D3
const int PIN_LDR    = A0; // LDR an Analog-Pin A0
const int PIN_CS     = 10; // SPI Chip Select für das HW-184 MCP2515 Modul

// MCP2515 Instanz initialisieren (CS auf Pin 10)
MCP2515 mcp2515(PIN_CS);

// CAN-Frame-Struktur deklarieren
struct can_frame canMsg;

unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 500; // Sende alle 500 ms

void setup() {
  Serial.begin(115200);

  // Schalter-Pin mit internem Pullup-Widerstand konfigurieren
  pinMode(PIN_SWITCH, INPUT_PULLUP);

  // MCP2515-Bus-Initialisierung (500 kbit/s, 8 MHz Quarz auf dem HW-184 Modul)
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ); 
  mcp2515.setNormalMode();

  // CAN-ID und Nutzlastlänge (DLC) festlegen (ID 0x101, 3 Bytes Payload)
  canMsg.can_id  = 0x101; 
  canMsg.can_dlc = 3;     

  Serial.println("Node 1 (Schalter & LDR) erfolgreich initialisiert.");
}

void loop() {
  unsigned long currentMillis = millis();

  // Zyklischer Sende-Intervall (500 ms)
  if (currentMillis - lastSendTime >= SEND_INTERVAL) {
    lastSendTime = currentMillis;

    // 1. Schalterstatus auslesen (ACTIVE_LOW durch INPUT_PULLUP: gedrückt = 1)
    uint8_t switchStatus = (digitalRead(PIN_SWITCH) == LOW) ? 1 : 0;

    // 2. LDR-Helligkeitswert auslesen (Wertebereich 0 bis 1023)
    uint16_t ldrValue = analogRead(PIN_LDR);

    // 3. Payload-Bytes gemäß DBC-Spezifikation belegen
    // Byte 0: Schalter-Status
    canMsg.data[0] = switchStatus;
    
    // Byte 1 & 2: LDR-Wert aufteilen (16-Bit in Low-Byte und High-Byte)
    canMsg.data[1] = ldrValue & 0xFF;        // Low-Byte
    canMsg.data[2] = (ldrValue >> 8) & 0xFF; // High-Byte
    
    // Bytes 3 bis 7 bleiben ungenutzt / 0

    // 4. Nachricht über den CAN-Bus senden
    if (mcp2515.sendMessage(&canMsg) == MCP2515::ERROR_OK) {
      Serial.print("CAN-Frame 0x101 gesendet -> Switch: ");
      Serial.print(switchStatus);
      Serial.print(" | LDR: ");
      Serial.println(ldrValue);
    } else {
      Serial.println("Fehler beim Senden des CAN-Frames (0x101)!");
    }
  }
}