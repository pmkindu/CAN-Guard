#include <SPI.h>
#include <mcp2515.h>
#include <DHT.h>
#include <Adafruit_NeoPixel.h>

// --- PIN- & HARDWARE-DEFINITIONEN ---
const int PIN_CAN_CS = 10;
const int PIN_NEOPIXEL = 6;
const int NUM_PIXELS = 4;

// DHT11 Sensoren (3 Stück gemäß Roadmap)
const int DHT_PIN_1 = 3;
const int DHT_PIN_2 = 4;
const int DHT_PIN_3 = 5;

DHT dht1(DHT_PIN_1, DHT11);
DHT dht2(DHT_PIN_2, DHT11);
DHT dht3(DHT_PIN_3, DHT11);

// NeoPixel LED-Strip Initialisierung
Adafruit_NeoPixel pixels(NUM_PIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// MCP2515 CAN-Instanz
MCP2515 mcp2515(PIN_CAN_CS);
struct can_frame canMsg;

// Zeitsteuerung für das Senden der Umweltdaten
unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 3000; // Alle 2 Sekunden

void setup() {
  Serial.begin(115200);

  // Sensoren und LEDs starten
  dht1.begin();
  dht2.begin();
  dht3.begin();
  pixels.begin();
  pixels.show(); // Alle LEDs initial ausschalten

  // CAN-Bus initialisieren
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  Serial.println("Node 3 (DHT11 + LED-Aktor) erfolgreich initialisiert.");
}

void loop() {
  // --- 1. SCHRITT: CAN-NACHRICHTEN EMPFANGEN (Aktor-Logik für Node 1) ---
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    // Prüfen, ob die Nachricht von Node 1 (ID 0x101) stammt
    if (canMsg.can_id == 0x101) {
      // LDR-Wert exakt passend zu Node 1 aus Byte 1 (Low-Byte) und Byte 2 (High-Byte) zusammensetzen
      uint16_t ldrValue = (canMsg.data[2] << 8) | canMsg.data[1];
      uint8_t switchStatus = canMsg.data[0];

      Serial.print("Empfangen von Node 1 -> Switch: ");
      Serial.print(switchStatus);
      Serial.print(" | LDR: ");
      Serial.println(ldrValue);

      // --- STÜCK-FÜR-STÜCK LED-LOGIK (Je dunkler, desto mehr LEDs) ---
      // LDR-Wertebereich: 0 (stockdunkel) bis 1023 (sehr hell)
      pixels.clear();
      
      int ledsToLight = 0;
      if (ldrValue < 800) ledsToLight = 1;
      if (ldrValue < 650) ledsToLight = 2;
      if (ldrValue < 450) ledsToLight = 3;
      if (ldrValue < 300) ledsToLight = 4;
      
      // Die entsprechende Anzahl an LEDs einschalten 
      for (int i = 0; i < ledsToLight; i++) {
        pixels.setPixelColor(i, pixels.Color(40, 40, 40)); 
      }
           
      pixels.show();
      delay(10);
    }
  }

  // --- 2. SCHRITT: UMWELTDATEN ZYKLISCH SENDEN (Sensor-Rolle für Node 3) ---
  unsigned long currentMillis = millis();
  if (currentMillis - lastSendTime >= SEND_INTERVAL) {
    lastSendTime = currentMillis;

// --- Sensor 1 auslesen und senden (ID 0x103) ---
    float temp1 = dht1.readTemperature();
    float hum1 = dht1.readHumidity();
    
    Serial.print("Sensor 1 (0x103) -> Temp: ");
    Serial.print(temp1);
    Serial.print(" °C | Feuchte: ");
    Serial.print(hum1);
    Serial.println(" %");

    int16_t t1 = (int16_t)(temp1 * 10);
    int16_t h1 = (int16_t)(hum1 * 10);
    
    canMsg.can_id = 0x103;
    canMsg.can_dlc = 4;
    canMsg.data[0] = (t1 >> 8) & 0xFF;
    canMsg.data[1] = t1 & 0xFF;
    canMsg.data[2] = (h1 >> 8) & 0xFF;
    canMsg.data[3] = h1 & 0xFF;
    mcp2515.sendMessage(&canMsg);

    // --- Sensor 2 auslesen und senden (ID 0x104) ---
    float temp2 = dht2.readTemperature();
    float hum2 = dht2.readHumidity();
    
    Serial.print("Sensor 2 (0x104) -> Temp: ");
    Serial.print(temp2);
    Serial.print(" °C | Feuchte: ");
    Serial.print(hum2);
    Serial.println(" %");

    int16_t t2 = (int16_t)(temp2 * 10);
    int16_t h2 = (int16_t)(hum2 * 10);
    
    canMsg.can_id = 0x104;
    canMsg.can_dlc = 4;
    canMsg.data[0] = (t2 >> 8) & 0xFF;
    canMsg.data[1] = t2 & 0xFF;
    canMsg.data[2] = (h2 >> 8) & 0xFF;
    canMsg.data[3] = h2 & 0xFF;
    mcp2515.sendMessage(&canMsg);

    // --- Sensor 3 auslesen und senden (ID 0x105) ---
    float temp3 = dht3.readTemperature();
    float hum3 = dht3.readHumidity();
    
    Serial.print("Sensor 3 (0x105) -> Temp: ");
    Serial.print(temp3);
    Serial.print(" °C | Feuchte: ");
    Serial.print(hum3);
    Serial.println(" %");

    int16_t t3 = (int16_t)(temp3 * 10);
    int16_t h3 = (int16_t)(hum3 * 10);
    
    canMsg.can_id = 0x105;
    canMsg.can_dlc = 4;
    canMsg.data[0] = (t3 >> 8) & 0xFF;
    canMsg.data[1] = t3 & 0xFF;
    canMsg.data[2] = (h3 >> 8) & 0xFF;
    canMsg.data[3] = h3 & 0xFF;
    mcp2515.sendMessage(&canMsg);

    Serial.println("----------------------------------------");

  }
}