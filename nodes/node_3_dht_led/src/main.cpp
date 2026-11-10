#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <DHT.h>
#include <Adafruit_NeoPixel.h>

// --- PIN- & HARDWARE-DEFINITIONEN ---
const int PIN_CAN_CS = 10;
const int PIN_NEOPIXEL = 6;
const int NUM_PIXELS = 4;

// DHT11 Sensoren (3 DHT11-Sensoren)
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

// Non-Blocking Zeitsteuerung für die 3 DHT11-Sensoren (gestaffelt)
unsigned long lastRead1 = 0;
unsigned long lastRead2 = 334;
unsigned long lastRead3 = 665;
const unsigned long SENSOR_INTERVAL = 1000; // 1 Sekunde Intervall pro Sensor

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
  unsigned long currentMillis = millis();

  // --- 1. SCHRITT: CAN-NACHRICHTEN EMPFANGEN (Aktor-Logik für Node 1) ---
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    if (canMsg.can_id == 0x101) {
      static unsigned long lastRx0x101 = 0;
      unsigned long now = millis();
      
      // Doppelte Frames (aus RXB0/RXB1 Puffer) innerhalb von 800ms herausfiltern
      if (now - lastRx0x101 >= 800) {
        lastRx0x101 = now;

        uint16_t ldrValue = (canMsg.data[2] << 8) | canMsg.data[1];
        uint8_t switchStatus = canMsg.data[0];

        Serial.print("Empfangen von Node 1 -> Switch: ");
        Serial.print(switchStatus);
        Serial.print(" | LDR: ");
        Serial.println(ldrValue);

        pixels.clear();
        int ledsToLight = 0;
        if (ldrValue < 800) ledsToLight = 1;
        if (ldrValue < 650) ledsToLight = 2;
        if (ldrValue < 450) ledsToLight = 3;
        if (ldrValue < 300) ledsToLight = 4;

        for (int i = 0; i < ledsToLight; i++) {
          pixels.setPixelColor(i, pixels.Color(40, 40, 40));
        }
        pixels.show();
      }
    }
    
  }

  // --- 2. SCHRITT: NON-BLOCKING SENSORABFRAGE (Sensor-Role für Node 3) ---

  // Sensor 1 (ID 0x103)
  if (currentMillis - lastRead1 >= SENSOR_INTERVAL) {
    lastRead1 = currentMillis;
    float temp1 = dht1.readTemperature();
    float hum1 = dht1.readHumidity();

    if (!isnan(temp1) && !isnan(hum1)) {
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
    }
  }
      // Sensor 2 (ID 0x104)
  if (currentMillis - lastRead2 >= SENSOR_INTERVAL) {
    lastRead2 = currentMillis;
    float temp2 = dht2.readTemperature();
    float hum2 = dht2.readHumidity();

    if (!isnan(temp2) && !isnan(hum2)) {
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
    }
  }

  // Sensor 3 (ID 0x105)
  if (currentMillis - lastRead3 >= SENSOR_INTERVAL) {
    lastRead3 = currentMillis;
    float temp3 = dht3.readTemperature();
    float hum3 = dht3.readHumidity();

    if (!isnan(temp3) && !isnan(hum3)) {
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
    }
  }
}