#include <SPI.h>
#include <mcp2515.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

// --- PIN- & HARDWARE-DEFINITIONEN ---
const int PIN_CAN_CS = 15; // D8 für SPI-CS am ESP8266
MCP2515 mcp2515(PIN_CAN_CS);
struct can_frame canMsg;

// OLED Display (SH1106 128x64, I2C Adresse 0x3C)
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Webserver auf Port 80
ESP8266WebServer server(80);

// Variablen zum Zwischenspeichern der Messwerte
uint8_t lastSwitch = 0;
uint16_t lastLdr = 0;
uint16_t lastDistance = 0;
float lastTemp1 = 0, lastHum1 = 0;
float lastTemp2 = 0, lastHum2 = 0;
float lastTemp3 = 0, lastHum3 = 0;

// Diagnose- & Zeitstempel-Variablen (ms seit Start)
unsigned long packetCounter = 0;
unsigned long decoderErrors = 0;
unsigned long activeTimeouts = 0;

unsigned long timeNode1 = 0;
unsigned long timeNode2 = 0;
unsigned long timeSensor1 = 0;
unsigned long timeSensor2 = 0;
unsigned long timeSensor3 = 0;

// Hilfsfunktion für deutsches Dezimalkomma
String formatFloatComma(float val) {
  String s = String(val, 1);
  s.replace('.', ',');
  return s;
}

// Hilfsfunktion zur Ermittlung des Empfangsstatus und Alters
String getAgeStatus(unsigned long lastRx) {
  if (lastRx == 0) return "Keine Daten";
  unsigned long diff = (millis() - lastRx) / 1000;
  if (diff > 10) return "Veraltet (" + String(diff) + "s)";
  return "Aktuell (" + String((millis() - lastRx) / 1000.0, 1) + "s)";
}

// --- HTML & CSS DASHBOARD ---
void handleRoot() {
  String html = "<html lang='de'><head><meta charset='UTF-8'>";
  html += "<title>CAN-Guard Control Center</title>";
  html += "<style>";
  html += "body { font-family: 'Segoe UI', Arial, sans-serif; background: #121212; color: #e0e0e0; text-align: center; margin: 0; padding: 20px; }";
  html += ".container { max-width: 850px; margin: 0 auto; background: #1e1e1e; padding: 20px; border-radius: 12px; box-shadow: 0 8px 24px rgba(0,0,0,0.5); border: 1px solid #333; }";
  html += ".header { display: flex; justify-content: space-between; align-items: center; border-bottom: 2px solid #333; padding-bottom: 15px; margin-bottom: 20px; }";
  html += ".header h1 { font-size: 20px; color: #00bcd4; margin: 0; text-align: left; }";
  html += ".header p { font-size: 13px; color: #b0b0b0; margin: 5px 0 0 0; text-align: left; }";
  html += ".status-badge { background: #1b5e20; color: #ffffff; padding: 6px 14px; border-radius: 6px; font-size: 13px; font-weight: bold; border: 1px solid #2e7d32; }";
  
  // Grid für Node 1 und Node 2
  html += ".grid-2 { display: grid; grid-template-columns: 1fr 1fr; gap: 15px; margin-bottom: 15px; }";
  html += ".card { background: #252526; padding: 15px; border-radius: 8px; border: 1px solid #383838; text-align: left; }";
  html += ".card h2 { font-size: 15px; color: #9cdcfe; margin-top: 0; border-bottom: 1px solid #333; padding-bottom: 8px; }";
  
  // Progress Bar für LDR
  html += ".progress-bg { background: #333; border-radius: 4px; height: 10px; width: 100%; margin: 8px 0; overflow: hidden; }";
  html += ".progress-fill { background: #00bcd4; height: 100%; width: 0%; transition: width 0.3s ease; }";
  
  // Node 3 Klima Grid (3 Sensoren mit neuer Hierarchie)
  html += ".klima-grid { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 10px; margin-top: 10px; }";
  html += ".sensor-box { background: #2d2d2d; padding: 12px 8px; border-radius: 6px; text-align: center; border: 1px solid #3d3d3d; }";
  html += ".sensor-title { font-size: 12px; color: #b0b0b0; margin-bottom: 4px; }";
  html += ".sensor-temp { font-size: 20px; font-weight: bold; color: #ffffff; }";
  html += ".sensor-hum { font-size: 13px; color: #00bcd4; margin-top: 2px; }";
  html += ".sensor-id { font-size: 10px; color: #aaaaaa; margin-top: 6px; }";
  
  // Beleuchtungs-Bereich in Node 3
  html += ".lighting-section { margin-top: 15px; padding-top: 12px; border-top: 1px solid #333; display: flex; justify-content: space-between; align-items: center; font-size: 13px; }";
  html += ".dots { font-size: 16px; letter-spacing: 3px; color: #00bcd4; }";
  
  // Meta / Subtexte & Diagnose Footer
  html += ".meta-info { font-size: 11px; color: #aaaaaa; margin-top: 8px; display: flex; justify-content: space-between; align-items: center; }";
  html += ".footer { margin-top: 20px; border-top: 1px solid #333; padding-top: 12px; font-size: 12px; color: #b0b0b0; display: flex; justify-content: space-between; }";
  html += ".val { font-weight: bold; color: #fff; }";
  html += "</style>";
  
  // JavaScript für flackerfreies Live-Update via AJAX
  html += "<script>";
  html += "setInterval(function() {";
  html += "  fetch('/json').then(res => res.json()).then(data => {";
  html += "    document.getElementById('sw').innerText = data.sw ? 'Betätigt (1)' : 'Offen (0)';";
  html += "    document.getElementById('ldr').innerText = data.ldr + ' ADC';";
  html += "    let ldrPercent = (data.ldr / 1023) * 100;";
  html += "    document.getElementById('ldr-bar').style.width = ldrPercent + '%';";
  html += "    document.getElementById('dist').innerText = data.dist + ' cm';";
  html += "    document.getElementById('t1').innerText = data.t1.toFixed(1).replace('.', ',') + ' \\u00B0C';";
  html += "    document.getElementById('h1').innerText = data.h1.toFixed(1).replace('.', ',') + ' %';";
  html += "    document.getElementById('t2').innerText = data.t2.toFixed(1).replace('.', ',') + ' \\u00B0C';";
  html += "    document.getElementById('h2').innerText = data.h2.toFixed(1).replace('.', ',') + ' %';";
  html += "    document.getElementById('t3').innerText = data.t3.toFixed(1).replace('.', ',') + ' \\u00B0C';";
  html += "    document.getElementById('h3').innerText = data.h3.toFixed(1).replace('.', ',') + ' %';";
  html += "    document.getElementById('status-node1').innerText = data.status1;";
  html += "    document.getElementById('status-node2').innerText = data.status2;";
  html += "    document.getElementById('status-s1').innerText = data.statuss1;";
  html += "    document.getElementById('status-s2').innerText = data.statuss2;";
  html += "    document.getElementById('status-s3').innerText = data.statuss3;";
  html += "    document.getElementById('pkts').innerText = data.pkts;";
  html += "    document.getElementById('timeouts').innerText = data.timeouts;";
  html += "    document.getElementById('errors').innerText = data.errors;";
  // Dynamische Beleuchtungs-Aktualisierung
  html += "    let leds = data.leds;";
  html += "    let dotsHtml = '';";
  html += "    for (let i = 1; i <= 4; i++) {";
  html += "      dotsHtml += (i <= leds ? '&● ' : '&○ ');";
  html += "    }";
  html += "    document.getElementById('lighting-dots').innerHTML = dotsHtml;";
  html += "    document.getElementById('lighting-stage').innerText = '(Stufe ' + leds + ' von 4)';";
  html += "  });";
  html += "}, 2000);";
  html += "</script>";
  
  html += "</head><body>";
  html += "<div class='container'>";
  
  // Header
  html += "<div class='header'>";
  html += "<div><h1>🛡️ CAN-Guard</h1><p>Sensorik &bull; Beleuchtung &bull; CAN-Diagnose</p></div>";
  html += "<div class='status-badge'>Gateway: erreichbar</div>";
  html += "</div>";
  
  // Row 1: Node 1 & Node 2
  html += "<div class='grid-2'>";
  
  // Node 1 Card
  html += "<div class='card'>";
  html += "<h2>NODE 1 &mdash; Licht &amp; Schalter</h2>";
  html += "<div>Helligkeit: <span class='val' id='ldr'>" + String(lastLdr) + " ADC</span></div>";
  html += "<div class='progress-bg'><div class='progress-fill' id='ldr-bar' style='width: " + String((lastLdr / 1023.0) * 100) + "%;'></div></div>";
  html += "<div>Schalter: <span class='val' id='sw'>" + String(lastSwitch ? "Betätigt (1)" : "Offen (0)") + "</span></div>";
  html += "<div class='meta-info'><span>CAN-ID: 0x101</span><span id='status-node1'>" + getAgeStatus(timeNode1) + "</span></div>";
  html += "</div>";
  
  // Node 2 Card
  html += "<div class='card'>";
  html += "<h2>NODE 2 &mdash; Abstand</h2>";
  html += "<div style='font-size: 26px; font-weight: bold; margin: 18px 0; text-align: center;'><span class='val' id='dist'>" + String(lastDistance) + " cm</span></div>";
  html += "<div class='meta-info' style='margin-top: 32px;'><span>CAN-ID: 0x102</span><span id='status-node2'>" + getAgeStatus(timeNode2) + "</span></div>";
  html += "</div>";
  
  html += "</div>";
  
  // Row 2: Node 3 (Klima & Beleuchtung)
  html += "<div class='card' style='margin-bottom: 0;'>";
  html += "<h2>NODE 3 &mdash; Klima &amp; Beleuchtung</h2>";
  html += "<div class='klima-grid'>";
  
  // Sensor 1 (ID 0x103)
  html += "<div class='sensor-box'>";
  html += "<div class='sensor-title'>Sensor 1</div>";
  html += "<div class='sensor-temp' id='t1'>" + formatFloatComma(lastTemp1) + " &deg;C</div>";
  html += "<div class='sensor-hum' id='h1'>" + formatFloatComma(lastHum1) + " %</div>";
  html += "<div class='sensor-id'>0x103 &bull; <span id='status-s1'>" + getAgeStatus(timeSensor1) + "</span></div>";
  html += "</div>";
  
  // Sensor 2 (ID 0x104)
  html += "<div class='sensor-box'>";
  html += "<div class='sensor-title'>Sensor 2</div>";
  html += "<div class='sensor-temp' id='t2'>" + formatFloatComma(lastTemp2) + " &deg;C</div>";
  html += "<div class='sensor-hum' id='h2'>" + formatFloatComma(lastHum2) + " %</div>";
  html += "<div class='sensor-id'>0x104 &bull; <span id='status-s2'>" + getAgeStatus(timeSensor2) + "</span></div>";
  html += "</div>";
  
  // Sensor 3 (ID 0x105)
  html += "<div class='sensor-box'>";
  html += "<div class='sensor-title'>Sensor 3</div>";
  html += "<div class='sensor-temp' id='t3'>" + formatFloatComma(lastTemp3) + " &deg;C</div>";
  html += "<div class='sensor-hum' id='h3'>" + formatFloatComma(lastHum3) + " %</div>";
  html += "<div class='sensor-id'>0x105 &bull; <span id='status-s3'>" + getAgeStatus(timeSensor3) + "</span></div>";
  html += "</div>";
  
  html += "</div>"; // Ende Klima-Grid
  
  // Beleuchtungs-Sektion (mit IDs für dynamische Aktualisierung)
  html += "<div class='lighting-section'>";
  html += "<div><b>Beleuchtung:</b> <span class='dots' id='lighting-dots'>&○ &○ &○ &○</span> <span style='color: #b0b0b0; margin-left: 8px;' id='lighting-stage'>(Stufe 0 von 4)</span></div>";
  html += "<div>Modus: <span class='val'>Automatik (LDR-gesteuert)</span></div>";
  html += "</div>";
  
  html += "</div>"; // Ende Node 3 Card
  
  // Diagnose Footer
  html += "<div class='footer'>";
  html += "<span>CAN-Rx gesamt: <span class='val' id='pkts'>" + String(packetCounter) + "</span></span>";
  html += "<span>Aktive Timeouts: <span class='val' id='timeouts'>" + String(activeTimeouts) + "</span></span>";
  html += "<span>Decoderfehler: <span class='val' id='errors'>" + String(decoderErrors) + "</span></span>";
  html += "</div>";
  
  html += "</div></body></html>";
  
  server.send(200, "text/html", html);
}

// JSON-Endpunkt zur Versorgung der AJAX-Abfrage inklusive LDR-LED-Berechnung
void handleJson() {
  int ledsToLight = 0;
  if (lastLdr < 800) ledsToLight = 1;
  if (lastLdr < 650) ledsToLight = 2;
  if (lastLdr < 450) ledsToLight = 3;
  if (lastLdr < 300) ledsToLight = 4;

  String json = "{";
  json += "\"sw\":" + String(lastSwitch) + ",";
  json += "\"ldr\":" + String(lastLdr) + ",";
  json += "\"dist\":" + String(lastDistance) + ",";
  json += "\"t1\":" + String(lastTemp1, 2) + ",";
  json += "\"h1\":" + String(lastHum1, 2) + ",";
  json += "\"t2\":" + String(lastTemp2, 2) + ",";
  json += "\"h2\":" + String(lastHum2, 2) + ",";
  json += "\"t3\":" + String(lastTemp3, 2) + ",";
  json += "\"h3\":" + String(lastHum3, 2) + ",";
  json += "\"leds\":" + String(ledsToLight) + ",";
  json += "\"status1\":\"" + getAgeStatus(timeNode1) + "\",";
  json += "\"status2\":\"" + getAgeStatus(timeNode2) + "\",";
  json += "\"statuss1\":\"" + getAgeStatus(timeSensor1) + "\",";
  json += "\"statuss2\":\"" + getAgeStatus(timeSensor2) + "\",";
  json += "\"statuss3\":\"" + getAgeStatus(timeSensor3) + "\",";
  json += "\"pkts\":" + String(packetCounter) + ",";
  json += "\"timeouts\":" + String(activeTimeouts) + ",";
  json += "\"errors\":" + String(decoderErrors);
  json += "}";
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.begin(4, 5); 
  delay(1000);

  if(!display.begin(0x3C, true)) {
    Serial.println("SH1106 OLED Allocation failed!");
    for(;;);
  }
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.println("CAN-Guard Gateway");
  display.println("Starte AP & Web...");
  display.display();

  WiFi.softAP("CAN-Guard-Gateway", "12345678");
  server.on("/", handleRoot);
  server.on("/json", handleJson);
  server.begin();

  display.setCursor(0, 20);
  display.println("WLAN AP aktiv!");
  display.print("IP: "); display.println(WiFi.softAPIP());
  display.display();
  delay(2000);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  Serial.println("ESP8266 Gateway & Dashboard bereit.");
}

void loop() {
  server.handleClient();

  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    packetCounter++;
    unsigned long now = millis();
    
    switch (canMsg.can_id) {
      case 0x101: // Node 1: Switch & LDR
        lastSwitch = canMsg.data[0];
        lastLdr = (canMsg.data[2] << 8) | canMsg.data[1];
        timeNode1 = now;
        break;
        
      case 0x102: // Node 2: Ultraschall-Distanz
        lastDistance = (canMsg.data[0] << 8) | canMsg.data[1];
        timeNode2 = now;
        break;

      case 0x103: // Node 3: Sensor 1 (Temp/Hum)
        lastTemp1 = (float)(((int16_t)((canMsg.data[0] << 8) | canMsg.data[1])) / 10.0);
        lastHum1 = (float)(((int16_t)((canMsg.data[2] << 8) | canMsg.data[3])) / 10.0);
        timeSensor1 = now;
        break;

      case 0x104: // Node 3: Sensor 2 (Temp/Hum)
        lastTemp2 = (float)(((int16_t)((canMsg.data[0] << 8) | canMsg.data[1])) / 10.0);
        lastHum2 = (float)(((int16_t)((canMsg.data[2] << 8) | canMsg.data[3])) / 10.0);
        timeSensor2 = now;
        break;

      case 0x105: // Node 3: Sensor 3 (Temp/Hum)
        lastTemp3 = (float)(((int16_t)((canMsg.data[0] << 8) | canMsg.data[1])) / 10.0);
        lastHum3 = (float)(((int16_t)((canMsg.data[2] << 8) | canMsg.data[3])) / 10.0);
        timeSensor3 = now;
        break;
        
      default:
        decoderErrors++;
        break;
    }

    // OLED-Display aktualisieren
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("- CAN-Guard Status --------------");
    display.print("Switch: "); display.println(lastSwitch);
    display.print("LDR:    "); display.println(lastLdr);
    display.print("Dist:   "); display.print(lastDistance); display.println(" cm");
    display.print("T-1:     "); display.print(lastTemp1); display.println(" C");
    display.print("H-1:     "); display.print(lastHum1); display.println(" %");
    display.print("Pkts:   "); display.println(packetCounter);
    display.display();
  }
}