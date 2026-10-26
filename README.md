# 🛡️ CAN-Guard – Dezentrales CAN-Bus- und IoT-Gateway-Netzwerk

**CAN-Guard** ist ein verteiltes Sensor- und Aktor-Netzwerk auf CAN-Bus-Basis. Mehrere Arduino-Nano-Knoten erfassen Umgebungsdaten (Licht, Distanz, Temperatur, Luftfeuchtigkeit) und senden diese über einen CAN-Bus an ein zentrales ESP8266-Gateway. Das Gateway visualisiert die Daten lokal auf einem OLED-Display und stellt sie über ein WLAN-Web-Dashboard bereit.

---

## 🚧 Aktueller Projektstatus (Zwischenstand)

Das Projekt befindet sich aktuell zu **ca. 50 %** in der Fertigstellung. Der Fokus lag bisher auf dem Hardware-Aufbau und der grundlegenden Kommunikation.

**Bereits abgeschlossen:**
- ✅ **Hardware-Aufbau:** Alle 3 Sensor-Knoten und das Gateway sind physisch aufgebaut und verkabelt.
- ✅ **CAN-Bus-Kommunikation:** Der Bus läuft stabil mit **500 kbit/s**. Alle Knoten senden ihre Daten erfolgreich.
- ✅ **Gateway-Empfang:** Das ESP8266-Gateway empfängt alle Frames (0x101–0x105) fehlerfrei.
- ✅ **Lokale Visualisierung:** Ein 1,3" SH1106 OLED-Display zeigt live den Status aller Sensoren.
- ✅ **Web-Dashboard:** Ein asynchroner Webserver auf dem ESP8266 stellt die Live-Daten über ein responsives WLAN-Dashboard bereit.
- ✅ **Aktorik:** Node 3 steuert 4x WS2812B RGB-LEDs basierend auf dem LDR-Wert von Node 1 an (Automatik-Modus).

**In Arbeit / Nächste Schritte:**
- ⏳ DBC-Datei finalisieren und im Repository versionieren.
- ⏳ Automatisierte Tests (Native Unit-Tests für die Firmware-Logik).
- ⏳ CI/CD-Pipeline via GitHub Actions aufsetzen.
- ⏳ Dokumentation der Systemtests (Bus-Off-Recovery, 24h-Dauerlauf).

---

## 🏗️ Hardware-Architektur & Systemaufbau

Das System ist als klassische **Bus-Topologie** aufgebaut. Alle Knoten teilen sich die gemeinsame CAN-High/CAN-Low-Leitung und sind an beiden Enden mit 120-Ohm-Widerständen terminiert, um Reflexionen zu vermeiden.


