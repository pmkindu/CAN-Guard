# 🛡️ CAN-Guard – Dezentrales CAN-Bus- und IoT-Gateway-Netzwerk

**CAN-Guard** ist ein verteiltes Sensor- und Aktor-Netzwerk auf CAN-Bus-Basis (500 kbit/s, 11-Bit-IDs). Vier Arduino-Nano-Knoten erfassen Licht, Distanz, Temperatur, Luftfeuchtigkeit und Bewegung und reagieren lokal mit LED-Anzeigen. Ein ESP32-S3-Gateway (Node 5) wertet alle Daten aus, zeigt sie auf einem 3,2"-Touch-Display an, signalisiert den Systemzustand über Status-LEDs und stellt sie über ein WLAN-Web-Dashboard bereit. Grundlage ist die Master-Roadmap v4.2.

---

## 🚧 Aktueller Projektstatus

Nach der Erweiterung der Roadmap auf **v4.2** werden die Phasen von vorne durchlaufen.
**Stand:** Phase 1 abgeschlossen, Phase 2 (Spezifikation und DBC) in Arbeit.

| Phase | Inhalt | Status |
|---|---|---|
| 1 | Bus-Topologie, Terminierung, elektrische Prüfung | ✅ abgeschlossen |
| 2 | Nachrichtenkatalog, DBC-Datei, DHT11-Timing | ⏳ in Arbeit |
| 3 | Firmware der Sensor-Knoten (Node 1–4) | ⏳ teilweise (siehe unten) |
| 4 | Gateway Node 5 auf ESP32-S3 mit TFT, Encoder, Web | ⏳ offen |
| 5 | Integrationstests, PC-Monitoring, CI | ⏳ CI-Grundgerüst steht |
| 6 | Systemtests, Fehlerinjektion, 24-h-Dauerlauf | ⏳ offen |
| 7 | Dokumentation und Release v1.0.0 | ⏳ offen |

**Bereits umgesetzt:**
- ✅ **Bus-Aufbau:** 5 Knoten plus PC-Adapter (CANable 2.0) an einem gemeinsamen Bus, 120 Ω an beiden Enden.
- ✅ **Firmware Node 1 und Node 2:** zyklisches Senden (0x101, 0x102) mit Diagnosenachrichten.
- ✅ **Firmware Node 3:** auf Roadmap v4.2 aktualisiert.
- ✅ **CI-Pipeline:** GitHub Actions prüft bei jedem Push die DBC-Datei und das Kodieren und Dekodieren aller Frames (pytest, cantools, python-can). Die Tests beschreiben den Sollzustand nach v4.2 und schlagen fehl, bis die DBC angepasst ist.

**Nächste Schritte:**
- ⏳ DBC auf v4.2 erweitern: Status-Byte für 0x103–0x105, neuer Frame 0x106 (Node 4), Diagnosenachrichten 0x1F1–0x1F4. Ziel: CI grün.
- ⏳ Node 2 und Node 3: WS2812-LED-Ring (16 LEDs) für Abstands- bzw. Helligkeitsanzeige.
- ⏳ Node 4 (Bewegungsmelder SR602 mit Status-LEDs) ins Repository aufnehmen und testen.
- ⏳ Gateway von ESP8266 auf ESP32-S3 umstellen: TFT mit Touch, Drehencoder EC11, rote und grüne Status-LED.
- ⏳ Native Unit-Tests der Firmware-Logik (Unity) in die CI-Pipeline aufnehmen.

---


