# MOSAM - Motion Synced Aircraft Model

**MOSAM** ist ein Teensy-basierter Controller für ein A330 Scale-Modell, der via USB direkt mit **X-Plane** synchronisiert wird. Das System steuert physikalische Bewegungen (Pitch/Roll Wippe), ein komplettes Beleuchtungssystem und Triebwerke synchron zur Simulation.

## Features
* **Motion Sync:** 3-Servo-System simuliert Pitch & Roll (Wippen-Mechanik) sowie das Einfedern/Abheben bei Landung/Start.
* **N1-Synchronisation:** Die Modell-Triebwerke (PWM) folgen exakt der N1-Drehzahl der Airbus-Simulation.
* **Intelligentes Lichtsystem:**
    * Nav & Beacon
    * Strobes (Airbus Doppelblitz-Muster)
    * Landing Lights, Taxi & Runway Turnoff (synchron zu Cockpit-Schaltern)
    * Nose Light Logik (Gedimmt bei Taxi, Hell bei T/O, Aus bei Gear Up)
* **Demo Modus:** Autarke Präsentationssequenz (Start, Flug, Landung, Shutdown), aktivierbar per Button oder bei Inaktivität.
* **Calibration Mode:** Manuelle Anpassung der Servo-Offsets über Serial Commands im laufenden Betrieb.
* **Day/Night Mode:** Globale Dimmung aller LEDs (30% / 100%) per Befehl.

## Hardware Setup

**Microcontroller:** Teensy 4.0 / 4.1 (FlightSim Library)

### Pinout

| Pin | System | Funktion |
| :--- | :--- | :--- |
| **2** | Nose Light | Taxi (Dim) / Takeoff (Bright) / Off |
| **3** | Servo Nose Gear | Fahrwerk Vorne |
| **4** | Servo Main Gear | Fahrwerk Hinten |
| **5** | Wing Strobes | 2x Blitz (Airbus Style) |
| **6** | Tail Strobe | 1x Blitz + Nav Dim |
| **7** | Wing Navs | Rot / Grün |
| **8** | Beacon | Rot Blinkend |
| **9** | Engine 1 | PWM (Links) |
| **10** | Engine 2 | PWM (Rechts) |
| **11** | Landing Lights | Main Landing & Runway Turnoff |
| **12** | Motion Left | Stütze Links |
| **13** | Motion Right | Stütze Rechts |
| **33** | PROG / DEMO | Button gegen GND (30s Timer für Demo) |
| **41** | Motion Front | Stütze Vorne |

## Serial Commands (Steuerung)

Über den Serial Monitor (9600 Baud) können folgende Befehle gesendet werden:

### System & Environment
* `engrun` / `engstop`: Triebwerke freigeben oder sperren (Sicherheit).
* `day`: LEDs auf 100% Helligkeit.
* `night`: LEDs auf 30% Helligkeit gedimmt.

### Kalibrierung (Offsets)
* `cal_left X` / `cal_right X`: Roll-Offset kalibrieren (X = Wert).
* `cal_up X` / `cal_down X`: Pitch-Offset kalibrieren.
* `cal_off`: Kalibrierungsmodus beenden, zurück zu X-Plane Sync.

## Funktionsweise Demo-Modus
Wird der Button an Pin 33 innerhalb der ersten 30 Sekunden nach Stromzufuhr gedrückt, startet **MOSAM** eine vorprogrammierte Flugsequenz (ca. 2 Minuten), die alle Systeme demonstriert, ohne dass X-Plane laufen muss.
