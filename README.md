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



## 🔌 Hardware-Belegung (Pinout)

Das System nutzt einen **Sub-D 25 Stecker** zur Verbindung mit dem Overhead Panel (OHP).

### Wichtige Hinweise zum Löten
* **GND (Pin 1):** Muss im Steckergehäuse auf alle Schalter verteilt werden (Common Ground).
* **Detect (Pin 2):** Muss im Stecker fest mit **Pin 1 (GND)** gebrückt werden.
* **LED Power (Pin 25):** Liefert +5V für die Schalter-Beleuchtung (Vorwiderstände beachten!).

### Sub-D 25 Belegungsplan (MOSAM v2.5)

| Sub-D Pin | Teensy Pin | Funktion | Logik / Typ |
| :---: | :---: | :--- | :--- |
| **1** | **GND** | **Masse (Common)** | **Verteiler für alle Schalter** |
| **2** | **1** | **Detect Logic** | **Brücke nach Pin 1 (GND)** |
| **3** | 14 | Beacon | Switch |
| **4** | 16 | Strobe | ON (Low) / AUTO (Open) |
| **5** | 17 | Nav Lights | Master Switch |
| **6** | 19 | Nose Light | TAXI (Low) / OFF (Open) |
| **7** | 20 | Nose Light | T.O. (Low) / OFF (Open) |
| **8** | 21 | Rwy Turnoff | Switch |
| **9** | 22 | Landing Lights | Master Switch (L+R combined) |
| **10** | 24 | Seatbelts | Switch |
| **11** | 25 | Dome Light | DIM (Low) / OFF (Open) |
| **12** | 27 | Wiper Capt | SLOW |
| **13** | 28 | Wiper Capt | FAST |
| **14** | 29 | Call Button | Pushbutton |
| **15** | 30 | Anti-Ice | Wing |
| **16** | 31 | Anti-Ice | Eng 1 + 2 (Combined) |
| **17** | 34 | APU Master | Switch |
| **18** | 35 | APU Start | Pushbutton |
| **19** | 36 | APU Bleed | Switch |
| **20** | 37 | X-Bleed | OPEN (Low) / AUTO (Open) |
| **21** | 38 | Elec Pump | Yellow |
| **22** | 39 | PTU | OFF (Low) / AUTO (Open) |
| **23** | 40 | Pack 1 | Switch |
| **24** | 32 | Pack 2 | Switch |
| **25** | **Vin** | **+5V Power** | **Nur für LEDs (Pluspol)** |