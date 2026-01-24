#include <FlightSim.h>

// -------------------------------------------------------------------------
// MOSAM v2.6 - OHP Logic + Debug Mode
// Hardware: Teensy 4.1 + Sub-D 25 Breakout
// Logic: INPUT_PULLUP (LOW = Aktiv/GND, HIGH = Offen/5V blocked by Diode)
// -------------------------------------------------------------------------

// --- PIN DEFINITIONEN ---
// Hier deine Belegung prüfen. Beispielhaft für Dome & Spare:
const int pin_DomeLight = 25; // Sub-D Pin 11 (mit Schottky/Dimm-Logik)
const int pin_Spare     = 18; // Sub-D Pin 1 (Reserve)

// Weitere Pins hier definieren oder im Array unten nutzen...

// --- DATAREFS (Beispiele für Airbus/Toliss/Jardesign etc.) ---
FlightSimInteger domeLightSim; 

// --- DEBUG VARS ---
bool debugMode = false;
unsigned long lastDebugTime = 0;
const int DEBUG_INTERVAL = 1000; // 1 Sekunde

void setup() {
  // 1. Serielle Kommunikation für Debug
  Serial.begin(9600);
  while (!Serial && millis() < 2000) {} // Kurz warten

  // 2. Pins Konfigurieren (WICHTIG: INPUT_PULLUP für Diode-Logik)
  pinMode(pin_DomeLight, INPUT_PULLUP);
  pinMode(pin_Spare, INPUT_PULLUP);

  // LOOP für alle anderen Pins (0 bis 26), damit sie im Debug nicht "floaten"
  // Achtung: Pin 13 ist die LED, lassen wir aus.
  for (int i = 0; i <= 26; i++) {
    if (i != 13) pinMode(i, INPUT_PULLUP);
  }

  // 3. Datarefs binden
  // (Passe den String an dein Flugzeug an, z.B. "sim/cockpit2/switches/dome_light_on")
  domeLightSim = XPlaneRef("sim/cockpit2/switches/dome_light_on");
  
  // LED Test am Teensy
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH); delay(500); digitalWrite(13, LOW);
  Serial.println("SYSTEM READY. Type 'debug' to start monitoring.");
}

// --- DEBUG MANAGER ---
void handleSerialDebug() {
  // A) Befehle lesen
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim(); // Leerzeichen entfernen

    if (command.equalsIgnoreCase("debug")) {
      debugMode = true;
      Serial.println(">>> DEBUG MODE STARTED <<<");
    } 
    else if (command.equalsIgnoreCase("debugstop")) {
      debugMode = false;
      Serial.println(">>> DEBUG MODE STOPPED <<<");
    }
  }

  // B) Ausgabe generieren (nur alle 1000ms)
  if (debugMode && (millis() - lastDebugTime >= DEBUG_INTERVAL)) {
    lastDebugTime = millis();
    
    Serial.print("STATUS > ");
    
    // Wir scannen Pin 0 bis 26 (die typischen Sub-D Pins am Teensy)
    for (int pin = 0; pin <= 26; pin++) {
      if (pin == 13) continue; // Skip Onboard LED
      
      int state = digitalRead(pin);
      
      // Formatierung: P[Nummer]:[HI/LO]
      Serial.print("P");
      Serial.print(pin);
      Serial.print(":");
      // Umgekehrte Logik anzeigen? Nein, wir zeigen den physischen Pegel.
      // LO = GND (Schalter an), HI = 3.3V (Schalter aus)
      if (state == LOW) Serial.print("LO"); else Serial.print("HI");
      Serial.print(" ");
    }
    Serial.println(); // Zeilenumbruch
  }
}

// --- OHP LOGIK ---
void updateOHP() {
  // DOME LIGHT LOGIK
  // Hardware: Pin LOW = Schalter auf DIM oder HELL (GND verbunden)
  // Hardware: Pin HIGH = Schalter AUS (Gesperrt durch Diode oder offen)
  if (digitalRead(pin_DomeLight) == LOW) {
    domeLightSim = 1; // Sim Licht AN
  } else {
    domeLightSim = 0; // Sim Licht AUS
  }

  // Hier weitere Schalter einfügen...
}

// --- MAIN LOOP ---
void loop() {
  // 1. Simulator Kommunikation
  FlightSim.update();

  // 2. Debugging prüfen
  handleSerialDebug();

  // 3. Cockpit Logik ausführen
  updateOHP();
}