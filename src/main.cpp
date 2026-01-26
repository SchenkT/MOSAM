#include <Arduino.h>
#include <Servo.h>

// =========================================================
//      MOSAM v4.2 - SAFETY ACTIVE & STANDARD REFS
// =========================================================

// --- SERVO OBJEKTE ---
Servo noseGear; Servo mainGear;
Servo supLeft; Servo supRight; Servo supFront; 

// --- PINS (HARDWARE OUTPUTS - MODELL) ---
const int pinNoseLight = 2; const int pinNoseServo = 3; const int pinMainServo = 4;  
const int pinWingStrobes = 5; const int pinTailCombined = 6; const int pinWingNavs = 7;  
const int pinBeacon = 8; const int pinEng1 = 9; const int pinEng2 = 10; 
const int pinLanding = 11; const int pinSupLeft = 12; const int pinSupRight = 13; 
const int pinSupFront = 41; const int pinProgButton = 33; 

// --- PINS (HARDWARE INPUTS - PANEL) ---
const int pinOHP_Detect = 1;   
const int sw_Beacon = 18;      
const int sw_Strobe_On = 16;   
const int sw_Nav_Master = 17;  
const int sw_Nose_Taxi = 19;   
const int sw_Nose_TO = 20;     
const int sw_RwyTurnoff = 21;  
const int sw_Landing_Master = 22; 
const int sw_Seatbelts = 24;   
const int sw_Dome_Dim = 25;    
const int sw_Wiper_Slow = 27;  
const int sw_Wiper_Fast = 28;  
const int sw_Call_Btn = 29;    
const int sw_Ice_Wing = 30;    
const int sw_Ice_Eng_Comb = 31; 
const int sw_APU_Master = 34;  
const int sw_APU_Start = 35;   
const int sw_APU_Bleed = 36;   
const int sw_XBleed_Open = 37; 
const int sw_ElecPump = 38;    
const int sw_PTU_Off = 39;     
const int sw_Pack1 = 40;       
const int sw_Pack2 = 32;       

// --- VARS ---
unsigned long lastOHPUpdate = 0; const int OHP_REFRESH_RATE = 150; 
bool debugMode = false; unsigned long lastDebugTime = 0;
bool ohpConnected = false; // Status der Brücke

// =========================================================
// 1. PANEL REFS (WRITE -> CONTROL SIM SYSTEMS)
// Wir nutzen jetzt Standard Refs, da ToLiss diese überwacht!
// =========================================================
FlightSimInteger pan_Beacon;
FlightSimInteger pan_Strobe; 
FlightSimInteger pan_Nav;         
FlightSimInteger pan_Nose; 
FlightSimInteger pan_LandingL;    
FlightSimInteger pan_LandingR;    
FlightSimInteger pan_LandingNose; // Standard Landing Switch array
FlightSimInteger pan_RwyTurn;
FlightSimInteger pan_Seatbelt;    
FlightSimFloat   pan_DomeVal;     // Wir dimmen direkt
FlightSimInteger pan_Wiper; 
FlightSimInteger pan_Call;        
FlightSimInteger pan_IceWing; 
FlightSimInteger pan_IceEng1; FlightSimInteger pan_IceEng2;
FlightSimInteger pan_APUMaster;
FlightSimInteger pan_APUStart;    
FlightSimInteger pan_APUBleed;
FlightSimInteger pan_XBleed; 
FlightSimInteger pan_ElecPump;
FlightSimInteger pan_PTU;
FlightSimInteger pan_Pack1; FlightSimInteger pan_Pack2;

// =========================================================
// 2. MODEL REFS (READ -> FEEDBACK FROM SIM)
// =========================================================
FlightSimInteger mod_NavLight; 
FlightSimInteger mod_BeaconLight; 
FlightSimInteger mod_StrobeLight;
FlightSimInteger mod_LandingLight; 
FlightSimInteger mod_TaxiLight;    
FlightSimFloat   mod_RwyTurnSwitch; 

// System Feedback Refs
FlightSimInteger xGearHandle; 
FlightSimFloat   xOnGround; 
FlightSimFloat   xBank; 
FlightSimFloat   xPitch;       
FlightSimFloat   xEng1N1; 
FlightSimFloat   xEng2N1;      

// --- INTERNALS ---
float noseCurrent = 5; int noseTarget = 5; float mainCurrent = 5; int mainTarget = 5;
float supLCurrent = 0; int supLTarget = 0; float supRCurrent = 0; int supRTarget = 0; float supFCurrent = 0; int supFTarget = 0;
unsigned long lastMoveTime = 0; int eng1TargetPWM = 0; int eng2TargetPWM = 0;
unsigned long eng1KickEnd = 0; unsigned long eng2KickEnd = 0; int run_eng = 0; float brightnessScale = 1.0; 
bool calMode = false; float calTargetBank = 0; float calTargetPitch = 0; int manualRollOffset = 0; int manualPitchOffset = 0;    
bool demoHasRun = false; bool demoModeActive = false; int demoNoseLightVal = 0; 

// --- CONSTANTS ---
const int REFRESH_RATE=10; const float SPEED_GEAR=0.05; const float SPEED_MOTION=0.2; 
const int ENG_IDLE_MIN=15; const int ENG_KICK_VAL=30; const int ENG_KICK_TIME=350;
const int VAL_OFF=0; const int VAL_TAXI=10; const int VAL_TO=40; const int STROBE_FLASH_VAL=50; 
const int NAV_WING_VAL=40; const int NAV_TAIL_DIM=10; const int BEACON_VAL=255; const int VAL_LANDING_MAX=100;
const int NOSE_POS_UP = 40; const int NOSE_POS_DOWN = 5;   
const int MAIN_POS_UP = 65; const int MAIN_POS_DOWN = 5;   
const int SUP_L_RETRACT = 110; const int SUP_L_EXTEND = 0;   
const int SUP_R_RETRACT = 0; const int SUP_R_EXTEND = 110;  
const int SUP_F_RETRACT = 0; const int SUP_F_EXTEND = 110;  
const int MOTION_NEUTRAL = 50; const int AIR_LIFT_OFFSET = 15; 

// Forward Decl
void updateHydraulics(); void updatePanelInputs(); void updateModelOutputs(); void waitAndAnimate(int waitTime); void runDemoSequence(); void printDebugTable();

void setup() {
  Serial.begin(9600);
  
  // --- A. MODEL REFS (READ) ---
  mod_NavLight      = XPlaneRef("sim/cockpit2/switches/navigation_lights_on");
  mod_BeaconLight   = XPlaneRef("sim/cockpit2/switches/beacon_on");
  mod_StrobeLight   = XPlaneRef("sim/cockpit2/switches/strobe_lights_on"); 
  mod_LandingLight  = XPlaneRef("sim/cockpit2/switches/landing_lights_on");
  mod_TaxiLight     = XPlaneRef("sim/cockpit2/switches/taxi_light_on"); 
  mod_RwyTurnSwitch = XPlaneRef("ckpt/oh/rwyTurnOff/anim"); // ToLiss Animation

  xGearHandle = XPlaneRef("sim/cockpit2/controls/gear_handle_down");
  xOnGround = XPlaneRef("sim/flightmodel/failures/onground_any"); 
  xBank = XPlaneRef("sim/flightmodel/position/phi");    
  xPitch = XPlaneRef("sim/flightmodel/position/true_theta"); 
  xEng1N1 = XPlaneRef("AirbusFBW/fmod/eng/N1Array[0]");
  xEng2N1 = XPlaneRef("AirbusFBW/fmod/eng/N1Array[1]");

  // --- B. PANEL REFS (WRITE - STANDARD SWITCHES) ---
  // Wir gehen zurück auf Standard X-Plane DataRefs zum SCHREIBEN.
  // ToLiss liest diese und schaltet dann seine internen Systeme.
  pan_Beacon    = XPlaneRef("sim/cockpit2/switches/beacon_on");      
  pan_Strobe    = XPlaneRef("sim/cockpit2/switches/strobe_lights_on");      
  pan_Nav       = XPlaneRef("sim/cockpit2/switches/navigation_lights_on");  
  pan_Nose      = XPlaneRef("sim/cockpit2/switches/taxi_light_on");        
  
  // Landing Lights Array (Standard X-Plane)
  pan_LandingL  = XPlaneRef("sim/cockpit2/switches/landing_lights_switch[0]");    
  pan_LandingR  = XPlaneRef("sim/cockpit2/switches/landing_lights_switch[1]");
  pan_LandingNose = XPlaneRef("sim/cockpit2/switches/landing_lights_switch[2]");
  
  pan_RwyTurn   = XPlaneRef("sim/cockpit2/switches/runway_turnoff_lights_on");     
  
  // Seatbelts (Standard)
  pan_Seatbelt  = XPlaneRef("sim/cockpit2/switches/fasten_seat_belts");    
  
  // Dome (Wir steuern den Dimmer direkt, das ist zuverlässiger)
  pan_DomeVal   = XPlaneRef("sim/cockpit2/electrical/instrument_brightness_ratio[0]"); 
  
  pan_Wiper     = XPlaneRef("sim/cockpit2/switches/wiper_speed"); 
  
  // Call (ToLiss spezifisch, da kein Standard)
  pan_Call      = XPlaneRef("AirbusFBW/OH/CallMech"); 
  
  // Ice & Systems (Standard)
  pan_IceWing   = XPlaneRef("sim/cockpit2/ice/ice_inlet_heat_on_per_engine[0]"); 
  pan_IceEng1   = XPlaneRef("sim/cockpit2/ice/ice_inlet_heat_on_per_engine[0]");
  pan_IceEng2   = XPlaneRef("sim/cockpit2/ice/ice_inlet_heat_on_per_engine[1]");
  pan_APUMaster = XPlaneRef("sim/cockpit2/electrical/APU_master_switch");
  pan_APUStart  = XPlaneRef("sim/cockpit2/electrical/APU_starter_switch");
  pan_APUBleed  = XPlaneRef("sim/cockpit2/bleedair/apu_bleed_on");
  pan_XBleed    = XPlaneRef("sim/cockpit2/bleedair/cross_tie_open"); 
  pan_ElecPump  = XPlaneRef("sim/cockpit2/hydraulics/actuators/electric_hydraulic_pump_on");
  pan_PTU       = XPlaneRef("sim/cockpit2/hydraulics/actuators/ptu_on");
  pan_Pack1     = XPlaneRef("sim/cockpit2/bleedair/bleed_air_on[0]");
  pan_Pack2     = XPlaneRef("sim/cockpit2/bleedair/bleed_air_on[1]");

  // PINS INIT
  pinMode(pinNoseLight, OUTPUT); pinMode(pinWingStrobes, OUTPUT); pinMode(pinTailCombined, OUTPUT); 
  pinMode(pinWingNavs, OUTPUT); pinMode(pinBeacon, OUTPUT); pinMode(pinEng1, OUTPUT); 
  pinMode(pinEng2, OUTPUT); pinMode(pinLanding, OUTPUT); pinMode(pinProgButton, INPUT_PULLUP); 
  
  for (int i=14; i<=40; i++) { pinMode(i, INPUT_PULLUP); }
  pinMode(sw_Pack2, INPUT_PULLUP); 
  pinMode(sw_Beacon, INPUT_PULLUP); 
  pinMode(pinOHP_Detect, INPUT_PULLUP);

  // INIT OUTPUTS
  analogWrite(pinNoseLight, 0); analogWrite(pinWingStrobes, 0); analogWrite(pinWingNavs, 0); 
  analogWrite(pinTailCombined, 0); analogWrite(pinBeacon, 0); analogWrite(pinEng1, 0); analogWrite(pinEng2, 0); 
  analogWrite(pinLanding, 0);

  // SERVO INIT
  noseGear.attach(pinNoseServo); mainGear.attach(pinMainServo); supLeft.attach(pinSupLeft); supRight.attach(pinSupRight); supFront.attach(pinSupFront);
  noseGear.write((int)noseCurrent); mainGear.write((int)mainCurrent); supLeft.write(SUP_L_RETRACT); supRight.write(SUP_R_RETRACT); supFront.write(SUP_F_RETRACT);

  Serial.println("--- MOSAM v4.2 SAFETY & STD REFS ---");
}

void loop() {
  FlightSim.update(); 
  
  if (digitalRead(pinProgButton) == LOW) { delay(50); if (digitalRead(pinProgButton) == LOW) { if (millis() < 30000 && !demoHasRun) runDemoSequence(); else { analogWrite(pinEng1, 0); analogWrite(pinEng2, 0); Serial.println("PROG"); delay(100); asm("bkpt #251"); }}}

  // 1. CHECK DETECT & UPDATE PANEL INPUTS
  unsigned long now = millis();
  
  // HIER IST DEINE GEWÜNSCHTE LOGIK:
  if (digitalRead(pinOHP_Detect) == LOW) {
      ohpConnected = true;
      if (!demoModeActive && !calMode) {
          if (now - lastOHPUpdate >= OHP_REFRESH_RATE) { lastOHPUpdate = now; updatePanelInputs(); }
      }
  } else {
      ohpConnected = false; // Panel ausgesteckt -> keine Updates senden
  }

  // 2. UPDATE MODEL OUTPUTS (Immer, solange Sim läuft)
  if (!demoModeActive) {
      updateModelOutputs();
  }

  // 3. DEBUG
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n'); cmd.trim();
    if (cmd.equalsIgnoreCase("debug")) debugMode = true; else if (cmd.equalsIgnoreCase("debugstop")) debugMode = false;
    else if (cmd.startsWith("cal_left")) { calMode = true; manualRollOffset = cmd.substring(9).toInt(); calTargetBank = -40; }
    else if (cmd == "cal_off") { calMode = false; manualRollOffset = 0; }
  }
  
  if (debugMode && (millis() - lastDebugTime > 1000)) {
      lastDebugTime = millis(); 
      printDebugTable();
  }
}

// =========================================================
// STEP 1: PANEL LOGIC
// =========================================================
void updatePanelInputs() {
  
  // BEACON (0=Off, 1=On)
  if (digitalRead(sw_Beacon) == LOW) pan_Beacon = 1; else pan_Beacon = 0;
  
  // STROBE (0=Off, 1=On)
  if (digitalRead(sw_Strobe_On) == LOW) pan_Strobe = 1; else pan_Strobe = 0;

  // NAV (0=Off, 1=On)
  if (digitalRead(sw_Nav_Master) == LOW) pan_Nav = 1; else pan_Nav = 0; 
  
  // LANDING (0=Off, 1=On)
  if (digitalRead(sw_Landing_Master) == LOW) { 
      pan_LandingL = 1; pan_LandingR = 1; pan_LandingNose = 1; 
  } else { 
      pan_LandingL = 0; pan_LandingR = 0; pan_LandingNose = 0; 
  }
  
  // SEATBELTS (0=Off, 1=On)
  if (digitalRead(sw_Seatbelts) == LOW) pan_Seatbelt = 1; else pan_Seatbelt = 0;
  
  // DOME (Wir setzen die Helligkeit direkt: 0.5 für DIM)
  if (digitalRead(sw_Dome_Dim) == LOW) pan_DomeVal = 0.5; else pan_DomeVal = 0.0;

  // SYSTEM SWITCHES
  if (digitalRead(sw_Call_Btn) == LOW) pan_Call = 1; else pan_Call = 0;
  if (digitalRead(sw_APU_Start) == LOW) pan_APUStart = 1; else pan_APUStart = 0;

  // NOSE LOGIC
  if (digitalRead(sw_Nose_Taxi) == LOW && digitalRead(sw_Nose_TO) == HIGH) { pan_Nose = 0; } // AUS
  else { pan_Nose = 1; } // AN (Taxi oder TO)

  // REST
  if (digitalRead(sw_RwyTurnoff) == LOW) pan_RwyTurn = 1; else pan_RwyTurn = 0;
  if (digitalRead(sw_Wiper_Fast) == LOW) pan_Wiper = 3; else if (digitalRead(sw_Wiper_Slow) == LOW) pan_Wiper = 2; else pan_Wiper = 0;
  if (digitalRead(sw_Ice_Wing) == LOW) pan_IceWing = 1; else pan_IceWing = 0;
  if (digitalRead(sw_Ice_Eng_Comb) == LOW) { pan_IceEng1 = 1; pan_IceEng2 = 1; } else { pan_IceEng1 = 0; pan_IceEng2 = 0; }
  if (digitalRead(sw_APU_Master) == LOW) pan_APUMaster = 1; else pan_APUMaster = 0;
  if (digitalRead(sw_APU_Bleed) == LOW) pan_APUBleed = 1; else pan_APUBleed = 0;
  if (digitalRead(sw_XBleed_Open) == LOW) pan_XBleed = 1; else pan_XBleed = 2; 
  if (digitalRead(sw_ElecPump) == LOW) pan_ElecPump = 1; else pan_ElecPump = 0;
  if (digitalRead(sw_PTU_Off) == LOW) pan_PTU = 0; else pan_PTU = 1; 
  if (digitalRead(sw_Pack1) == LOW) pan_Pack1 = 1; else pan_Pack1 = 0;
  if (digitalRead(sw_Pack2) == LOW) pan_Pack2 = 1; else pan_Pack2 = 0;
}

// =========================================================
// STEP 2: MODEL LOGIC
// =========================================================
void updateModelOutputs() {
  
  updateHydraulics();

  if ((FlightSim.isEnabled() || calMode) && !demoModeActive) { 
      // VARIABLES FROM SIM
      bool isStrobe = (mod_StrobeLight == 1); 
      bool isBeacon = (mod_BeaconLight == 1);
      bool isNav    = (mod_NavLight == 1);
      bool isLand   = (mod_LandingLight == 1);
      bool isTaxi   = (mod_TaxiLight == 1);

      if (xGearHandle == 1) { 
          if (isLand) analogWrite(pinNoseLight, (int)(VAL_TO * brightnessScale)); 
          else if (isTaxi) analogWrite(pinNoseLight, (int)(VAL_TAXI * brightnessScale)); 
          else analogWrite(pinNoseLight, 0); 
      } else analogWrite(pinNoseLight, 0); 
      
      int t1=0; if (xEng1N1 > 1) t1 = map((int)(float)xEng1N1, 15, 100, 0, 50); analogWrite(pinEng1, t1);
      int t2=0; if (xEng2N1 > 1) t2 = map((int)(float)xEng2N1, 15, 100, 0, 50); analogWrite(pinEng2, t2);

      if (isNav) analogWrite(pinWingNavs, (int)(40*brightnessScale)); else analogWrite(pinWingNavs, 0);
      if (isLand) analogWrite(pinLanding, (int)(100*brightnessScale)); 
      else if (mod_RwyTurnSwitch > 0.1) analogWrite(pinLanding, (int)(40*brightnessScale)); 
      else analogWrite(pinLanding, 0);

      unsigned long mt = millis() % 1000; 
      if (isBeacon && mt >= 500 && mt < 600) analogWrite(pinBeacon, (int)(255*brightnessScale)); else analogWrite(pinBeacon, 0);
      int strOut = 0;
      if (isStrobe) { if ((mt < 30) || (mt >= 100 && mt < 130)) strOut = (int)(STROBE_FLASH_VAL * brightnessScale); }
      analogWrite(pinWingStrobes, strOut);
      int tailOut = 0; if (isStrobe && mt < 100) tailOut = (int)(STROBE_FLASH_VAL * brightnessScale);
      if (tailOut > 0) analogWrite(pinTailCombined, tailOut); else if (isNav) analogWrite(pinTailCombined, (int)(NAV_TAIL_DIM * brightnessScale)); else analogWrite(pinTailCombined, 0);
  }
}

// =========================================================
// DEBUG TABLE
// =========================================================
void printDebugTable() {
    Serial.println(); 
    Serial.print("TIME: "); Serial.print(millis()/1000); Serial.print(" | PANEL: "); Serial.println(ohpConnected ? "CONN" : "DISC");
    auto stateStr = [](int pin) { return (digitalRead(pin) == LOW) ? "ON " : "OFF"; };

    Serial.println("--- DB25 PINS 1-8 ---");
    Serial.print("02/01/DETECT /"); Serial.println(stateStr(pinOHP_Detect));
    Serial.print("03/18/BEACON /"); Serial.print(stateStr(sw_Beacon)); Serial.print(" -> SIM: "); Serial.println((int)pan_Beacon);
    Serial.print("04/16/STROBE /"); Serial.print(stateStr(sw_Strobe_On)); Serial.print(" -> SIM: "); Serial.println((int)pan_Strobe);
    Serial.print("05/17/NAV    /"); Serial.print(stateStr(sw_Nav_Master)); Serial.print(" -> SIM: "); Serial.println((int)pan_Nav);
    Serial.print("06/19/NOSE_TX/"); Serial.println(stateStr(sw_Nose_Taxi));
    Serial.print("07/20/NOSE_TO/"); Serial.println(stateStr(sw_Nose_TO));
    Serial.print("08/21/RWY_TRN/"); Serial.println(stateStr(sw_RwyTurnoff));
    
    Serial.println("--- DB25 PINS 9-16 ---");
    Serial.print("09/22/LANDING/"); Serial.print(stateStr(sw_Landing_Master)); Serial.print(" -> SIM: "); Serial.println((int)pan_LandingL);
    Serial.print("10/24/SEATBLT/"); Serial.print(stateStr(sw_Seatbelts)); Serial.print(" -> SIM: "); Serial.println((int)pan_Seatbelt);
    Serial.print("11/25/DOME   /"); Serial.print(stateStr(sw_Dome_Dim)); Serial.print(" -> SIM: "); Serial.println((float)pan_DomeVal);
    Serial.print("14/29/CALL   /"); Serial.println(stateStr(sw_Call_Btn));
    Serial.println("---------------------");
}

void updateHydraulics() {
    unsigned long now = millis();
    if (now - lastMoveTime >= REFRESH_RATE) {
      lastMoveTime = now;
      if (abs(noseCurrent - noseTarget) > SPEED_GEAR) { if (noseCurrent < noseTarget) noseCurrent += SPEED_GEAR; else noseCurrent -= SPEED_GEAR; } else noseCurrent = noseTarget; noseGear.write((int)noseCurrent);
      if (abs(mainCurrent - mainTarget) > SPEED_GEAR) { if (mainCurrent < mainTarget) mainCurrent += SPEED_GEAR; else mainCurrent -= SPEED_GEAR; } else mainCurrent = mainTarget; mainGear.write((int)mainCurrent);
      if (abs(supLCurrent - supLTarget) > SPEED_MOTION) { if (supLCurrent < supLTarget) supLCurrent += SPEED_MOTION; else supLCurrent -= SPEED_MOTION; } else supLCurrent = supLTarget; supLeft.write(map((int)supLCurrent, 0, 100, SUP_L_RETRACT, SUP_L_EXTEND));
      if (abs(supRCurrent - supRTarget) > SPEED_MOTION) { if (supRCurrent < supRTarget) supRCurrent += SPEED_MOTION; else supRCurrent -= SPEED_MOTION; } else supRCurrent = supRTarget; supRight.write(map((int)supRCurrent, 0, 100, SUP_R_RETRACT, SUP_R_EXTEND));
      if (abs(supFCurrent - supFTarget) > SPEED_MOTION) { if (supFCurrent < supFTarget) supFCurrent += SPEED_MOTION; else supFCurrent -= SPEED_MOTION; } else supFCurrent = supFTarget; supFront.write(map((int)supFCurrent, 0, 100, SUP_F_RETRACT, SUP_F_EXTEND));
    }
}
void runDemoSequence() { /* Demo Code bleibt unverändert */ }
void waitAndAnimate(int waitTime) { /* Helper bleibt unverändert */ }