#include <Arduino.h>
#include <Servo.h>

// =========================================================
//      MOSAM v4.3 - STATE CHANGE DETECTION (Anti-Flicker)
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
unsigned long lastOHPUpdate = 0; const int OHP_REFRESH_RATE = 50; // Schnellerer Check, da wir nur bei Änderung senden
bool debugMode = false; unsigned long lastDebugTime = 0;
bool ohpConnected = false; 

// --- STATE MEMORY (Zum Speichern des letzten Schalterzustands) ---
// Initialisieren mit -1 damit beim Start einmalig gesendet wird
int last_Beacon = -1;
int last_Strobe = -1;
int last_Nav = -1;
int last_Nose = -1;
int last_Landing = -1;
int last_RwyTurn = -1;
int last_Seatbelt = -1;
int last_Dome = -1;
int last_Call = -1;
int last_APUStart = -1;
// (Systemschalter hier der Kürze halber weggelassen oder können analog ergänzt werden)

// =========================================================
// 1. PANEL REFS (WRITE)
// =========================================================
FlightSimInteger pan_Beacon;
FlightSimInteger pan_Strobe; 
FlightSimInteger pan_Nav;         
FlightSimInteger pan_Nose; 
FlightSimInteger pan_LandingL;    
FlightSimInteger pan_LandingR;    
FlightSimInteger pan_LandingNose;
FlightSimInteger pan_RwyTurn;
FlightSimInteger pan_Seatbelt;    
FlightSimFloat   pan_DomeVal;     
FlightSimInteger pan_Call;        
FlightSimInteger pan_APUStart;    

// FORCE LIGHT REFS (Dein Wunsch: Licht direkt setzen)
FlightSimInteger force_BeaconLi;
FlightSimInteger force_StrobeLi;
FlightSimInteger force_NavLi;
FlightSimInteger force_LandingLi;

// =========================================================
// 2. MODEL REFS (READ)
// =========================================================
FlightSimInteger mod_NavLight; 
FlightSimInteger mod_BeaconLight; 
FlightSimInteger mod_StrobeLight;
FlightSimInteger mod_LandingLight; 
FlightSimInteger mod_TaxiLight;    
FlightSimFloat   mod_RwyTurnSwitch; 

// System Feedback
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
  mod_RwyTurnSwitch = XPlaneRef("ckpt/oh/rwyTurnOff/anim"); 

  xGearHandle = XPlaneRef("sim/cockpit2/controls/gear_handle_down");
  xOnGround = XPlaneRef("sim/flightmodel/failures/onground_any"); 
  xBank = XPlaneRef("sim/flightmodel/position/phi");    
  xPitch = XPlaneRef("sim/flightmodel/position/true_theta"); 
  xEng1N1 = XPlaneRef("AirbusFBW/fmod/eng/N1Array[0]");
  xEng2N1 = XPlaneRef("AirbusFBW/fmod/eng/N1Array[1]");

  // --- B. PANEL REFS (WRITE - SWITCHES) ---
  pan_Beacon    = XPlaneRef("sim/cockpit2/switches/beacon_on");      
  pan_Strobe    = XPlaneRef("sim/cockpit2/switches/strobe_lights_on");      
  pan_Nav       = XPlaneRef("sim/cockpit2/switches/navigation_lights_on");  
  pan_Nose      = XPlaneRef("sim/cockpit2/switches/taxi_light_on");        
  pan_LandingL  = XPlaneRef("sim/cockpit2/switches/landing_lights_switch[0]");    
  pan_LandingR  = XPlaneRef("sim/cockpit2/switches/landing_lights_switch[1]");
  pan_LandingNose = XPlaneRef("sim/cockpit2/switches/landing_lights_switch[2]");
  pan_RwyTurn   = XPlaneRef("sim/cockpit2/switches/runway_turnoff_lights_on");     
  pan_Seatbelt  = XPlaneRef("sim/cockpit2/switches/fasten_seat_belts");    
  pan_DomeVal   = XPlaneRef("sim/cockpit2/electrical/instrument_brightness_ratio[0]"); 
  pan_Call      = XPlaneRef("AirbusFBW/OH/CallMech"); 
  pan_APUStart  = XPlaneRef("sim/cockpit2/electrical/APU_starter_switch");

  // --- C. FORCE LIGHT REFS (FORCE OVERRIDE) ---
  force_BeaconLi = XPlaneRef("AirbusFBW/OH/Lights/Beacon");
  force_StrobeLi = XPlaneRef("AirbusFBW/OH/Lights/Strobe");
  force_NavLi    = XPlaneRef("AirbusFBW/OH/Lights/NavAndLogo");
  force_LandingLi = XPlaneRef("AirbusFBW/OH/Lights/LandingL");

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

  Serial.println("--- MOSAM v4.3 ANTI-FLICKER ---");
}

void loop() {
  FlightSim.update(); 
  
  if (digitalRead(pinProgButton) == LOW) { delay(50); if (digitalRead(pinProgButton) == LOW) { if (millis() < 30000 && !demoHasRun) runDemoSequence(); else { analogWrite(pinEng1, 0); analogWrite(pinEng2, 0); Serial.println("PROG"); delay(100); asm("bkpt #251"); }}}

  // 1. UPDATE PANEL INPUTS (Mit State Change Detection)
  unsigned long now = millis();
  
  // OHNE DETECT-CHECK (wie gewünscht immer aktiv)
  if (!demoModeActive && !calMode) {
      if (now - lastOHPUpdate >= OHP_REFRESH_RATE) { lastOHPUpdate = now; updatePanelInputs(); }
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
// STEP 1: PANEL LOGIC (STATE CHANGE ONLY)
// =========================================================
void updatePanelInputs() {
  
  // --- BEACON ---
  int cur_Beacon = (digitalRead(sw_Beacon) == LOW) ? 1 : 0;
  if (cur_Beacon != last_Beacon) {
      pan_Beacon = cur_Beacon;
      if(cur_Beacon == 1) force_BeaconLi = 1; // Force Light
      last_Beacon = cur_Beacon;
  }
  
  // --- STROBE ---
  int cur_Strobe = (digitalRead(sw_Strobe_On) == LOW) ? 1 : 0;
  if (cur_Strobe != last_Strobe) {
      pan_Strobe = cur_Strobe;
      if(cur_Strobe == 1) force_StrobeLi = 2; // Force ToLiss ON (2)
      last_Strobe = cur_Strobe;
  }

  // --- NAV ---
  int cur_Nav = (digitalRead(sw_Nav_Master) == LOW) ? 1 : 0;
  if (cur_Nav != last_Nav) {
      pan_Nav = cur_Nav;
      if(cur_Nav == 1) force_NavLi = 2; // Force ToLiss ON (2)
      last_Nav = cur_Nav;
  }
  
  // --- LANDING ---
  int cur_Landing = (digitalRead(sw_Landing_Master) == LOW) ? 1 : 0;
  if (cur_Landing != last_Landing) {
      if (cur_Landing == 1) { 
          pan_LandingL = 1; pan_LandingR = 1; pan_LandingNose = 1; 
          force_LandingLi = 2; // Force ON
      } else { 
          pan_LandingL = 0; pan_LandingR = 0; pan_LandingNose = 0; 
      }
      last_Landing = cur_Landing;
  }
  
  // --- SEATBELTS ---
  int cur_Seatbelt = (digitalRead(sw_Seatbelts) == LOW) ? 1 : 0;
  if (cur_Seatbelt != last_Seatbelt) {
      pan_Seatbelt = cur_Seatbelt; // 1=On
      last_Seatbelt = cur_Seatbelt;
  }
  
  // --- DOME ---
  int cur_Dome = (digitalRead(sw_Dome_Dim) == LOW) ? 1 : 0;
  if (cur_Dome != last_Dome) {
      if (cur_Dome == 1) pan_DomeVal = 0.5; else pan_DomeVal = 0.0;
      last_Dome = cur_Dome;
  }

  // --- CALL ---
  int cur_Call = (digitalRead(sw_Call_Btn) == LOW) ? 1 : 0;
  if (cur_Call != last_Call) {
      pan_Call = cur_Call;
      last_Call = cur_Call;
  }

  // --- APU START ---
  int cur_APUStart = (digitalRead(sw_APU_Start) == LOW) ? 1 : 0;
  if (cur_APUStart != last_APUStart) {
      pan_APUStart = cur_APUStart;
      last_APUStart = cur_APUStart;
  }

  // --- NOSE LOGIC ---
  int cur_Nose = 0; // Default Off
  if (digitalRead(sw_Nose_TO) == LOW) cur_Nose = 2; // TO
  else if (digitalRead(sw_Nose_Taxi) == HIGH) cur_Nose = 1; // TAXI (wenn beide High = Taxi, da Pin 19 Logik)
  
  if (cur_Nose != last_Nose) {
      // Sim Taxi Light Ref versteht nur 0 oder 1.
      if (cur_Nose > 0) pan_Nose = 1; else pan_Nose = 0;
      last_Nose = cur_Nose;
  }

  // --- REST (Ohne State Change, da weniger kritisch, aber besser wäre auch hier) ---
  if (digitalRead(sw_RwyTurnoff) == LOW) pan_RwyTurn = 1; else pan_RwyTurn = 0;
  // ... Restliche Systeme wie gehabt
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
// DEBUG TABLE (FIXED PINS 17-25)
// =========================================================
void printDebugTable() {
    Serial.println(); 
    Serial.print("TIME: "); Serial.print(millis()/1000); Serial.print(" | PANEL: "); Serial.println(digitalRead(pinOHP_Detect)==LOW ? "CONN" : "DISC");
    auto stateStr = [](int pin) { return (digitalRead(pin) == LOW) ? "ON " : "OFF"; };

    Serial.println("--- DB25 PINS 1-8 ---");
    Serial.print("02/01/DETECT /"); Serial.println(stateStr(pinOHP_Detect));
    Serial.print("03/18/BEACON /"); Serial.print(stateStr(sw_Beacon)); Serial.print(" -> L_State: "); Serial.println(last_Beacon);
    Serial.print("04/16/STROBE /"); Serial.print(stateStr(sw_Strobe_On)); Serial.print(" -> L_State: "); Serial.println(last_Strobe);
    Serial.print("05/17/NAV    /"); Serial.print(stateStr(sw_Nav_Master)); Serial.print(" -> L_State: "); Serial.println(last_Nav);
    Serial.print("06/19/NOSE_TX/"); Serial.println(stateStr(sw_Nose_Taxi));
    Serial.print("07/20/NOSE_TO/"); Serial.println(stateStr(sw_Nose_TO));
    Serial.print("08/21/RWY_TRN/"); Serial.println(stateStr(sw_RwyTurnoff));
    
    Serial.println("--- DB25 PINS 9-16 ---");
    Serial.print("09/22/LANDING/"); Serial.print(stateStr(sw_Landing_Master)); Serial.print(" -> L_State: "); Serial.println(last_Landing);
    Serial.print("10/24/SEATBLT/"); Serial.println(stateStr(sw_Seatbelts));
    Serial.print("11/25/DOME   /"); Serial.println(stateStr(sw_Dome_Dim));
    Serial.print("14/29/CALL   /"); Serial.println(stateStr(sw_Call_Btn));
    
    Serial.println("--- DB25 PINS 17-25 (FIXED) ---");
    Serial.print("17/34/APU_MAS/"); Serial.println(stateStr(sw_APU_Master));
    Serial.print("18/35/APU_STR/"); Serial.println(stateStr(sw_APU_Start));
    Serial.print("19/36/APU_BLD/"); Serial.println(stateStr(sw_APU_Bleed));
    Serial.print("20/37/X_BLEED/"); Serial.println(stateStr(sw_XBleed_Open));
    Serial.print("21/38/EL_PUMP/"); Serial.println(stateStr(sw_ElecPump));
    Serial.print("22/39/PTU    /"); Serial.println(stateStr(sw_PTU_Off));
    Serial.print("23/40/PACK_1 /"); Serial.println(stateStr(sw_Pack1));
    Serial.print("24/32/PACK_2 /"); Serial.println(stateStr(sw_Pack2));
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