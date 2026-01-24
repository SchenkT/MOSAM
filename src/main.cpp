#include <Arduino.h>
#include <Servo.h>

// =========================================================
//      MOSAM v2.13 - FINAL (Seatbelt Commands + Dome Fix)
// =========================================================

// --- SERVO OBJEKTE ---
Servo noseGear; Servo mainGear;
Servo supLeft; Servo supRight; Servo supFront; 

// --- PINS (HARDWARE) ---
const int pinNoseLight = 2; const int pinNoseServo = 3; const int pinMainServo = 4;  
const int pinWingStrobes = 5; const int pinTailCombined = 6; const int pinWingNavs = 7;  
const int pinBeacon = 8; const int pinEng1 = 9; const int pinEng2 = 10; 
const int pinLanding = 11; const int pinSupLeft = 12; const int pinSupRight = 13; 
const int pinSupFront = 41; const int pinProgButton = 33; 

// --- INPUT PINS ---
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
const int sw_Wiper_Slow = 27; const int sw_Wiper_Fast = 28; 
const int sw_Call_Btn = 29; 
const int sw_Ice_Wing = 30; const int sw_Ice_Eng_Comb = 31; 
const int sw_APU_Master = 34; const int sw_APU_Start = 35; const int sw_APU_Bleed = 36; 
const int sw_XBleed_Open = 37; const int sw_ElecPump = 38; const int sw_PTU_Off = 39; 
const int sw_Pack1 = 40; const int sw_Pack2 = 32; 

// --- VARS ---
unsigned long lastOHPUpdate = 0; const int OHP_REFRESH_RATE = 150; bool ohpConnected = false; 
bool debugMode = false; unsigned long lastDebugTime = 0;

// Hilfsvariable für Seatbelt Command-Logik (damit wir nicht spammen)
int lastSeatbeltState = -1; 

// --- COMMANDS (NEU) ---
FlightSimCommand cmdSeatbeltOn;
FlightSimCommand cmdSeatbeltOff;

// --- DATAREFS (WRITE) ---
FlightSimInteger wBeacon;
FlightSimInteger wStrobe; 
FlightSimInteger wNav;         
FlightSimInteger wNose; 
FlightSimInteger wLandingL;    
FlightSimInteger wLandingR;    
FlightSimInteger wLandingNose; 
FlightSimInteger wRwyTurn;
// wSeatbelt entfernt -> Jetzt Commands
FlightSimInteger wDomeSwitch;  
FlightSimFloat   wDomeLightVal;
FlightSimInteger wWiper; 
FlightSimInteger wCall;        
FlightSimInteger wIceWing; 
FlightSimInteger wIceEng1; FlightSimInteger wIceEng2;
FlightSimInteger wAPUMaster;
FlightSimInteger wAPUStart;    
FlightSimInteger wAPUBleed;
FlightSimInteger wXBleed; 
FlightSimInteger wElecPump;
FlightSimInteger wPTU;
FlightSimInteger wPack1; FlightSimInteger wPack2;

// --- DATAREFS (READ) ---
FlightSimInteger xNavLight; FlightSimInteger xBeaconLight; FlightSimInteger xStrobeLight;
FlightSimInteger xLandingLight; FlightSimInteger xNoseSwitch; FlightSimFloat xRwyTurnSwitch; 
FlightSimInteger xGearHandle; FlightSimFloat xOnGround; FlightSimFloat xBank; FlightSimFloat xPitch;       
FlightSimFloat xEng1N1; FlightSimFloat xEng2N1;      

// --- INTERNALS ---
bool strobesActive = false; bool navsActive = false; bool beaconActive = false;
bool landingActive = false; bool turnoffActive = false; 
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
const int MOTION_NEUTRAL = 50; 
const int AIR_LIFT_OFFSET = 15; 

// Forward Decl
void updateHydraulics(); void updateOHPInputs(); void waitAndAnimate(int waitTime); void runDemoSequence();

void setup() {
  Serial.begin(9600);
  
  // --- READ REFS ---
  xNavLight = XPlaneRef("sim/cockpit2/switches/navigation_lights_on");
  xBeaconLight = XPlaneRef("sim/cockpit2/switches/beacon_on");
  xStrobeLight = XPlaneRef("sim/cockpit2/switches/strobe_lights_on"); 
  xLandingLight = XPlaneRef("sim/cockpit2/switches/landing_lights_on");
  xNoseSwitch = XPlaneRef("ckpt/oh/taxiLight/anim");    
  xRwyTurnSwitch = XPlaneRef("ckpt/oh/rwyTurnOff/anim");   
  xGearHandle = XPlaneRef("sim/cockpit2/controls/gear_handle_down");
  xOnGround = XPlaneRef("sim/flightmodel/failures/onground_any"); 
  xBank = XPlaneRef("sim/flightmodel/position/phi");    
  xPitch = XPlaneRef("sim/flightmodel/position/true_theta"); 
  xEng1N1 = XPlaneRef("AirbusFBW/fmod/eng/N1Array[0]");
  xEng2N1 = XPlaneRef("AirbusFBW/fmod/eng/N1Array[1]");

  // --- WRITE REFS ---
  wBeacon    = XPlaneRef("sim/cockpit2/switches/beacon_on");
  wStrobe    = XPlaneRef("ckpt/oh/strobeLight/anim"); 
  wNav       = XPlaneRef("AirbusFBW/OH/Lights/NavAndLogo"); 
  wNose      = XPlaneRef("ckpt/oh/taxiLight/anim"); 
  wLandingL  = XPlaneRef("AirbusFBW/OH/Lights/LandingL");
  wLandingR  = XPlaneRef("AirbusFBW/OH/Lights/LandingR");
  wLandingNose = XPlaneRef("AirbusFBW/OH/Lights/Nose"); 
  wRwyTurn   = XPlaneRef("ckpt/oh/rwyTurnOff/anim");
  
  // SEATBELTS = COMMANDS
  cmdSeatbeltOn  = XPlaneRef("toliss_airbus/lightcommands/FSBSignOn");
  cmdSeatbeltOff = XPlaneRef("toliss_airbus/lightcommands/FSBSignOff");
  
  // DOME REFS
  wDomeSwitch   = XPlaneRef("ckpt/oh/domeLight/anim"); 
  wDomeLightVal = XPlaneRef("sim/cockpit/electrical/cockpit_lights");

  wWiper     = XPlaneRef("sim/cockpit2/switches/wiper_speed"); 
  wCall      = XPlaneRef("AirbusFBW/OH/CallMech"); 
  wIceWing   = XPlaneRef("sim/cockpit2/ice/ice_inlet_heat_on_per_engine[0]"); 
  wIceEng1   = XPlaneRef("sim/cockpit2/ice/ice_inlet_heat_on_per_engine[0]");
  wIceEng2   = XPlaneRef("sim/cockpit2/ice/ice_inlet_heat_on_per_engine[1]");
  wAPUMaster = XPlaneRef("sim/cockpit2/electrical/APU_master_switch");
  wAPUStart  = XPlaneRef("AirbusFBW/OH/APUStart");
  wAPUBleed  = XPlaneRef("sim/cockpit2/bleedair/apu_bleed_on");
  wXBleed    = XPlaneRef("sim/cockpit2/bleedair/cross_tie_open"); 
  wElecPump  = XPlaneRef("sim/cockpit2/hydraulics/actuators/electric_hydraulic_pump_on");
  wPTU       = XPlaneRef("sim/cockpit2/hydraulics/actuators/ptu_on");
  wPack1     = XPlaneRef("sim/cockpit2/bleedair/bleed_air_on[0]");
  wPack2     = XPlaneRef("sim/cockpit2/bleedair/bleed_air_on[1]");

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

  Serial.println("--- MOSAM v2.13 SEATBELT COMMANDS ---");
}

void loop() {
  FlightSim.update(); 
  
  if (digitalRead(pinProgButton) == LOW) { delay(50); if (digitalRead(pinProgButton) == LOW) { if (millis() < 30000 && !demoHasRun) runDemoSequence(); else { analogWrite(pinEng1, 0); analogWrite(pinEng2, 0); Serial.println("PROG"); delay(100); asm("bkpt #251"); }}}

  unsigned long now = millis();
  if (!demoModeActive && !calMode) {
      if (digitalRead(pinOHP_Detect) == LOW) {
          ohpConnected = true;
          if (now - lastOHPUpdate >= OHP_REFRESH_RATE) { lastOHPUpdate = now; updateOHPInputs(); }
      } else ohpConnected = false; 
  }

  // Debugging & Serial
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n'); cmd.trim();
    if (cmd.equalsIgnoreCase("debug")) debugMode = true; else if (cmd.equalsIgnoreCase("debugstop")) debugMode = false;
    else if (cmd.startsWith("cal_left")) { calMode = true; manualRollOffset = cmd.substring(9).toInt(); calTargetBank = -40; }
    else if (cmd == "cal_off") { calMode = false; manualRollOffset = 0; }
  }
  if (debugMode && (millis() - lastDebugTime > 1000)) {
      lastDebugTime = millis(); Serial.print("DBG > ");
      for (int i=14; i<=41; i++) { if (i!=33) { Serial.print(i); Serial.print(":"); Serial.print(digitalRead(i)==LOW?"L":"H"); Serial.print(" "); }} Serial.println();
  }

  // LED & Servo Feedback Logic
  if ((FlightSim.isEnabled() || calMode) && !demoModeActive) { 
    if (!calMode) {
        navsActive = xNavLight; beaconActive = xBeaconLight; long strbVal = xStrobeLight; strobesActive = (strbVal > 0);
        landingActive = xLandingLight; turnoffActive = (xRwyTurnSwitch > 0.1);
    }
    if (xGearHandle == 1) { int ns = xNoseSwitch; if (ns == 2) analogWrite(pinNoseLight, (int)(VAL_TO * brightnessScale)); else if (ns == 1) analogWrite(pinNoseLight, (int)(VAL_TAXI * brightnessScale)); else analogWrite(pinNoseLight, 0); } else analogWrite(pinNoseLight, 0); 
    
    if (run_eng == 1 && !calMode) {
       int t1=0; if (xEng1N1 > 1) t1 = map((int)(float)xEng1N1, 15, 100, 0, 50); analogWrite(pinEng1, t1);
       int t2=0; if (xEng2N1 > 1) t2 = map((int)(float)xEng2N1, 15, 100, 0, 50); analogWrite(pinEng2, t2);
    }
    if (navsActive) analogWrite(pinWingNavs, (int)(40*brightnessScale)); else analogWrite(pinWingNavs, 0);
    if (landingActive) analogWrite(pinLanding, (int)(100*brightnessScale)); else if (turnoffActive) analogWrite(pinLanding, (int)(40*brightnessScale)); else analogWrite(pinLanding, 0);
  }

  if (!demoModeActive) {
     updateHydraulics();
     unsigned long mt = millis() % 1000; 
     if (beaconActive && mt >= 500 && mt < 600) analogWrite(pinBeacon, (int)(255*brightnessScale)); else analogWrite(pinBeacon, 0);
     delay(REFRESH_RATE);
  }
}

// =========================================================
// OHP INPUT LOGIC
// =========================================================
void updateOHPInputs() {
  
  if (digitalRead(sw_Beacon) == LOW) wBeacon = 1; else wBeacon = 0;
  if (digitalRead(sw_Strobe_On) == LOW) wStrobe = 2; else wStrobe = 1;

  if (digitalRead(sw_Nav_Master) == LOW) wNav = 1; else wNav = 0; 
  if (digitalRead(sw_Landing_Master) == LOW) { wLandingL = 2; wLandingR = 2; wLandingNose = 2; } else { wLandingL = 0; wLandingR = 0; wLandingNose = 0; }
  
  // --- SEATBELT COMMAND LOGIC (State Change Detection) ---
  int currentSeatbeltState = digitalRead(sw_Seatbelts); // LOW=On, HIGH=Off
  
  // Nur wenn sich der Zustand geändert hat, senden wir den Befehl EINMAL
  if (currentSeatbeltState != lastSeatbeltState) {
      if (currentSeatbeltState == LOW) {
          cmdSeatbeltOn.once();  // "SignOn"
      } else {
          cmdSeatbeltOff.once(); // "SignOff"
      }
      lastSeatbeltState = currentSeatbeltState; // Zustand merken
  }
  
  // DOME FIX
  if (digitalRead(sw_Dome_Dim) == LOW) {
      wDomeSwitch = 1;      
      wDomeLightVal = 0.45; 
  } else {
      wDomeSwitch = 0;      
      wDomeLightVal = 0.0;  
  }

  if (digitalRead(sw_Call_Btn) == LOW) wCall = 1; else wCall = 0;
  if (digitalRead(sw_APU_Start) == LOW) wAPUStart = 1; else wAPUStart = 0;

  // NOSE
  if (digitalRead(sw_Nose_Taxi) == LOW) { wNose = 0; } 
  else if (digitalRead(sw_Nose_TO) == LOW) { wNose = 2; } 
  else { wNose = 1; }

  // REST
  if (digitalRead(sw_RwyTurnoff) == LOW) wRwyTurn = 1; else wRwyTurn = 0;
  if (digitalRead(sw_Wiper_Fast) == LOW) wWiper = 3; else if (digitalRead(sw_Wiper_Slow) == LOW) wWiper = 2; else wWiper = 0;
  if (digitalRead(sw_Ice_Wing) == LOW) wIceWing = 1; else wIceWing = 0;
  if (digitalRead(sw_Ice_Eng_Comb) == LOW) { wIceEng1 = 1; wIceEng2 = 1; } else { wIceEng1 = 0; wIceEng2 = 0; }
  if (digitalRead(sw_APU_Master) == LOW) wAPUMaster = 1; else wAPUMaster = 0;
  if (digitalRead(sw_APU_Bleed) == LOW) wAPUBleed = 1; else wAPUBleed = 0;
  if (digitalRead(sw_XBleed_Open) == LOW) wXBleed = 1; else wXBleed = 2; 
  if (digitalRead(sw_ElecPump) == LOW) wElecPump = 1; else wElecPump = 0;
  if (digitalRead(sw_PTU_Off) == LOW) wPTU = 0; else wPTU = 1; 
  if (digitalRead(sw_Pack1) == LOW) wPack1 = 1; else wPack1 = 0;
  if (digitalRead(sw_Pack2) == LOW) wPack2 = 1; else wPack2 = 0;
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
void runDemoSequence() { 
  Serial.println(">> DEMO"); demoModeActive = true; demoHasRun = true;
  navsActive=0; beaconActive=0; strobesActive=0; landingActive=0; turnoffActive=0; demoNoseLightVal = VAL_OFF; run_eng = 0; eng1TargetPWM = 0; eng2TargetPWM = 0;
  noseTarget = NOSE_POS_DOWN; mainTarget = MAIN_POS_DOWN; supLTarget=0; supRTarget=0; supFTarget=0; 
  navsActive = true; waitAndAnimate(2000); beaconActive = true; waitAndAnimate(2000);
  run_eng = 1; eng1TargetPWM = ENG_IDLE_MIN; waitAndAnimate(3000); eng2TargetPWM = ENG_IDLE_MIN; waitAndAnimate(3000); 
  demoNoseLightVal = VAL_TAXI; turnoffActive = true; waitAndAnimate(2000); strobesActive = true; waitAndAnimate(2000);
  demoNoseLightVal = VAL_TO; landingActive = true; waitAndAnimate(2000); eng1TargetPWM = 40; eng2TargetPWM = 40; waitAndAnimate(2000);
  supFTarget = 70; supLTarget = 0; supRTarget = 0; waitAndAnimate(2000);
  int base = MOTION_NEUTRAL + AIR_LIFT_OFFSET;
  int pitchMove = 35; supFTarget = constrain(base + pitchMove, 0, 100); supLTarget = constrain(base - pitchMove, 0, 100); supRTarget = constrain(base - pitchMove, 0, 100); waitAndAnimate(4000); 
  noseTarget = NOSE_POS_UP; mainTarget = MAIN_POS_UP; waitAndAnimate(4000); 
  pitchMove = 10; int rollMove = -35; supFTarget = constrain(base + pitchMove, 0, 100); supLTarget = constrain(base + rollMove - pitchMove, 0, 100); supRTarget = constrain(base - rollMove - pitchMove, 0, 100); waitAndAnimate(5000); 
  landingActive = false; turnoffActive = false; demoNoseLightVal = VAL_OFF; waitAndAnimate(2000);
  supFTarget = base; supLTarget = base; supRTarget = base; waitAndAnimate(4000); 
  rollMove = 35; supFTarget = constrain(base, 0, 100); supLTarget = constrain(base + rollMove, 0, 100); supRTarget = constrain(base - rollMove, 0, 100); waitAndAnimate(5000); 
  pitchMove = -10; supFTarget = constrain(base + pitchMove, 0, 100); supLTarget = constrain(base - pitchMove, 0, 100); supRTarget = constrain(base - pitchMove, 0, 100);
  noseTarget = NOSE_POS_DOWN; mainTarget = MAIN_POS_DOWN; eng1TargetPWM = ENG_IDLE_MIN; eng2TargetPWM = ENG_IDLE_MIN; waitAndAnimate(5000); 
  landingActive = true; demoNoseLightVal = VAL_TO; waitAndAnimate(2000);
  pitchMove = 15; supFTarget = constrain(base + pitchMove, 0, 100); supLTarget = constrain(base - pitchMove, 0, 100); supRTarget = constrain(base - pitchMove, 0, 100); waitAndAnimate(3000);
  supLTarget = 0; supRTarget = 0; supFTarget = 0; waitAndAnimate(1000); waitAndAnimate(2000);
  strobesActive = false; landingActive = false; waitAndAnimate(2000); demoNoseLightVal = VAL_OFF; waitAndAnimate(2000);
  eng1TargetPWM = 0; eng2TargetPWM = 0; waitAndAnimate(2000); beaconActive = false; waitAndAnimate(2000); navsActive = false; waitAndAnimate(2000);
  Serial.println(">> END"); demoModeActive = false;
}
void waitAndAnimate(int waitTime) {
  unsigned long start = millis();
  while (millis() - start < (unsigned long)waitTime) {
    updateHydraulics();
    unsigned long nowEngine = millis();
    if (eng1KickEnd > 0 && nowEngine > eng1KickEnd) { analogWrite(pinEng1, eng1TargetPWM); eng1KickEnd = 0; }
    if (eng2KickEnd > 0 && nowEngine > eng2KickEnd) { analogWrite(pinEng2, eng2TargetPWM); eng2KickEnd = 0; }
    delay(REFRESH_RATE);
  }
}