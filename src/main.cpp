#include <Arduino.h>
#include <Servo.h>

// =========================================================
//      MOSAM v6.8 - A330 AUTO-DETECT & LOGIC UPDATE
// =========================================================

// --- SERVO OBJEKTE ---
Servo noseGear; Servo mainGear;
Servo supLeft; Servo supRight; Servo supFront; 

// --- PINS (OUTPUTS MODELL) ---
const int pinNoseLight = 2; const int pinNoseServo = 3; const int pinMainServo = 4;  
const int pinWingStrobes = 5; const int pinTailCombined = 6; const int pinWingNavs = 7;  
const int pinBeacon = 8; const int pinEng1 = 9; const int pinEng2 = 10; 
const int pinLanding = 11; const int pinSupLeft = 12; const int pinSupRight = 13; 
const int pinSupFront = 41; const int pinProgButton = 33; 

// --- PINS (INPUTS PANEL - SUB-D MAPPING) ---
const int pinOHP_Detect = 1;   // P01 (Detect)
const int sw_Beacon = 18;//15;      // P03 (Pin 15)
const int sw_Strobe_On = 16;   // P04
const int sw_Nav_Master = 17;  // P05
const int sw_Nose_Taxi = 19;//23;   // P06 (Pin 23)
const int sw_Nose_TO = 20;     // P07
const int sw_RwyTurnoff = 21;  // P08
const int sw_Landing_Master = 22; // P09
const int sw_Seatbelts = 24;   // P10
const int sw_Dome_Dim = 25;    // P11
const int sw_Wiper_Slow = 27;  // P12
const int sw_Wiper_Fast = 28;  // P13
const int sw_Call_Btn = 29;    // P14
const int sw_Ice_Wing = 30;    // P15
const int sw_Ice_Eng_Comb = 31; // P16
const int sw_APU_Master = 34;  // P17
const int sw_APU_Start = 35;   // P18
const int sw_APU_Bleed = 36;   // P19
const int sw_XBleed_Open = 37; // P20
const int sw_ElecPump = 38;    // P21
const int sw_PTU_Off = 39;     // P22
const int sw_Pack1 = 40;       // P23
const int sw_Pack2 = 32;       // P24

// --- VARS ---
unsigned long lastOHPUpdate = 0; const int OHP_REFRESH_RATE = 50; 
bool debugMode = false; unsigned long lastDebugTime = 0;
bool ohpConnected = false;

// --- SAFETY TIMERS ---
unsigned long changeTimer[50]; 
const unsigned long BLOCK_TIME = 1200; 

// Startup & Detect Vars
unsigned long startupTimer = 0;
unsigned long detectDebounceStart = 0;
bool detectPinStable = false;
const unsigned long STARTUP_DELAY = 1500; 
const unsigned long DETECT_STABLE_TIME = 500; 

// --- A330 DETECTION VARS ---
bool isA330 = false;
bool typeCheckDone = false;
char acf_icao_buf[10]; // Puffer für ICAO Code

// --- STATE MEMORY ---
int last_Beacon = -1; int last_Strobe = -1; int last_Nav = -1;
int last_Nose = -1; int last_Land = -1; int last_Rwy = -1;
int last_Seat = -1; int last_Dome = -1; int last_WiperS = -1; int last_WiperF = -1;
int last_Call = -1; int last_IceW = -1; int last_IceE = -1;
int last_APUM = -1; int last_APUS = -1; int last_APUB = -1;
int last_XBleed = -1; int last_Elec = -1; int last_PTU = -1;
int last_Pack1 = -1; int last_Pack2 = -1;

// --- WRITE REFS ---
FlightSimInteger pan_Beacon; 
FlightSimInteger pan_Nav;    
FlightSimInteger pan_Nose;   
FlightSimInteger pan_LandL;  
FlightSimInteger pan_LandR;  
FlightSimInteger pan_Rwy;    
FlightSimInteger pan_Strobe; 
FlightSimInteger pan_Dome;   
FlightSimInteger pan_Seat;   

FlightSimInteger pan_Wiper;     
FlightSimCommand cmd_Call;      
FlightSimInteger pan_IceW;      
FlightSimInteger pan_IceE1;     
FlightSimInteger pan_IceE2;     
FlightSimInteger pan_APUM;      
FlightSimInteger pan_APUS;      
FlightSimInteger pan_APUB;
FlightSimInteger pan_XBleed;    
FlightSimInteger pan_HydElec;   
FlightSimInteger pan_HydPTU;    
FlightSimInteger pan_Pack1;     
FlightSimInteger pan_Pack2;     

// --- NEW A330 REFS ---
FlightSimInteger pan_APUB_330; 
FlightSimInteger pan_Dome_330;
FlightSimInteger pan_NoPed_330; // [12] No PED Signs
FlightSimString acf_icao_ref;   // Zur Erkennung

// --- READ REFS ---
FlightSimInteger mod_Nav; FlightSimInteger mod_Beacon; FlightSimInteger mod_Strobe;
FlightSimInteger mod_Land; FlightSimInteger mod_Taxi; FlightSimInteger mod_Rwy; 
FlightSimInteger mod_Dome; 
FlightSimInteger mod_Seat; FlightSimInteger mod_APU; 

FlightSimInteger xGearHandle; FlightSimFloat xOnGround; 
FlightSimFloat xBank; FlightSimFloat xPitch;       
FlightSimFloat xEng1N1; FlightSimFloat xEng2N1;      

// --- INTERNALS ---
float noseCurrent = 5; int noseTarget = 5; float mainCurrent = 5; int mainTarget = 5;
float supLCurrent = 0; int supLTarget = 0; float supRCurrent = 0; int supRTarget = 0; float supFCurrent = 0; int supFTarget = 0;
unsigned long lastMoveTime = 0; int eng1TargetPWM = 0; int eng2TargetPWM = 0;
unsigned long eng1KickEnd = 0; unsigned long eng2KickEnd = 0; int run_eng = 0; 

// DIMMING
float brightnessScale = 1.0; 
bool serialNightMode = false;

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
void updateHydraulics(); void updatePanelInputs(); void updateModelOutputs(); void waitAndAnimate(int waitTime); 
void runDemoSequence(); void printRow(String sub, int pin, String name, int switchVal, int lastVal, long simVal); 
void printDebugTable(); void runServoTest();

void setup() {
  Serial.begin(9600);
  for(int i=0; i<50; i++) changeTimer[i] = 0;
  startupTimer = millis(); 

  // --- READ REFS ---
  mod_Nav     = XPlaneRef("sim/cockpit2/switches/navigation_lights_on");
  mod_Beacon  = XPlaneRef("sim/cockpit2/switches/beacon_on");
  mod_Strobe  = XPlaneRef("sim/cockpit2/switches/strobe_lights_on"); 
  mod_Land    = XPlaneRef("sim/cockpit2/switches/landing_lights_on");
  mod_Taxi    = XPlaneRef("sim/cockpit2/switches/taxi_light_on"); 
  mod_Rwy     = XPlaneRef("sim/cockpit2/switches/runway_turnoff_lights_on");
  mod_Dome    = XPlaneRef("AirbusFBW/OH/Lights/Dome"); 
  mod_Seat    = XPlaneRef("AirbusFBW/SeatBeltSignsOn");
  mod_APU     = XPlaneRef("AirbusFBW/OH/Lights/APUStart");

  xGearHandle = XPlaneRef("sim/cockpit2/controls/gear_handle_down");
  xOnGround = XPlaneRef("sim/flightmodel/failures/onground_any"); 
  xBank = XPlaneRef("sim/flightmodel/position/phi");    
  xPitch = XPlaneRef("sim/flightmodel/position/true_theta"); 
  xEng1N1 = XPlaneRef("AirbusFBW/fmod/eng/N1Array[0]");
  xEng2N1 = XPlaneRef("AirbusFBW/fmod/eng/N1Array[1]");
  
  // A330 DETECTION
  acf_icao_ref = XPlaneRef("sim/aircraft/view/acf_ICAO");

  // --- WRITE REFS ---
  pan_Beacon    = XPlaneRef("AirbusFBW/OHPLightSwitches[0]"); 
  pan_Nav       = XPlaneRef("AirbusFBW/OHPLightSwitches[2]"); 
  pan_Nose      = XPlaneRef("AirbusFBW/OHPLightSwitches[3]"); 
  pan_LandL     = XPlaneRef("AirbusFBW/OHPLightSwitches[4]"); 
  pan_LandR     = XPlaneRef("AirbusFBW/OHPLightSwitches[5]"); 
  pan_Rwy       = XPlaneRef("AirbusFBW/OHPLightSwitches[6]"); 
  pan_Strobe    = XPlaneRef("AirbusFBW/OHPLightSwitches[7]"); 
  pan_Dome      = XPlaneRef("AirbusFBW/OHPLightSwitches[8]"); 
  pan_Seat      = XPlaneRef("AirbusFBW/OHPLightSwitches[11]"); 

  pan_Wiper     = XPlaneRef("AirbusFBW/LeftWiperSwitch"); 
  cmd_Call      = XPlaneRef("AirbusFBW/purser/fwd"); 
  
  pan_HydElec   = XPlaneRef("AirbusFBW/HydOHPArray[3]"); 
  pan_HydPTU    = XPlaneRef("AirbusFBW/HydOHPArray[4]"); 
  pan_Pack1     = XPlaneRef("AirbusFBW/Pack1Switch");
  pan_Pack2     = XPlaneRef("AirbusFBW/Pack2Switch");
  pan_APUM      = XPlaneRef("AirbusFBW/APUMaster"); 
  pan_APUS      = XPlaneRef("AirbusFBW/APUStarter"); 
  
  pan_IceW      = XPlaneRef("AirbusFBW/WAISwitch"); 
  pan_IceE1     = XPlaneRef("AirbusFBW/ENG1AISwitch");
  pan_IceE2     = XPlaneRef("AirbusFBW/ENG2AISwitch");
  pan_APUB      = XPlaneRef("sim/cockpit2/bleedair/apu_bleed_on");
  pan_XBleed    = XPlaneRef("AirbusFBW/XBleedSwitch"); 

  // --- NEW A330 REFS ---
  pan_APUB_330  = XPlaneRef("AirbusFBW/APUBleedSwitch"); 
  pan_Dome_330  = XPlaneRef("AirbusFBW/OHPLightSwitches[13]"); 
  pan_NoPed_330 = XPlaneRef("AirbusFBW/OHPLightSwitches[12]");

  // PINS
  pinMode(pinNoseLight, OUTPUT); pinMode(pinWingStrobes, OUTPUT); pinMode(pinTailCombined, OUTPUT); 
  pinMode(pinWingNavs, OUTPUT); pinMode(pinBeacon, OUTPUT); pinMode(pinEng1, OUTPUT); 
  pinMode(pinEng2, OUTPUT); pinMode(pinLanding, OUTPUT); pinMode(pinProgButton, INPUT_PULLUP); 
  for (int i=14; i<=40; i++) { pinMode(i, INPUT_PULLUP); }
  pinMode(sw_Pack2, INPUT_PULLUP); pinMode(sw_Beacon, INPUT_PULLUP); pinMode(pinOHP_Detect, INPUT_PULLUP);

  // OUTPUTS
  analogWrite(pinNoseLight, 0); analogWrite(pinWingStrobes, 0); analogWrite(pinWingNavs, 0); 
  analogWrite(pinTailCombined, 0); analogWrite(pinBeacon, 0); analogWrite(pinEng1, 0); analogWrite(pinEng2, 0); 
  analogWrite(pinLanding, 0);

  // SERVO
  noseGear.attach(pinNoseServo); mainGear.attach(pinMainServo); supLeft.attach(pinSupLeft); supRight.attach(pinSupRight); supFront.attach(pinSupFront);
  noseGear.write((int)noseCurrent); mainGear.write((int)mainCurrent); supLeft.write(SUP_L_RETRACT); supRight.write(SUP_R_RETRACT); supFront.write(SUP_F_RETRACT);

  Serial.println("--- MOSAM v6.8 PACK FIX & A330 DETECT ---");
}

void loop() {
  FlightSim.update(); 
  unsigned long now = millis();

  if (digitalRead(pinProgButton) == LOW) { delay(50); if (digitalRead(pinProgButton) == LOW) { if (millis() < 30000 && !demoHasRun) runDemoSequence(); else { analogWrite(pinEng1, 0); analogWrite(pinEng2, 0); Serial.println("PROG"); delay(100); asm("bkpt #251"); }}}

  // 1. SAFETY & DETECT LOGIC
  bool safeToRun = (now - startupTimer > STARTUP_DELAY); 
  
  if (safeToRun) {
      if (digitalRead(pinOHP_Detect) == LOW) {
          if (detectDebounceStart == 0) detectDebounceStart = now; 
          else if (now - detectDebounceStart > DETECT_STABLE_TIME) {
              if (!ohpConnected) { Serial.println("SYS: PANEL CONNECTED"); }
              ohpConnected = true;
          }
      } else {
          detectDebounceStart = 0;
          if (ohpConnected) { Serial.println("SYS: PANEL DISCONNECTED"); }
          ohpConnected = false;
      }
  }

  // A330 Type Detection (One Time Check after 3 seconds)
  if (!typeCheckDone && now > 3000) {
      acf_icao_ref.read(acf_icao_buf, 9);
      String acfType = String(acf_icao_buf);
      if (acfType.indexOf("A33") != -1) {
          isA330 = true;
          Serial.println("SYS: A330 DETECTED - HYD PROTECTION ACTIVE");
      } else {
          isA330 = false;
          Serial.println("SYS: A320 MODE (STD)");
      }
      typeCheckDone = true;
  }

  // 2. PANEL INPUT UPDATE
  if (ohpConnected && !demoModeActive && !calMode) {
      if (now - lastOHPUpdate >= OHP_REFRESH_RATE) { lastOHPUpdate = now; updatePanelInputs(); }
  }

  if (!demoModeActive) { updateModelOutputs(); }

  // SERIAL COMMANDS
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n'); cmd.trim();
    if (cmd.equalsIgnoreCase("debug")) { debugMode = true; Serial.println("CMD OK: Debug ON"); } 
    else if (cmd.equalsIgnoreCase("debugstop")) { debugMode = false; Serial.println("CMD OK: Debug OFF"); }
    else if (cmd.equalsIgnoreCase("status")) { Serial.println("CMD OK: Status"); printDebugTable(); }
    else if (cmd.equalsIgnoreCase("engrun")) { run_eng = 1; Serial.println("CMD OK: Engines ON"); }
    else if (cmd.equalsIgnoreCase("engstop")) { run_eng = 0; analogWrite(pinEng1, 0); analogWrite(pinEng2, 0); Serial.println("CMD OK: Engines OFF"); }
    else if (cmd.equalsIgnoreCase("night")) { serialNightMode = true; Serial.println("CMD OK: Night Mode (50%)"); }
    else if (cmd.equalsIgnoreCase("day")) { serialNightMode = false; Serial.println("CMD OK: Day Mode (100%)"); }
    else if (cmd.equalsIgnoreCase("servotest")) { Serial.println("CMD OK: Testing Servos..."); runServoTest(); }
    else if (cmd.startsWith("cal_left")) { calMode = true; manualRollOffset = cmd.substring(9).toInt(); calTargetBank = -40; Serial.println("CMD OK: Cal Left"); }
    else if (cmd.equalsIgnoreCase("cal_off")) { calMode = false; manualRollOffset = 0; Serial.println("CMD OK: Cal Off"); }
    else { Serial.println("CMD: Unknown"); }
  }
  
  if (debugMode && (millis() - lastDebugTime > 500)) { 
      lastDebugTime = millis(); 
      printDebugTable();
  }
}

void updatePanelInputs() {
  int val; 
  // P03 Beacon
  val = (digitalRead(sw_Beacon) == LOW); if (val != last_Beacon) { pan_Beacon = val; last_Beacon = val; changeTimer[sw_Beacon] = millis(); }
  // P04 Strobe
  val = (digitalRead(sw_Strobe_On) == LOW) ? 2 : 1; if (val != last_Strobe) { pan_Strobe = val; last_Strobe = val; changeTimer[sw_Strobe_On] = millis(); }
  // P05 Nav
  val = (digitalRead(sw_Nav_Master) == LOW) ? 2 : 0; if (val != last_Nav) { pan_Nav = val; last_Nav = val; changeTimer[sw_Nav_Master] = millis(); }
  // P06/07 Nose
  int noseState = 0; if (digitalRead(sw_Nose_TO) == LOW) noseState = 2; else if (digitalRead(sw_Nose_Taxi) == HIGH) noseState = 1; 
  if (noseState != last_Nose) { pan_Nose = noseState; last_Nose = noseState; changeTimer[sw_Nose_TO] = millis(); }
  // P08 Rwy
  val = (digitalRead(sw_RwyTurnoff) == LOW); if (val != last_Rwy) { pan_Rwy = val; last_Rwy = val; changeTimer[sw_RwyTurnoff] = millis(); }
  
  // P09 Landing (A330 LOGIK CHANGE)
  // A330: ON=1, A320: ON=2
  int landTarget = 0;
  if (isA330) {
      landTarget = (digitalRead(sw_Landing_Master) == LOW) ? 1 : 0;
  } else {
      landTarget = (digitalRead(sw_Landing_Master) == LOW) ? 2 : 0;
  }
  if (landTarget != last_Land) { 
      pan_LandL = landTarget; pan_LandR = landTarget; 
      last_Land = landTarget; changeTimer[sw_Landing_Master] = millis(); 
  }

  // P10 Seatbelt (A330 LOGIK CHANGE)
  if (isA330) {
      // A330: Schalter AUS (HIGH) = 1 (AUTO), Schalter EIN (LOW) = 2 (ON)
      // Gleichzeitig [11] und [12] bedienen
      int seatVal = (digitalRead(sw_Seatbelts) == LOW) ? 2 : 1; 
      if (seatVal != last_Seat) {
          pan_Seat = seatVal;      // [11] Signs
          pan_NoPed_330 = seatVal; // [12] No PED
          last_Seat = seatVal;
          changeTimer[sw_Seatbelts] = millis();
      }
  } else {
      // A320 Logik (Original)
      val = (digitalRead(sw_Seatbelts) == LOW); 
      if (val != last_Seat) { pan_Seat = val; last_Seat = val; changeTimer[sw_Seatbelts] = millis(); }
  }
  
  // P11 Dome (UPDATE A330)
  val = (digitalRead(sw_Dome_Dim) == LOW); // 1=DIM/ON, 0=OFF
  if (val != last_Dome) { 
      pan_Dome = val; 
      pan_Dome_330 = val; // Index 13 für A330
      last_Dome = val; changeTimer[sw_Dome_Dim] = millis(); 
  }
  
  // P14 Call
  val = (digitalRead(sw_Call_Btn) == LOW); if (val != last_Call) { if (val == 1) cmd_Call.once(); last_Call = val; changeTimer[sw_Call_Btn] = millis(); }
  // P12/13 Wiper
  int wiperState = 0; if (digitalRead(sw_Wiper_Fast) == LOW) wiperState = 2; else if (digitalRead(sw_Wiper_Slow) == LOW) wiperState = 1;
  if (digitalRead(sw_Wiper_Slow) != last_WiperS || digitalRead(sw_Wiper_Fast) != last_WiperF) { pan_Wiper = wiperState; last_WiperS = digitalRead(sw_Wiper_Slow); last_WiperF = digitalRead(sw_Wiper_Fast); changeTimer[sw_Wiper_Slow] = millis(); }
  // P15/16 Ice
  val = (digitalRead(sw_Ice_Wing) == LOW); if (val != last_IceW) { pan_IceW = val; last_IceW = val; changeTimer[sw_Ice_Wing] = millis(); }
  val = (digitalRead(sw_Ice_Eng_Comb) == LOW); if (val != last_IceE) { pan_IceE1 = val; pan_IceE2 = val; last_IceE = val; changeTimer[sw_Ice_Eng_Comb] = millis(); }
  // P17 APU M
  val = (digitalRead(sw_APU_Master) == LOW); if (val != last_APUM) { pan_APUM = val; last_APUM = val; changeTimer[sw_APU_Master] = millis(); }
  // P18 APU S
  val = (digitalRead(sw_APU_Start) == LOW); if (val != last_APUS) { if (val == 1) pan_APUS = 1; last_APUS = val; changeTimer[sw_APU_Start] = millis(); }
  
  // P19 APU B (UPDATE A330)
  val = (digitalRead(sw_APU_Bleed) == LOW); 
  if (val != last_APUB) { 
      pan_APUB = val; 
      pan_APUB_330 = val; // Extra Ref für A330
      last_APUB = val; changeTimer[sw_APU_Bleed] = millis(); 
  }
  
  // P20 XBleed
  val = (digitalRead(sw_XBleed_Open) == LOW); if (val != last_XBleed) { pan_XBleed = val ? 2 : 1; last_XBleed = val; changeTimer[sw_XBleed_Open] = millis(); }
  
  // HYDRAULIK PROTECTION A330
  if (!isA330) {
      // P21 Elec
      val = (digitalRead(sw_ElecPump) == LOW); if (val != last_Elec) { pan_HydElec = val; last_Elec = val; changeTimer[sw_ElecPump] = millis(); }
      // P22 PTU
      val = (digitalRead(sw_PTU_Off) == LOW) ? 0 : 1; if (val != last_PTU) { pan_HydPTU = val; last_PTU = val; changeTimer[sw_PTU_Off] = millis(); }
  } else {
      // Bei A330 passiert hier NICHTS
  }
  
  // P23/24 Packs (PACK LOGIC FIX)
  // LOW = 0 (OFF), HIGH = 1 (ON)
  val = (digitalRead(sw_Pack1) == LOW) ? 0 : 1; 
  if (val != last_Pack1) { pan_Pack1 = val; last_Pack1 = val; changeTimer[sw_Pack1] = millis(); }
  
  val = (digitalRead(sw_Pack2) == LOW) ? 0 : 1; 
  if (val != last_Pack2) { pan_Pack2 = val; last_Pack2 = val; changeTimer[sw_Pack2] = millis(); }
}

void updateModelOutputs() {
  
  // 1. DIMMING CALCULATION
  brightnessScale = 1.0;
  if (serialNightMode) brightnessScale = 0.5;
  if (digitalRead(sw_Dome_Dim) == LOW) brightnessScale = 0.3;

  // 2. SERVO TARGETS
  if (FlightSim.isEnabled() || calMode) {
      if (xGearHandle == 1) { noseTarget = NOSE_POS_DOWN; mainTarget = MAIN_POS_DOWN; } 
      else { noseTarget = NOSE_POS_UP; mainTarget = MAIN_POS_UP; }

      float p = xPitch; float r = xBank;
      if(calMode) { p = 0; r = calTargetBank; } 
      int pitchOffset = (int)(p * 1.5); 
      int bankOffset  = (int)(r * 1.5);

      supFTarget = constrain(MOTION_NEUTRAL + pitchOffset, 0, 100); 
      supLTarget = constrain(MOTION_NEUTRAL - pitchOffset + bankOffset, 0, 100);
      supRTarget = constrain(MOTION_NEUTRAL - pitchOffset - bankOffset, 0, 100);
  }

  updateHydraulics();

  if ((FlightSim.isEnabled() || calMode) && !demoModeActive) { 
      bool isStrobe = (mod_Strobe == 1); bool isBeacon = (mod_Beacon == 1);
      bool isNav = (mod_Nav == 1); bool isLand = (mod_Land == 1); bool isTaxi = (mod_Taxi == 1);

      if (xGearHandle == 1) { 
          if (isLand) analogWrite(pinNoseLight, (int)(VAL_TO * brightnessScale)); 
          else if (isTaxi) analogWrite(pinNoseLight, (int)(VAL_TAXI * brightnessScale)); 
          else analogWrite(pinNoseLight, 0); 
      } else analogWrite(pinNoseLight, 0); 
      
      if (run_eng == 1) {
          int t1=0; if (xEng1N1 > 1) t1 = map((int)(float)xEng1N1, 15, 100, 0, 50); analogWrite(pinEng1, t1);
          int t2=0; if (xEng2N1 > 1) t2 = map((int)(float)xEng2N1, 15, 100, 0, 50); analogWrite(pinEng2, t2);
      } else { analogWrite(pinEng1, 0); analogWrite(pinEng2, 0); }

      if (isNav) analogWrite(pinWingNavs, (int)(40*brightnessScale)); else analogWrite(pinWingNavs, 0);
      if (isLand) analogWrite(pinLanding, (int)(100*brightnessScale)); 
      else if (mod_Rwy > 0.1) analogWrite(pinLanding, (int)(40*brightnessScale)); else analogWrite(pinLanding, 0);

      unsigned long mt = millis() % 1000; 
      if (isBeacon && mt >= 500 && mt < 600) analogWrite(pinBeacon, (int)(255*brightnessScale)); else analogWrite(pinBeacon, 0);
      int strOut = 0; if (isStrobe) { if ((mt < 30) || (mt >= 100 && mt < 130)) strOut = (int)(STROBE_FLASH_VAL * brightnessScale); }
      analogWrite(pinWingStrobes, strOut);
      int tailOut = 0; if (isStrobe && mt < 100) tailOut = (int)(STROBE_FLASH_VAL * brightnessScale);
      if (tailOut > 0) analogWrite(pinTailCombined, tailOut); else if (isNav) analogWrite(pinTailCombined, (int)(NAV_TAIL_DIM * brightnessScale)); else analogWrite(pinTailCombined, 0);
  }
}

void runServoTest() {
    Serial.println("TEST: Neutral");
    noseGear.write(NOSE_POS_DOWN); mainGear.write(MAIN_POS_DOWN);
    supLeft.write(50); supRight.write(50); supFront.write(50);
    delay(1000);
    Serial.println("TEST: Full Motion");
    supLeft.write(100); supRight.write(100); supFront.write(100);
    delay(1000);
    supLeft.write(0); supRight.write(0); supFront.write(0);
    delay(1000);
    Serial.println("TEST: Done");
    noseGear.write(NOSE_POS_DOWN); mainGear.write(MAIN_POS_DOWN);
    supLeft.write(50); supRight.write(50); supFront.write(50);
}

void printRow(String sub, int pin, String name, int switchVal, int lastVal, long simVal) {
    String pS = (switchVal == 0) ? "ON " : "OFF"; 
    bool blocked = (millis() - changeTimer[pin] < BLOCK_TIME);
    String chg = blocked ? " + " : " - "; 
    String sVal = String(simVal); if (name.equals("CALL")) sVal = (switchVal == 0) ? "TRIG" : " - ";
    Serial.print("P"); Serial.print(sub); Serial.print("/T"); Serial.print(pin); Serial.print(" "); Serial.print(name); 
    while(name.length() < 9) { Serial.print(" "); name += " "; } 
    Serial.print("| "); Serial.print(pS); Serial.print(" |  "); Serial.print(chg); Serial.print("  | "); Serial.println(sVal);
}

void printDebugTable() {
    Serial.println("\n--- DEBUG STATUS (v6.8) ---"); 
    Serial.print("PANEL: "); Serial.print(ohpConnected ? "CONN" : "DISC");
    Serial.print(" | TYPE: "); Serial.print(isA330 ? "A330" : "A320");
    Serial.print(" | DIM: "); 
    if (brightnessScale < 0.4) Serial.println("HARD (30%)");
    else if (brightnessScale < 0.9) Serial.println("NIGHT (50%)");
    else Serial.println("DAY (100%)");

    Serial.println("PIN/TEENSY/NAME      | PANEL | BLOCK | SIM (Read)");
    printRow("03", 15, "BEACON", digitalRead(sw_Beacon), last_Beacon, mod_Beacon);
    printRow("04", 16, "STROBE", digitalRead(sw_Strobe_On), last_Strobe, mod_Strobe);
    printRow("05", 17, "NAV",    digitalRead(sw_Nav_Master), last_Nav, mod_Nav);
    Serial.print("P06/07 NOSE (TX/TO)  | "); if (digitalRead(sw_Nose_TO)==LOW) Serial.print("TO "); else if (digitalRead(sw_Nose_Taxi)==HIGH) Serial.print("TAXI"); else Serial.print("OFF ");
    bool nBlocked = (millis() - changeTimer[sw_Nose_TO] < BLOCK_TIME); Serial.print(" |  "); Serial.print(nBlocked ? " + " : " - "); Serial.print("  | "); Serial.println(mod_Taxi);
    printRow("08", 21, "RWY_TRN", digitalRead(sw_RwyTurnoff), last_Rwy, mod_Rwy);
    printRow("09", 22, "LANDING", digitalRead(sw_Landing_Master), last_Land, mod_Land);
    printRow("10", 24, "SEATBLT", digitalRead(sw_Seatbelts), last_Seat, mod_Seat);
    printRow("11", 25, "DOME",    digitalRead(sw_Dome_Dim), last_Dome, mod_Dome);
    printRow("14", 29, "CALL",    digitalRead(sw_Call_Btn), last_Call, 0); 
    printRow("15", 30, "ICE_W",   digitalRead(sw_Ice_Wing), last_IceW, 0);
    printRow("16", 31, "ICE_E",   digitalRead(sw_Ice_Eng_Comb), last_IceE, 0);
    printRow("17", 34, "APU_MAS", digitalRead(sw_APU_Master), last_APUM, 0);
    printRow("18", 35, "APU_STR", digitalRead(sw_APU_Start), last_APUS, mod_APU);
    printRow("19", 36, "APU_BLD", digitalRead(sw_APU_Bleed), last_APUB, 0);
    printRow("20", 37, "X_BLEED", digitalRead(sw_XBleed_Open), last_XBleed, 0);
    printRow("21", 38, "EL_PUMP", digitalRead(sw_ElecPump), last_Elec, 0);
    printRow("22", 39, "PTU",     digitalRead(sw_PTU_Off), last_PTU, 0);
    printRow("23", 40, "PACK1",   digitalRead(sw_Pack1), last_Pack1, 0);
    printRow("24", 32, "PACK2",   digitalRead(sw_Pack2), last_Pack2, 0);
    Serial.println("----------------------------------------------");
    Serial.print("GEAR: "); Serial.print((int)xGearHandle); Serial.print(" | NOSE_TGT: "); Serial.println(noseTarget);
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