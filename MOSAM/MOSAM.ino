#include <Servo.h>
// FlightSim.h ist automatisch aktiv

// =========================================================
//            MOSAM v2.1 - OHP MASTER (Single Land LT)
// =========================================================

// --- SERVO OBJEKTE ---
Servo noseGear;
Servo mainGear;
Servo supLeft;  
Servo supRight; 
Servo supFront; 

// --- OUTPUT PINS (Modell-Steuerung) ---
const int pinNoseLight    = 2;  
const int pinNoseServo    = 3;  
const int pinMainServo    = 4;  
const int pinWingStrobes  = 5;  
const int pinTailCombined = 6;  
const int pinWingNavs     = 7;  
const int pinBeacon       = 8;  
const int pinEng1         = 9;  
const int pinEng2         = 10; 
const int pinLanding      = 11; 

const int pinSupLeft      = 12; 
const int pinSupRight     = 13; 
const int pinSupFront     = 41; 

const int pinProgButton   = 33; 

// --- INPUT PINS (OHP Schalter gegen GND) ---
// Logik: LOW (GND) = AKTIV/ON

// LIGHTS
const int sw_Beacon       = 14; 
const int sw_Strobe_Auto  = 15; // 3-Way: Off - Auto - On
const int sw_Strobe_On    = 16; 
const int sw_Nav_1        = 17; // 3-Way: 1 - Off - 2
const int sw_Nav_2        = 18; 
const int sw_Nose_Taxi    = 19; // 3-Way: Off - Taxi - TO
const int sw_Nose_TO      = 20; 
const int sw_RwyTurnoff   = 21; 

// ÄNDERUNG: Nur noch ein Master-Switch für Landing Lights
const int sw_Landing_Master = 22; 
// Pin 23 ist jetzt FREI (Spare)

const int sw_Seatbelts    = 24; 
const int sw_Dome_Dim     = 25; 
const int sw_Dome_Bright  = 26; 

// SYSTEMS
const int sw_Wiper_Slow   = 27; 
const int sw_Wiper_Fast   = 28; 
const int sw_Call_Btn     = 29; 
const int sw_Ice_Wing     = 30; 
const int sw_Ice_Eng1     = 31; 
const int sw_Ice_Eng2     = 32; 

const int sw_APU_Master   = 34; 
const int sw_APU_Start    = 35; 
const int sw_APU_Bleed    = 36; 
const int sw_XBleed_Open  = 37; 
const int sw_ElecPump     = 38; 
const int sw_PTU_Off      = 39; 
const int sw_Pack1        = 40; 
const int sw_Pack2        = 0;  // Pin 0

// --- VARIABLES & STATES ---
unsigned long lastOHPUpdate = 0;
const int OHP_REFRESH_RATE = 150; 

// Motion Constants
const int NOSE_POS_UP = 40; const int NOSE_POS_DOWN = 5;   
const int MAIN_POS_UP = 65; const int MAIN_POS_DOWN = 5;   
const int SUP_L_RETRACT = 110; const int SUP_L_EXTEND = 0;   
const int SUP_R_RETRACT = 0; const int SUP_R_EXTEND = 110;  
const int SUP_F_RETRACT = 0; const int SUP_F_EXTEND = 110;  
const int MOTION_NEUTRAL = 50; 
const int AIR_LIFT_OFFSET = 15; 
const int REFRESH_RATE = 10;   
const float SPEED_GEAR = 0.05;  
const float SPEED_MOTION = 0.2;  
const int HARDWARE_PWM_LIMIT = 100; 
const int ENG_VISUAL_MAX = 50;  
const int ENG_IDLE_MIN = 15;  
const int ENG_KICK_VAL = 30;  
const int ENG_KICK_TIME = 350;
const float FACTOR_ROLL = 1.5; 
const float FACTOR_PITCH = 2.0;  
const float FACTOR_DOWN_EXTRA = 1.3; 
const float LIMIT_BANK = 40.0; 
const float LIMIT_PITCH_UP = 40.0; 
const float LIMIT_PITCH_DOWN = 30.0; 
const int FIXED_PITCH_OFFSET = -10; 

// Light Constants
const int VAL_OFF = 0; const int VAL_TAXI = 10; const int VAL_TO = 40;
const int STROBE_FLASH_VAL = 50; const int NAV_WING_VAL = 40; 
const int NAV_TAIL_DIM = 10; const int BEACON_VAL = 255; 
const int VAL_LANDING_MAX = 100; const int VAL_RUNWAY_DIM = 40;

// --- DATAREFS (READ) ---
FlightSimInteger xNavLight;
FlightSimInteger xBeaconLight;
FlightSimInteger xStrobeLight;
FlightSimInteger xLandingLight; 
FlightSimInteger xNoseSwitch;    
FlightSimFloat   xRwyTurnSwitch; 
FlightSimInteger xGearHandle; 
FlightSimFloat   xOnGround;   
FlightSimFloat   xBank;       
FlightSimFloat   xPitch;      
FlightSimFloat   xEng1N1;     
FlightSimFloat   xEng2N1;     

// --- DATAREFS (WRITE) ---
FlightSimInteger wBeacon("sim/cockpit2/switches/beacon_on");
FlightSimInteger wStrobe("ckpt/oh/strobeLight/anim"); 
FlightSimInteger wNav("sim/cockpit2/switches/navigation_lights_on"); 
FlightSimInteger wNose("ckpt/oh/taxiLight/anim"); 
FlightSimInteger wLanding("sim/cockpit2/switches/landing_lights_switch");
FlightSimInteger wRwyTurn("ckpt/oh/rwyTurnOff/anim");
FlightSimInteger wSeatbelt("AirbusFBW/SeatBeltSignsOn"); 
FlightSimInteger wDome("sim/cockpit2/switches/generic_lights_switch[0]"); 
FlightSimInteger wWiper("sim/cockpit2/switches/wiper_speed"); 
FlightSimInteger wCall("sim/cockpit2/switches/cabin_call"); 
FlightSimInteger wIceWing("sim/cockpit2/ice/ice_inlet_heat_on_per_engine[0]"); 
FlightSimInteger wIceEng1("sim/cockpit2/ice/ice_inlet_heat_on_per_engine[0]");
FlightSimInteger wIceEng2("sim/cockpit2/ice/ice_inlet_heat_on_per_engine[1]");
FlightSimInteger wAPUMaster("sim/cockpit2/electrical/APU_master_switch");
FlightSimInteger wAPUStart("sim/cockpit2/electrical/APU_starter_switch");
FlightSimInteger wAPUBleed("sim/cockpit2/bleedair/apu_bleed_on");
FlightSimInteger wXBleed("sim/cockpit2/bleedair/cross_tie_open"); 
FlightSimInteger wElecPump("sim/cockpit2/hydraulics/actuators/electric_hydraulic_pump_on");
FlightSimInteger wPTU("sim/cockpit2/hydraulics/actuators/ptu_on");
FlightSimInteger wPack1("sim/cockpit2/bleedair/bleed_air_on[0]");
FlightSimInteger wPack2("sim/cockpit2/bleedair/bleed_air_on[1]");

// --- VARIABLES INTERNAL ---
bool strobesActive = false; bool navsActive = false; bool beaconActive = false;
bool landingActive = false; bool turnoffActive = false; 

// Motion Logic
float noseCurrent = NOSE_POS_DOWN; int noseTarget = NOSE_POS_DOWN;
float mainCurrent = MAIN_POS_DOWN; int mainTarget = MAIN_POS_DOWN;
float supLCurrent = 0.0; int supLTarget = 0;
float supRCurrent = 0.0; int supRTarget = 0;
float supFCurrent = 0.0; int supFTarget = 0;

unsigned long lastMoveTime = 0;    
int eng1TargetPWM = 0; int eng2TargetPWM = 0;
unsigned long eng1KickEnd = 0; unsigned long eng2KickEnd = 0;

int run_eng = 0; 
float brightnessScale = 1.0; 
unsigned long lastStatusTime = 0;
unsigned long lastDebugTime = 0; 

bool calMode = false;         
float calTargetBank = 0.0;    
float calTargetPitch = 0.0;   
int manualRollOffset = 0;     
int manualPitchOffset = 0;   

bool demoHasRun = false; 
bool demoModeActive = false;
int demoNoseLightVal = 0; 

// Forward Declaration
void updateHydraulics();
void updateOHPInputs();
void waitAndAnimate(int waitTime);
void runDemoSequence();

void setup() {
  Serial.begin(9600);
  
  // OUTPUT REFS
  xNavLight      = XPlaneRef("sim/cockpit2/switches/navigation_lights_on");
  xBeaconLight   = XPlaneRef("sim/cockpit2/switches/beacon_on");
  xStrobeLight   = XPlaneRef("sim/cockpit2/switches/strobe_lights_on"); 
  xLandingLight  = XPlaneRef("sim/cockpit2/switches/landing_lights_on");
  xNoseSwitch    = XPlaneRef("ckpt/oh/taxiLight/anim");   
  xRwyTurnSwitch = XPlaneRef("ckpt/oh/rwyTurnOff/anim");  
  xGearHandle    = XPlaneRef("sim/cockpit2/controls/gear_handle_down");
  xOnGround      = XPlaneRef("sim/flightmodel/failures/onground_any"); 
  xBank          = XPlaneRef("sim/flightmodel/position/phi");   
  xPitch         = XPlaneRef("sim/flightmodel/position/true_theta"); 
  xEng1N1        = XPlaneRef("AirbusFBW/fmod/eng/N1Array[0]");
  xEng2N1        = XPlaneRef("AirbusFBW/fmod/eng/N1Array[1]");

  // PINS MODES
  pinMode(pinNoseLight, OUTPUT); pinMode(pinWingStrobes, OUTPUT);
  pinMode(pinTailCombined, OUTPUT); pinMode(pinWingNavs, OUTPUT);
  pinMode(pinBeacon, OUTPUT); pinMode(pinEng1, OUTPUT);
  pinMode(pinEng2, OUTPUT); pinMode(pinLanding, OUTPUT);
  pinMode(pinProgButton, INPUT_PULLUP); 

  // INPUTS (OHP) PULLUP
  for (int i=14; i<=40; i++) { pinMode(i, INPUT_PULLUP); }
  pinMode(sw_Pack2, INPUT_PULLUP); // Pin 0

  // INIT
  analogWrite(pinNoseLight, VAL_OFF); 
  analogWrite(pinWingStrobes, 0); analogWrite(pinWingNavs, 0); 
  analogWrite(pinTailCombined, 0); analogWrite(pinBeacon, 0); 
  analogWrite(pinEng1, 0); analogWrite(pinEng2, 0); 
  analogWrite(pinLanding, 0);

  // SERVO
  noseGear.attach(pinNoseServo); delay(500);
  mainGear.attach(pinMainServo); delay(500);
  noseGear.write((int)noseCurrent); mainGear.write((int)mainCurrent); delay(200);
  supLeft.attach(pinSupLeft);   supLeft.write(SUP_L_RETRACT);   delay(200);
  supRight.attach(pinSupRight); supRight.write(SUP_R_RETRACT);  delay(200);
  supFront.attach(pinSupFront); supFront.write(SUP_F_RETRACT);  delay(200);

  Serial.println("--- MOSAM v2.1 OHP MASTER (SINGLE LAND) READY ---");
  Serial.println("INFO: Press Button within 30s for DEMO, later for PROG MODE.");
}

void loop() {
  FlightSim.update(); 
  
  if (digitalRead(pinProgButton) == LOW) {
    delay(50); 
    if (digitalRead(pinProgButton) == LOW) {
       if (millis() < 30000 && !demoHasRun) {
          runDemoSequence();
       } else {
          analogWrite(pinEng1, 0); analogWrite(pinEng2, 0); 
          Serial.println("!!! ENTERING PROGRAM MODE !!!");
          delay(100);
          asm("bkpt #251"); 
       }
    }
  }

  unsigned long now = millis();
  if (now - lastOHPUpdate >= OHP_REFRESH_RATE && !demoModeActive && !calMode) {
     lastOHPUpdate = now;
     updateOHPInputs();
  }

  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "engrun") { run_eng = 1; Serial.println(">> ENGINES ENABLED"); } 
    else if (cmd == "engstop") { run_eng = 0; Serial.println(">> ENGINES STOPPED"); }
    else if (cmd == "night") { brightnessScale = 0.3; Serial.println(">> MODE: NIGHT (30%)"); }
    else if (cmd == "day") { brightnessScale = 1.0; Serial.println(">> MODE: DAY (100%)"); }
    
    // Calibration
    else if (cmd.startsWith("cal_left")) {
      calMode = true; manualRollOffset = cmd.substring(9).toInt(); manualPitchOffset = 0; 
      calTargetBank = -LIMIT_BANK; calTargetPitch = 0.0; 
      Serial.print(">> CAL: LEFT MAX | Off: "); Serial.println(manualRollOffset);
    }
    else if (cmd.startsWith("cal_right")) {
      calMode = true; manualRollOffset = cmd.substring(10).toInt(); manualPitchOffset = 0;
      calTargetBank = LIMIT_BANK; calTargetPitch = 0.0;
      Serial.print(">> CAL: RIGHT MAX | Off: "); Serial.println(manualRollOffset);
    }
    else if (cmd.startsWith("cal_up")) {
      calMode = true; manualPitchOffset = cmd.substring(7).toInt(); manualRollOffset = 0;
      calTargetBank = 0.0; calTargetPitch = LIMIT_PITCH_UP; 
      Serial.print(">> CAL: PITCH UP MAX | Off: "); Serial.println(manualPitchOffset);
    }
    else if (cmd.startsWith("cal_down")) {
      calMode = true; manualPitchOffset = cmd.substring(9).toInt(); manualRollOffset = 0;
      calTargetBank = 0.0; calTargetPitch = -LIMIT_PITCH_DOWN; 
      Serial.print(">> CAL: PITCH DOWN MAX | Off: "); Serial.println(manualPitchOffset);
    }
    else if (cmd == "cal_off") {
      calMode = false; manualRollOffset = 0; manualPitchOffset = 0; Serial.println(">> CAL: OFF");
    }
  }

  int noseSwitchPos = 0; 

  if ((FlightSim.isEnabled() || calMode) && !demoModeActive) { 
    if (!calMode) {
        navsActive    = xNavLight;
        beaconActive  = xBeaconLight;
        
        long strbVal = xStrobeLight; 
        if (strbVal > 0) strobesActive = true; else strobesActive = false;

        landingActive = xLandingLight;  
        if (xRwyTurnSwitch > 0.1) turnoffActive = true; else turnoffActive = false;
        noseSwitchPos = xNoseSwitch;
    }

    if (xGearHandle == 1) { 
      if (noseSwitchPos == 2)      analogWrite(pinNoseLight, (int)(VAL_TO * brightnessScale));   
      else if (noseSwitchPos == 1) analogWrite(pinNoseLight, (int)(VAL_TAXI * brightnessScale)); 
      else                         analogWrite(pinNoseLight, VAL_OFF);
    } else analogWrite(pinNoseLight, VAL_OFF); 

    // Engines
    float n1_1 = 0; float n1_2 = 0;
    if (run_eng == 1 && !calMode) { n1_1 = xEng1N1; n1_2 = xEng2N1; }
    
    int targetP1 = 0;
    if (n1_1 > 1.0) { 
      targetP1 = map((int)n1_1, 15, 100, 0, ENG_VISUAL_MAX);
      if (targetP1 < ENG_IDLE_MIN) targetP1 = ENG_IDLE_MIN;
    }
    if (eng1TargetPWM == 0 && targetP1 > ENG_IDLE_MIN) { 
      analogWrite(pinEng1, map(ENG_KICK_VAL, 0, 100, 0, HARDWARE_PWM_LIMIT));
      eng1KickEnd = millis() + ENG_KICK_TIME;
    } else analogWrite(pinEng1, targetP1);
    eng1TargetPWM = targetP1;

    int targetP2 = 0;
    if (n1_2 > 1.0) {
      targetP2 = map((int)n1_2, 15, 100, 0, ENG_VISUAL_MAX);
      if (targetP2 < ENG_IDLE_MIN) targetP2 = ENG_IDLE_MIN;
    }
    if (eng2TargetPWM == 0 && targetP2 > ENG_IDLE_MIN) {
      analogWrite(pinEng2, map(ENG_KICK_VAL, 0, 100, 0, HARDWARE_PWM_LIMIT));
      eng2KickEnd = millis() + ENG_KICK_TIME;
    } else analogWrite(pinEng2, targetP2);
    eng2TargetPWM = targetP2;

    if (xGearHandle == 1) { noseTarget = NOSE_POS_DOWN; mainTarget = MAIN_POS_DOWN; } 
    else { noseTarget = NOSE_POS_UP; mainTarget = MAIN_POS_UP; }

    // Motion
    float rollDeg, pitchDeg;
    bool isOnGround;

    if (calMode) {
        rollDeg = calTargetBank;
        pitchDeg = calTargetPitch;
        isOnGround = false; 
    } else {
        rollDeg = xBank * FACTOR_ROLL;
        pitchDeg = xPitch * FACTOR_PITCH;
        if (pitchDeg < 0) pitchDeg *= FACTOR_DOWN_EXTRA; 
        isOnGround = xOnGround;
    }
    
    float rollClamped = constrain(rollDeg, -LIMIT_BANK, LIMIT_BANK);
    float pitchClamped;
    
    if (pitchDeg >= 0) pitchClamped = constrain(pitchDeg, 0, LIMIT_PITCH_UP);
    else pitchClamped = constrain(pitchDeg, -LIMIT_PITCH_DOWN, 0);

    int currentPitchOffset = FIXED_PITCH_OFFSET;
    if (calMode) currentPitchOffset = manualPitchOffset;

    if (isOnGround) {
      int targetFront = map((long)(pitchDeg * 10), 0, (long)(LIMIT_PITCH_UP*10), 0, 100);
      targetFront = constrain(targetFront, 0, 100);
      supLTarget = 0; supRTarget = 0; supFTarget = targetFront;
    } else {
      int baseLevel = MOTION_NEUTRAL + AIR_LIFT_OFFSET;
      int rollMovement = map((long)(abs(rollClamped)*10), 0, (long)(LIMIT_BANK*10), 0, 50);
      if (rollClamped < 0) rollMovement = -rollMovement; 
      
      int pitchMovement = 0;
      if (pitchClamped >= 0) {
          pitchMovement = map((long)(pitchClamped*10), 0, (long)(LIMIT_PITCH_UP*10), 0, 50);
      } else {
          pitchMovement = map((long)(abs(pitchClamped)*10), 0, (long)(LIMIT_PITCH_DOWN*10), 0, 50);
          pitchMovement = -pitchMovement; 
      }

      int combinedPitchRear = pitchMovement + currentPitchOffset;
      int combinedPitchFront = pitchMovement + currentPitchOffset;
      int combinedRoll = rollMovement + manualRollOffset;

      int targetL = baseLevel + combinedRoll - combinedPitchRear;
      int targetR = baseLevel - combinedRoll - combinedPitchRear;
      int targetF = baseLevel + combinedPitchFront;

      supLTarget = constrain(targetL, 0, 100);
      supRTarget = constrain(targetR, 0, 100);
      supFTarget = constrain(targetF, 0, 100);
    }
    
    unsigned long nowDebug = millis();
    if (nowDebug - lastDebugTime > 1000) {
        lastDebugTime = nowDebug;
        if (!calMode && FlightSim.isEnabled()) {
            Serial.print("SYS: P="); Serial.print((int)xPitch);
            Serial.print(" R="); Serial.print((int)xBank);
            Serial.print(" | Motion F: "); Serial.println(supFTarget);
        }
    }
  }

  if (!demoModeActive) {
     updateHydraulics();

     unsigned long nowEngine = millis();
     if (eng1KickEnd > 0 && nowEngine > eng1KickEnd) { analogWrite(pinEng1, eng1TargetPWM); eng1KickEnd = 0; }
     if (eng2KickEnd > 0 && nowEngine > eng2KickEnd) { analogWrite(pinEng2, eng2TargetPWM); eng2KickEnd = 0; }
  
     if (navsActive) analogWrite(pinWingNavs, (int)(NAV_WING_VAL * brightnessScale)); 
     else analogWrite(pinWingNavs, 0);
     
     unsigned long masterTimer = millis() % 1000; 
     int wingStrobeOut = 0;
     if (strobesActive) {
       if ((masterTimer < 30) || (masterTimer >= 100 && masterTimer < 130)) wingStrobeOut = (int)(STROBE_FLASH_VAL * brightnessScale);
     }
     analogWrite(pinWingStrobes, wingStrobeOut);
     
     int tailStrobeOut = 0;
     if (strobesActive && masterTimer < 100) tailStrobeOut = (int)(STROBE_FLASH_VAL * brightnessScale);
     
     if (tailStrobeOut > 0) analogWrite(pinTailCombined, tailStrobeOut);
     else if (navsActive) analogWrite(pinTailCombined, (int)(NAV_TAIL_DIM * brightnessScale));
     else analogWrite(pinTailCombined, 0);
   
     if (beaconActive && masterTimer >= 500 && masterTimer < 600) analogWrite(pinBeacon, (int)(BEACON_VAL * brightnessScale));
     else analogWrite(pinBeacon, 0);
   
     if (landingActive) analogWrite(pinLanding, (int)(VAL_LANDING_MAX * brightnessScale));
     else if (turnoffActive) analogWrite(pinLanding, (int)(VAL_RUNWAY_DIM * brightnessScale));
     else analogWrite(pinLanding, 0);
  }
}

// =========================================================
// OHP INPUT LOGIC
// =========================================================
void updateOHPInputs() {
  
  if (digitalRead(sw_Beacon) == LOW) wBeacon = 1; else wBeacon = 0;

  if (digitalRead(sw_Strobe_On) == LOW) wStrobe = 2;
  else if (digitalRead(sw_Strobe_Auto) == LOW) wStrobe = 1;
  else wStrobe = 0; 

  if (digitalRead(sw_Nav_1) == LOW || digitalRead(sw_Nav_2) == LOW) wNav = 1; else wNav = 0;

  if (digitalRead(sw_Nose_TO) == LOW) wNose = 2;
  else if (digitalRead(sw_Nose_Taxi) == LOW) wNose = 1;
  else wNose = 0;

  // ÄNDERUNG: SINGLE MASTER SWITCH LANDING
  if (digitalRead(sw_Landing_Master) == LOW) wLanding = 1; else wLanding = 0;

  if (digitalRead(sw_RwyTurnoff) == LOW) wRwyTurn = 1; else wRwyTurn = 0;
  if (digitalRead(sw_Seatbelts) == LOW) wSeatbelt = 1; else wSeatbelt = 0;
  if (digitalRead(sw_Dome_Bright) == LOW) wDome = 1; else wDome = 0;

  if (digitalRead(sw_Wiper_Fast) == LOW) wWiper = 3; 
  else if (digitalRead(sw_Wiper_Slow) == LOW) wWiper = 2; 
  else wWiper = 0;

  if (digitalRead(sw_Call_Btn) == LOW) wCall = 1; else wCall = 0;
  if (digitalRead(sw_Ice_Wing) == LOW) wIceWing = 1; else wIceWing = 0;
  if (digitalRead(sw_Ice_Eng1) == LOW) wIceEng1 = 1; else wIceEng1 = 0;
  if (digitalRead(sw_Ice_Eng2) == LOW) wIceEng2 = 1; else wIceEng2 = 0;
  if (digitalRead(sw_APU_Master) == LOW) wAPUMaster = 1; else wAPUMaster = 0;
  if (digitalRead(sw_APU_Start) == LOW) wAPUStart = 1; else wAPUStart = 0;
  if (digitalRead(sw_APU_Bleed) == LOW) wAPUBleed = 1; else wAPUBleed = 0;
  
  if (digitalRead(sw_XBleed_Open) == LOW) wXBleed = 1; else wXBleed = 2; // Auto
  if (digitalRead(sw_ElecPump) == LOW) wElecPump = 1; else wElecPump = 0;
  if (digitalRead(sw_PTU_Off) == LOW) wPTU = 0; else wPTU = 1; // Auto default
  if (digitalRead(sw_Pack1) == LOW) wPack1 = 1; else wPack1 = 0;
  if (digitalRead(sw_Pack2) == LOW) wPack2 = 1; else wPack2 = 0;
}

void updateHydraulics() {
    unsigned long now = millis();
    if (now - lastMoveTime >= REFRESH_RATE) {
      lastMoveTime = now;
      if (abs(noseCurrent - noseTarget) > SPEED_GEAR) {
         if (noseCurrent < noseTarget) noseCurrent += SPEED_GEAR; else noseCurrent -= SPEED_GEAR;
      } else noseCurrent = noseTarget; 
      noseGear.write((int)noseCurrent);
      if (abs(mainCurrent - mainTarget) > SPEED_GEAR) {
         if (mainCurrent < mainTarget) mainCurrent += SPEED_GEAR; else mainCurrent -= SPEED_GEAR;
      } else mainCurrent = mainTarget;
      mainGear.write((int)mainCurrent);
      if (abs(supLCurrent - supLTarget) > SPEED_MOTION) {
         if (supLCurrent < supLTarget) supLCurrent += SPEED_MOTION; else supLCurrent -= SPEED_MOTION;
      } else supLCurrent = supLTarget;
      supLeft.write(map((int)supLCurrent, 0, 100, SUP_L_RETRACT, SUP_L_EXTEND));
      if (abs(supRCurrent - supRTarget) > SPEED_MOTION) {
         if (supRCurrent < supRTarget) supRCurrent += SPEED_MOTION; else supRCurrent -= SPEED_MOTION;
      } else supRCurrent = supRTarget;
      supRight.write(map((int)supRCurrent, 0, 100, SUP_R_RETRACT, SUP_R_EXTEND));
      if (abs(supFCurrent - supFTarget) > SPEED_MOTION) {
         if (supFCurrent < supFTarget) supFCurrent += SPEED_MOTION; else supFCurrent -= SPEED_MOTION;
      } else supFCurrent = supFTarget;
      supFront.write(map((int)supFCurrent, 0, 100, SUP_F_RETRACT, SUP_F_EXTEND));
    }
}

void runDemoSequence() {
  Serial.println(">> DEMO START");
  demoModeActive = true; demoHasRun = true;
  navsActive=0; beaconActive=0; strobesActive=0; landingActive=0; turnoffActive=0;
  demoNoseLightVal = VAL_OFF; run_eng = 0; eng1TargetPWM = 0; eng2TargetPWM = 0;
  noseTarget = NOSE_POS_DOWN; mainTarget = MAIN_POS_DOWN; supLTarget=0; supRTarget=0; supFTarget=0; 
  
  navsActive = true; waitAndAnimate(2000);
  beaconActive = true; waitAndAnimate(2000);
  run_eng = 1; eng1TargetPWM = ENG_IDLE_MIN; waitAndAnimate(3000); 
  eng2TargetPWM = ENG_IDLE_MIN; waitAndAnimate(3000); 
  demoNoseLightVal = VAL_TAXI; turnoffActive = true; waitAndAnimate(2000);
  strobesActive = true; waitAndAnimate(2000);
  demoNoseLightVal = VAL_TO; landingActive = true; waitAndAnimate(2000);
  eng1TargetPWM = 40; eng2TargetPWM = 40; waitAndAnimate(2000);
  supFTarget = 70; supLTarget = 0; supRTarget = 0; waitAndAnimate(2000);
  
  int base = MOTION_NEUTRAL + AIR_LIFT_OFFSET;
  int pitchMove = 35; 
  supFTarget = constrain(base + pitchMove, 0, 100);
  supLTarget = constrain(base - pitchMove, 0, 100);
  supRTarget = constrain(base - pitchMove, 0, 100); waitAndAnimate(4000); 

  noseTarget = NOSE_POS_UP; mainTarget = MAIN_POS_UP; waitAndAnimate(4000); 
  pitchMove = 10; int rollMove = -35; 
  supFTarget = constrain(base + pitchMove, 0, 100);
  supLTarget = constrain(base + rollMove - pitchMove, 0, 100);
  supRTarget = constrain(base - rollMove - pitchMove, 0, 100); waitAndAnimate(5000); 
  landingActive = false; turnoffActive = false; demoNoseLightVal = VAL_OFF; waitAndAnimate(2000);
  supFTarget = base; supLTarget = base; supRTarget = base; waitAndAnimate(4000); 
  rollMove = 35; 
  supFTarget = constrain(base, 0, 100);
  supLTarget = constrain(base + rollMove, 0, 100);
  supRTarget = constrain(base - rollMove, 0, 100); waitAndAnimate(5000); 
  pitchMove = -10; 
  supFTarget = constrain(base + pitchMove, 0, 100);
  supLTarget = constrain(base - pitchMove, 0, 100); 
  supRTarget = constrain(base - pitchMove, 0, 100);
  noseTarget = NOSE_POS_DOWN; mainTarget = MAIN_POS_DOWN;
  eng1TargetPWM = ENG_IDLE_MIN; eng2TargetPWM = ENG_IDLE_MIN; waitAndAnimate(5000); 
  landingActive = true; demoNoseLightVal = VAL_TO; waitAndAnimate(2000);
  pitchMove = 15; 
  supFTarget = constrain(base + pitchMove, 0, 100);
  supLTarget = constrain(base - pitchMove, 0, 100);
  supRTarget = constrain(base - pitchMove, 0, 100); waitAndAnimate(3000);
  supLTarget = 0; supRTarget = 0; supFTarget = 0; waitAndAnimate(1000); waitAndAnimate(2000);
  strobesActive = false; landingActive = false; waitAndAnimate(2000);
  demoNoseLightVal = VAL_OFF; waitAndAnimate(2000);
  eng1TargetPWM = 0; eng2TargetPWM = 0; waitAndAnimate(2000);
  beaconActive = false; waitAndAnimate(2000);
  navsActive = false; waitAndAnimate(2000);
  Serial.println(">> DEMO END"); demoModeActive = false;
}

void waitAndAnimate(int waitTime) {
  unsigned long start = millis();
  while (millis() - start < (unsigned long)waitTime) {
    updateHydraulics(); 
    analogWrite(pinEng1, eng1TargetPWM); analogWrite(pinEng2, eng2TargetPWM);
    if (navsActive) analogWrite(pinWingNavs, (int)(NAV_WING_VAL * brightnessScale)); else analogWrite(pinWingNavs, 0);
    unsigned long masterTimer = millis() % 1000; int wingStrobeOut = 0;
    if (strobesActive) { if ((masterTimer < 30) || (masterTimer >= 100 && masterTimer < 130)) wingStrobeOut = (int)(STROBE_FLASH_VAL * brightnessScale); }
    analogWrite(pinWingStrobes, wingStrobeOut);
    int tailStrobeOut = 0; if (strobesActive && masterTimer < 100) tailStrobeOut = (int)(STROBE_FLASH_VAL * brightnessScale);
    if (tailStrobeOut > 0) analogWrite(pinTailCombined, tailStrobeOut); else if (navsActive) analogWrite(pinTailCombined, (int)(NAV_TAIL_DIM * brightnessScale)); else analogWrite(pinTailCombined, 0);
    if (beaconActive && masterTimer >= 500 && masterTimer < 600) analogWrite(pinBeacon, (int)(BEACON_VAL * brightnessScale)); else analogWrite(pinBeacon, 0);
    if (landingActive) analogWrite(pinLanding, (int)(VAL_LANDING_MAX * brightnessScale)); else if (turnoffActive) analogWrite(pinLanding, (int)(VAL_RUNWAY_DIM * brightnessScale)); else analogWrite(pinLanding, 0);
    if (xGearHandle == 1 || demoModeActive) { 
       if (noseTarget == NOSE_POS_DOWN) analogWrite(pinNoseLight, (int)(demoNoseLightVal * brightnessScale));
       else analogWrite(pinNoseLight, 0);
    } else analogWrite(pinNoseLight, 0);
    delay(REFRESH_RATE);
  }
}