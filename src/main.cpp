#include <Arduino.h>
#include <Servo.h>

// =========================================================
//      MOSAM v2.4 - OHP MASTER (Advanced Status Report)
// =========================================================

// --- SERVO OBJEKTE ---
Servo noseGear;
Servo mainGear;
Servo supLeft;  
Servo supRight; 
Servo supFront; 

// --- OUTPUT PINS ---
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

// --- INPUT PINS ---
const int sw_Beacon       = 14; 
const int sw_Strobe_Auto  = 15; 
const int sw_Strobe_On    = 16; 
const int sw_Nav_1        = 17; 
const int sw_Nav_2        = 18; 
const int sw_Nose_Taxi    = 19; 
const int sw_Nose_TO      = 20; 
const int sw_RwyTurnoff   = 21; 
const int sw_Landing_Master = 22; 

const int pinOHP_Detect   = 1; // Pin 1 ist Detect

const int sw_Seatbelts    = 24; 
const int sw_Dome_Dim     = 25; 
const int sw_Dome_Bright  = 26; 
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
const int sw_Pack2        = 0;

// --- VARIABLES ---
unsigned long lastOHPUpdate = 0;
const int OHP_REFRESH_RATE = 150; 
bool ohpConnected = false; 

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

// --- DATAREFS DECLARATION ---
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

// Write Refs
FlightSimInteger wBeacon;
FlightSimInteger wStrobe; 
FlightSimInteger wNav; 
FlightSimInteger wNose; 
FlightSimInteger wLanding;
FlightSimInteger wRwyTurn;
FlightSimInteger wSeatbelt; 
FlightSimInteger wDome; 
FlightSimInteger wWiper; 
FlightSimInteger wCall; 
FlightSimInteger wIceWing; 
FlightSimInteger wIceEng1;
FlightSimInteger wIceEng2;
FlightSimInteger wAPUMaster;
FlightSimInteger wAPUStart;
FlightSimInteger wAPUBleed;
FlightSimInteger wXBleed; 
FlightSimInteger wElecPump;
FlightSimInteger wPTU;
FlightSimInteger wPack1;
FlightSimInteger wPack2;

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
  
  // --- DATAREF ASSIGNMENT ---
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

  wBeacon = XPlaneRef("sim/cockpit2/switches/beacon_on");
  wStrobe = XPlaneRef("ckpt/oh/strobeLight/anim"); 
  wNav = XPlaneRef("sim/cockpit2/switches/navigation_lights_on"); 
  wNose = XPlaneRef("ckpt/oh/taxiLight/anim"); 
  wLanding = XPlaneRef("sim/cockpit2/switches/landing_lights_switch");
  wRwyTurn = XPlaneRef("ckpt/oh/rwyTurnOff/anim");
  wSeatbelt = XPlaneRef("AirbusFBW/SeatBeltSignsOn"); 
  wDome = XPlaneRef("sim/cockpit2/switches/generic_lights_switch[0]"); 
  wWiper = XPlaneRef("sim/cockpit2/switches/wiper_speed"); 
  wCall = XPlaneRef("sim/cockpit2/switches/cabin_call"); 
  wIceWing = XPlaneRef("sim/cockpit2/ice/ice_inlet_heat_on_per_engine[0]"); 
  wIceEng1 = XPlaneRef("sim/cockpit2/ice/ice_inlet_heat_on_per_engine[0]");
  wIceEng2 = XPlaneRef("sim/cockpit2/ice/ice_inlet_heat_on_per_engine[1]");
  wAPUMaster = XPlaneRef("sim/cockpit2/electrical/APU_master_switch");
  wAPUStart = XPlaneRef("sim/cockpit2/electrical/APU_starter_switch");
  wAPUBleed = XPlaneRef("sim/cockpit2/bleedair/apu_bleed_on");
  wXBleed = XPlaneRef("sim/cockpit2/bleedair/cross_tie_open"); 
  wElecPump = XPlaneRef("sim/cockpit2/hydraulics/actuators/electric_hydraulic_pump_on");
  wPTU = XPlaneRef("sim/cockpit2/hydraulics/actuators/ptu_on");
  wPack1 = XPlaneRef("sim/cockpit2/bleedair/bleed_air_on[0]");
  wPack2 = XPlaneRef("sim/cockpit2/bleedair/bleed_air_on[1]");

  // PINS MODES OUTPUT
  pinMode(pinNoseLight, OUTPUT); pinMode(pinWingStrobes, OUTPUT);
  pinMode(pinTailCombined, OUTPUT); pinMode(pinWingNavs, OUTPUT);
  pinMode(pinBeacon, OUTPUT); pinMode(pinEng1, OUTPUT);
  pinMode(pinEng2, OUTPUT); pinMode(pinLanding, OUTPUT);
  pinMode(pinProgButton, INPUT_PULLUP); 

  // INPUTS (OHP) PULLUP
  for (int i=14; i<=40; i++) { pinMode(i, INPUT_PULLUP); }
  pinMode(sw_Pack2, INPUT_PULLUP); // Pin 0

  // OHP DETECT PIN
  pinMode(pinOHP_Detect, INPUT_PULLUP);

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

  Serial.println("--- MOSAM v2.4 STATUS REPORT READY ---");
  Serial.println("INFO: Type 'help' for commands.");
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

  // --- OHP AUTO DETECT ---
  unsigned long now = millis();
  if (!demoModeActive && !calMode) {
      if (digitalRead(pinOHP_Detect) == LOW) {
          ohpConnected = true;
          if (now - lastOHPUpdate >= OHP_REFRESH_RATE) {
              lastOHPUpdate = now;
              updateOHPInputs();
          }
      } else {
          ohpConnected = false; 
      }
  }

  // --- SERIAL COMMANDS ---
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd == "help" || cmd == "?") {
        Serial.println("\n=== MOSAM v2.4 COMMAND LIST ===");
        Serial.println("  status        : Show full system report (Inputs, SimData, States)");
        Serial.println("  engrun / stop : Control Engine PWM");
        Serial.println("  day / night   : Control Brightness");
        Serial.println("  cal_[dir] [v] : Calibration (left, right, up, down)");
        Serial.println("===============================\n");
    }
    else if (cmd == "engrun") { run_eng = 1; Serial.println(">> ENGINES ENABLED"); } 
    else if (cmd == "engstop") { run_eng = 0; Serial.println(">> ENGINES STOPPED"); }
    else if (cmd == "night") { brightnessScale = 0.3; Serial.println(">> MODE: NIGHT (30%)"); }
    else if (cmd == "day") { brightnessScale = 1.0; Serial.println(">> MODE: DAY (100%)"); }
    
    // --- STATUS REPORT UPDATE ---
    else if (cmd == "status") {
        Serial.println("\n=== MOSAM v2.4 SYSTEM REPORT ===");

        // 1. HARDWARE INPUTS
        Serial.println("[OHP RAW INPUTS] (0=ON/GND, 1=OFF/OPEN)");
        Serial.print("  DETECT(P1):"); Serial.print(digitalRead(pinOHP_Detect)); 
        Serial.print(" | OHP_LOGIC:"); Serial.println(ohpConnected ? "ACTIVE" : "IGNORED");
        
        Serial.print("  LIGHTS: Bcn:"); Serial.print(digitalRead(sw_Beacon));
        Serial.print(" Strb(A/O):"); Serial.print(digitalRead(sw_Strobe_Auto)); Serial.print("/"); Serial.print(digitalRead(sw_Strobe_On));
        Serial.print(" Nav(1/2):"); Serial.print(digitalRead(sw_Nav_1)); Serial.print("/"); Serial.print(digitalRead(sw_Nav_2));
        Serial.print(" Nose(T/TO):"); Serial.print(digitalRead(sw_Nose_Taxi)); Serial.print("/"); Serial.print(digitalRead(sw_Nose_TO));
        Serial.print(" Land:"); Serial.print(digitalRead(sw_Landing_Master));
        Serial.print(" Rwy:"); Serial.print(digitalRead(sw_RwyTurnoff));
        Serial.print(" Dome(D/B):"); Serial.print(digitalRead(sw_Dome_Dim)); Serial.print("/"); Serial.println(digitalRead(sw_Dome_Bright));
        
        Serial.print("  SYSTEM: APU(M/S/B):"); Serial.print(digitalRead(sw_APU_Master)); Serial.print("/"); Serial.print(digitalRead(sw_APU_Start)); Serial.print("/"); Serial.print(digitalRead(sw_APU_Bleed));
        Serial.print(" Ice(W/E1/E2):"); Serial.print(digitalRead(sw_Ice_Wing)); Serial.print("/"); Serial.print(digitalRead(sw_Ice_Eng1)); Serial.print("/"); Serial.print(digitalRead(sw_Ice_Eng2));
        Serial.print(" Wiper(S/F):"); Serial.print(digitalRead(sw_Wiper_Slow)); Serial.print("/"); Serial.println(digitalRead(sw_Wiper_Fast));

        Serial.print("  HYD/AIR: Elec:"); Serial.print(digitalRead(sw_ElecPump));
        Serial.print(" PTU:"); Serial.print(digitalRead(sw_PTU_Off)); 
        Serial.print(" XBleed:"); Serial.print(digitalRead(sw_XBleed_Open)); 
        Serial.print(" Packs:"); Serial.print(digitalRead(sw_Pack1)); Serial.print("/"); Serial.println(digitalRead(sw_Pack2));

        // 2. SIMULATOR VALUES
        Serial.println("[SIMULATOR DATA]");
        if (FlightSim.isEnabled()) {
            Serial.print("  PITCH:"); Serial.print((float)xPitch, 1);
            Serial.print(" | BANK:"); Serial.print((float)xBank, 1);
            Serial.print(" | GROUND:"); Serial.println((int)xOnGround);
            Serial.print("  GEAR:"); Serial.print((int)xGearHandle);
            Serial.print(" | N1:"); Serial.print((float)xEng1N1, 0); Serial.print("/"); Serial.println((float)xEng2N1, 0);
            Serial.print("  LTS(Read): Strobe="); Serial.print((int)xStrobeLight);
            Serial.print(" Nav="); Serial.print((int)xNavLight);
            Serial.print(" Bcn="); Serial.println((int)xBeaconLight);
        } else {
            Serial.println("  >> X-PLANE NOT RUNNING / NO CONNECTION <<");
        }

        // 3. INTERNAL STATES
        Serial.println("[INTERNAL MODEL STATE]");
        Serial.print("  Motion Targets (F/L/R): "); Serial.print(supFTarget); Serial.print(" / "); Serial.print(supLTarget); Serial.print(" / "); Serial.println(supRTarget);
        Serial.print("  Servos Current (Nose/Main): "); Serial.print(noseCurrent); Serial.print(" / "); Serial.println(mainCurrent);
        Serial.print("  Flags: StrobeActive="); Serial.print(strobesActive);
        Serial.print(" | DemoMode="); Serial.println(demoModeActive);
        Serial.println("===============================");
    }
    
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
            Serial.print(" | Motion F: "); Serial.print(supFTarget);
            Serial.print(" | OHP: "); Serial.println(ohpConnected ? "ON" : "OFF");
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