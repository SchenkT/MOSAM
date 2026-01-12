#include <Servo.h>
// FlightSim.h ist automatisch aktiv

Servo noseGear;
Servo mainGear;
Servo supLeft;  
Servo supRight; 
Servo supFront; 

// --- PINS ---
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

// --- KALIBRIERUNG: GEOMETRIE ---
const int NOSE_POS_UP   = 40;  const int NOSE_POS_DOWN = 5;   
const int MAIN_POS_UP   = 65;  const int MAIN_POS_DOWN = 5;   

const int SUP_L_RETRACT = 110; const int SUP_L_EXTEND  = 0;   
const int SUP_R_RETRACT = 0;   const int SUP_R_EXTEND  = 110;  
const int SUP_F_RETRACT = 0;   const int SUP_F_EXTEND  = 110;  

// --- MOTION DEFINITIONEN ---
const int MOTION_NEUTRAL = 50; 
const int AIR_LIFT_OFFSET = 15; 

// --- TIMING & SPEED ---
const int REFRESH_RATE    = 10;   
const float SPEED_GEAR    = 0.05;  
const float SPEED_MOTION  = 0.2;  

// --- KALIBRIERUNG: ENGINES ---
const int HARDWARE_PWM_LIMIT = 100; 
const int ENG_VISUAL_MAX = 50;  
const int ENG_IDLE_MIN   = 15;  
const int ENG_KICK_VAL   = 30;  
const int ENG_KICK_TIME  = 350;

// --- KALIBRIERUNG: MOTION ---
const float FACTOR_ROLL        = 1.5; 
const float FACTOR_PITCH       = 2.0;  
const float FACTOR_DOWN_EXTRA  = 1.3; 

const float LIMIT_BANK       = 40.0; 
const float LIMIT_PITCH_UP   = 40.0; 
const float LIMIT_PITCH_DOWN = 30.0; 

const int FIXED_PITCH_OFFSET = -10; 

// --- LIGHT VALUES ---
const int VAL_OFF = 0; const int VAL_TAXI = 10; const int VAL_TO = 40;
const int STROBE_FLASH_VAL = 50; const int NAV_WING_VAL = 40; 
const int NAV_TAIL_DIM = 10; const int BEACON_VAL = 255; 
const int VAL_LANDING_MAX = 100; const int VAL_RUNWAY_DIM = 40;

// --- DATAREFS ---
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

// --- VARIABLES ---
bool strobesActive = false; bool navsActive = false; bool beaconActive = false;
bool landingActive = false; bool turnoffActive = false; 

// Positionen
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

// Kalibrierung
bool calMode = false;         
float calTargetBank = 0.0;    
float calTargetPitch = 0.0;   
int manualRollOffset = 0;     
int manualPitchOffset = 0;   

// Demo Status
bool demoHasRun = false; 
bool demoModeActive = false;
int demoNoseLightVal = 0; 

// Change Detection
bool prevNav=0, prevBcn=0, prevStrobe=0, prevLand=0, prevTurn=0;
int prevGear=-1, prevNoseSw=-1;

// Forward Declaration
void updateHydraulics();
void waitAndAnimate(int waitTime);

void setup() {
  Serial.begin(9600);
  
  // Datarefs
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

  // Pins
  pinMode(pinNoseLight, OUTPUT); pinMode(pinWingStrobes, OUTPUT);
  pinMode(pinTailCombined, OUTPUT); pinMode(pinWingNavs, OUTPUT);
  pinMode(pinBeacon, OUTPUT); pinMode(pinEng1, OUTPUT);
  pinMode(pinEng2, OUTPUT); pinMode(pinLanding, OUTPUT);
  pinMode(pinProgButton, INPUT_PULLUP); 

  // Init
  analogWrite(pinNoseLight, VAL_OFF); 
  analogWrite(pinWingStrobes, 0); analogWrite(pinWingNavs, 0); 
  analogWrite(pinTailCombined, 0); analogWrite(pinBeacon, 0); 
  analogWrite(pinEng1, 0); analogWrite(pinEng2, 0); 
  analogWrite(pinLanding, 0);

  // Start Sequence
  noseGear.attach(pinNoseServo); delay(1000);
  mainGear.attach(pinMainServo); delay(1000);
  noseGear.write((int)noseCurrent); mainGear.write((int)mainCurrent); delay(200);
  supLeft.attach(pinSupLeft);   supLeft.write(SUP_L_RETRACT);   delay(500);
  supRight.attach(pinSupRight); supRight.write(SUP_R_RETRACT);  delay(1000);
  supFront.attach(pinSupFront); supFront.write(SUP_F_RETRACT);  delay(500);

  Serial.println("--- A330 READY ---");
  Serial.println("INFO: Press Button within 30s for DEMO, later for PROG MODE.");
}

// --- DEMO SEQUENCE FUNKTION ---
void runDemoSequence() {
  Serial.println(">> STARTING DEMO SEQUENCE...");
  demoModeActive = true;
  demoHasRun = true;
  
  // Reset
  navsActive=0; beaconActive=0; strobesActive=0; landingActive=0; turnoffActive=0;
  demoNoseLightVal = VAL_OFF;
  run_eng = 0; eng1TargetPWM = 0; eng2TargetPWM = 0;
  noseTarget = NOSE_POS_DOWN; mainTarget = MAIN_POS_DOWN;
  supLTarget=0; supRTarget=0; supFTarget=0; 
  
  // 1. Nav Lights
  navsActive = true; 
  waitAndAnimate(2000);

  // 2. Beacon
  beaconActive = true;
  waitAndAnimate(2000);

  // 3. Engines start (SEQUENZIELL)
  run_eng = 1; 
  
  // Engine 1
  eng1TargetPWM = ENG_IDLE_MIN; 
  waitAndAnimate(3000); 

  // Engine 2
  eng2TargetPWM = ENG_IDLE_MIN; 
  waitAndAnimate(3000); 

  // 4. Taxi Light und Runway Turnoff
  demoNoseLightVal = VAL_TAXI; 
  turnoffActive = true;
  waitAndAnimate(2000);

  // 5. Strobes
  strobesActive = true;
  waitAndAnimate(2000);

  // 6. Nose TO und Landing Lights
  demoNoseLightVal = VAL_TO;
  landingActive = true;
  waitAndAnimate(2000);

  // 7. Engines 80% (von 50 = 40 PWM)
  eng1TargetPWM = 40; eng2TargetPWM = 40;
  waitAndAnimate(2000);

  // 8. Nase anheben (Rotation Boden)
  supFTarget = 70; supLTarget = 0; supRTarget = 0; 
  waitAndAnimate(2000);

  // 9. Abheben und steil steigen
  int base = MOTION_NEUTRAL + AIR_LIFT_OFFSET;
  int pitchMove = 35; // Steil
  supFTarget = constrain(base + pitchMove, 0, 100);
  supLTarget = constrain(base - pitchMove, 0, 100);
  supRTarget = constrain(base - pitchMove, 0, 100);
  waitAndAnimate(4000); // LÄNGER (4s)

  // 10. Fahrwerk ein
  noseTarget = NOSE_POS_UP; mainTarget = MAIN_POS_UP;
  waitAndAnimate(4000); // Zeit für Gear

  // 11. Flacher steigen und Linksneigung (STEILER)
  pitchMove = 10; 
  int rollMove = -35; // Links Steil (War -20)
  supFTarget = constrain(base + pitchMove, 0, 100);
  supLTarget = constrain(base + rollMove - pitchMove, 0, 100);
  supRTarget = constrain(base - rollMove - pitchMove, 0, 100);
  waitAndAnimate(5000); // LÄNGER (5s)

  // 12. Lichter aus (Land, Turn, Taxi, TO)
  landingActive = false; turnoffActive = false; demoNoseLightVal = VAL_OFF;
  waitAndAnimate(2000);

  // 13. Geradeausflug Neutral
  supFTarget = base; supLTarget = base; supRTarget = base;
  waitAndAnimate(4000); // LÄNGER (4s)

  // 14. Rechtsneigung (STEILER)
  rollMove = 35; // Rechts Steil (War 20)
  supFTarget = constrain(base, 0, 100);
  supLTarget = constrain(base + rollMove, 0, 100);
  supRTarget = constrain(base - rollMove, 0, 100);
  waitAndAnimate(5000); // LÄNGER (5s)

  // 15. Geradeaus, Nase leicht runter, Gear Down, Engines Idle
  pitchMove = -10; 
  supFTarget = constrain(base + pitchMove, 0, 100);
  supLTarget = constrain(base - pitchMove, 0, 100); 
  supRTarget = constrain(base - pitchMove, 0, 100);
  
  noseTarget = NOSE_POS_DOWN; mainTarget = MAIN_POS_DOWN;
  eng1TargetPWM = ENG_IDLE_MIN; eng2TargetPWM = ENG_IDLE_MIN;
  waitAndAnimate(5000); // LÄNGER (5s) für stabilen Anflug

  // 16. Landelichter an und Nose TO
  landingActive = true; demoNoseLightVal = VAL_TO;
  waitAndAnimate(2000);

  // 17. Flare (Heck leicht runter / Nase hoch)
  pitchMove = 15; 
  supFTarget = constrain(base + pitchMove, 0, 100);
  supLTarget = constrain(base - pitchMove, 0, 100);
  supRTarget = constrain(base - pitchMove, 0, 100);
  waitAndAnimate(3000);

  // 18. Touchdown (Absenken + Nase runter)
  supLTarget = 0; supRTarget = 0; 
  supFTarget = 0; 
  waitAndAnimate(1000);
  waitAndAnimate(2000);

  // 19. Strobes und Landelichter aus
  strobesActive = false; landingActive = false;
  waitAndAnimate(2000);

  // 20. Taxi Licht aus
  demoNoseLightVal = VAL_OFF;
  waitAndAnimate(2000);

  // 21. Engines aus
  eng1TargetPWM = 0; eng2TargetPWM = 0;
  waitAndAnimate(2000);

  // 22. Beacon aus
  beaconActive = false;
  waitAndAnimate(2000);

  // 23. Nav Lights aus
  navsActive = false;
  waitAndAnimate(2000);

  Serial.println(">> DEMO FINISHED. LISTENING TO SIM...");
  demoModeActive = false;
}

// Hilfsfunktion: Wartet X ms und bewegt dabei weiter die Hydraulik
void waitAndAnimate(int waitTime) {
  unsigned long start = millis();
  while (millis() - start < (unsigned long)waitTime) {
    updateHydraulics(); 
    
    unsigned long now = millis();
    analogWrite(pinEng1, eng1TargetPWM);
    analogWrite(pinEng2, eng2TargetPWM);

    if (navsActive) analogWrite(pinWingNavs, (int)(NAV_WING_VAL * brightnessScale)); 
    else analogWrite(pinWingNavs, 0);
    
    unsigned long masterTimer = now % 1000; 
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

    // Nose Light Demo Override
    if (xGearHandle == 1 || demoModeActive) { 
       if (noseTarget == NOSE_POS_DOWN) { 
         analogWrite(pinNoseLight, (int)(demoNoseLightVal * brightnessScale));
       } else analogWrite(pinNoseLight, 0);
    } else analogWrite(pinNoseLight, 0);

    delay(REFRESH_RATE);
  }
}

void loop() {
  FlightSim.update(); 
  
  // --- BUTTON LOGIK ---
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
  
  // Serial CMD
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "engrun") { run_eng = 1; Serial.println(">> ENGINES ENABLED"); } 
    else if (cmd == "engstop") { run_eng = 0; Serial.println(">> ENGINES STOPPED"); }
    else if (cmd == "night") { brightnessScale = 0.3; Serial.println(">> MODE: NIGHT (30%)"); }
    else if (cmd == "day") { brightnessScale = 1.0; Serial.println(">> MODE: DAY (100%)"); }
    
    // Calibration
    else if (cmd.startsWith("cal_left")) {
      calMode = true; 
      manualRollOffset = cmd.substring(9).toInt(); manualPitchOffset = 0; 
      calTargetBank = -LIMIT_BANK; calTargetPitch = 0.0; 
      Serial.print(">> CAL: LEFT MAX | Off: "); Serial.println(manualRollOffset);
    }
    else if (cmd.startsWith("cal_right")) {
      calMode = true; 
      manualRollOffset = cmd.substring(10).toInt(); manualPitchOffset = 0;
      calTargetBank = LIMIT_BANK; calTargetPitch = 0.0;
      Serial.print(">> CAL: RIGHT MAX | Off: "); Serial.println(manualRollOffset);
    }
    else if (cmd.startsWith("cal_up")) {
      calMode = true; 
      manualPitchOffset = cmd.substring(7).toInt(); manualRollOffset = 0;
      calTargetBank = 0.0; calTargetPitch = LIMIT_PITCH_UP; 
      Serial.print(">> CAL: PITCH UP MAX | Off: "); Serial.println(manualPitchOffset);
    }
    else if (cmd.startsWith("cal_down")) {
      calMode = true; 
      manualPitchOffset = cmd.substring(9).toInt(); manualRollOffset = 0;
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
        strobesActive = xStrobeLight;
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

    // Gear
    if (xGearHandle == 1) { noseTarget = NOSE_POS_DOWN; mainTarget = MAIN_POS_DOWN; } 
    else { noseTarget = NOSE_POS_UP; mainTarget = MAIN_POS_UP; }

    // --- MOTION CALCULATION ---
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
    
    // Limits
    float rollClamped = constrain(rollDeg, -LIMIT_BANK, LIMIT_BANK);
    float pitchClamped;
    
    if (pitchDeg >= 0) pitchClamped = constrain(pitchDeg, 0, LIMIT_PITCH_UP);
    else pitchClamped = constrain(pitchDeg, -LIMIT_PITCH_DOWN, 0);

    int currentPitchOffset = FIXED_PITCH_OFFSET;
    if (calMode) currentPitchOffset = manualPitchOffset;

    if (isOnGround) {
      // BODEN:
      int targetFront = map((long)(pitchDeg * 10), 0, (long)(LIMIT_PITCH_UP*10), 0, 100);
      targetFront = constrain(targetFront, 0, 100);
      supLTarget = 0; supRTarget = 0; supFTarget = targetFront;
    } else {
      // FLUG:
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
    
    // --- DEBUG ---
    unsigned long now = millis();
    if (now - lastDebugTime > 500) {
        lastDebugTime = now;
        if (!calMode && FlightSim.isEnabled()) {
            Serial.print("M: P="); Serial.print((int)xPitch);
            Serial.print(" R="); Serial.print((int)xBank);
            Serial.print(" | F: "); Serial.print(supFTarget);
            Serial.print(" | L: "); Serial.print(supLTarget);
            Serial.print(" | R: "); Serial.println(supRTarget);
        }
    }
  }

  // UPDATE CALL (Hydraulik & Standard Lichter wenn nicht Demo)
  if (!demoModeActive) {
     updateHydraulics();

     unsigned long now = millis();
     if (eng1KickEnd > 0 && now > eng1KickEnd) { analogWrite(pinEng1, eng1TargetPWM); eng1KickEnd = 0; }
     if (eng2KickEnd > 0 && now > eng2KickEnd) { analogWrite(pinEng2, eng2TargetPWM); eng2KickEnd = 0; }
  
     if (navsActive) analogWrite(pinWingNavs, (int)(NAV_WING_VAL * brightnessScale)); 
     else analogWrite(pinWingNavs, 0);
     
     unsigned long masterTimer = now % 1000; 
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

  // --- MONITOR ---
  unsigned long now = millis();
  if (now - lastStatusTime >= 1000) {
    lastStatusTime = now;
    // (Gekürzter Monitor für Loop Übersicht)
  }
}

// --- HYDRAULIK FUNKTION ---
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