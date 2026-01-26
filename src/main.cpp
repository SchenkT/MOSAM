#include <Arduino.h>
#include <Servo.h>

// =========================================================
//      MOSAM v4.7 - NAV LIGHT ARRAY TEST
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

// --- PINS (INPUTS PANEL) ---
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
unsigned long lastOHPUpdate = 0; const int OHP_REFRESH_RATE = 50; 
bool debugMode = false; unsigned long lastDebugTime = 0;

// --- TIMERS ---
unsigned long changeTimer[50]; 
const unsigned long BLOCK_TIME = 1200; 

// --- STATE MEMORY ---
int last_Beacon = -1; int last_Strobe = -1; int last_Nav = -1;
int last_Nose = -1; int last_Land = -1; int last_Rwy = -1;
int last_Seat = -1; int last_Dome = -1; int last_WiperS = -1; int last_WiperF = -1;
int last_Call = -1; int last_IceW = -1; int last_IceE = -1;
int last_APUM = -1; int last_APUS = -1; int last_APUB = -1;
int last_XBleed = -1; int last_Elec = -1; int last_PTU = -1;
int last_Pack1 = -1; int last_Pack2 = -1;

// --- WRITE REFS ---
FlightSimInteger pan_Beacon; FlightSimInteger pan_Strobe; FlightSimInteger pan_Nav;         
FlightSimInteger pan_Nose; 
FlightSimInteger pan_LandL; FlightSimInteger pan_LandR; FlightSimInteger pan_LandN;
FlightSimInteger pan_Rwy; FlightSimInteger pan_Seat;    
FlightSimInteger pan_Dome; 
FlightSimInteger pan_Wiper; FlightSimInteger pan_Call;        
FlightSimInteger pan_IceW; FlightSimInteger pan_IceE1; FlightSimInteger pan_IceE2;
FlightSimInteger pan_APUM; FlightSimInteger pan_APUS; FlightSimInteger pan_APUB;
FlightSimInteger pan_XBleed; FlightSimInteger pan_Elec; FlightSimInteger pan_PTU;
FlightSimInteger pan_Pack1; FlightSimInteger pan_Pack2;

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
void updateHydraulics(); void updatePanelInputs(); void updateModelOutputs(); void waitAndAnimate(int waitTime); void runDemoSequence(); void printRow(String sub, int pin, String name, int switchVal, int lastVal, long simVal); void printDebugTable();

void setup() {
  Serial.begin(9600);
  for(int i=0; i<50; i++) changeTimer[i] = 0;

  // READ REFS
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

  // WRITE REFS
  pan_Beacon    = XPlaneRef("sim/cockpit2/switches/beacon_on");      
  pan_Strobe    = XPlaneRef("sim/cockpit2/switches/strobe_lights_on");      
  
  // *** NAV EXPERIMENT ***
  // Array Index 2 (3. Position)
  pan_Nav       = XPlaneRef("AirbusFBW/OHPLightSwitches[2]");  
  
  pan_Nose      = XPlaneRef("sim/cockpit2/switches/taxi_light_on");        
  pan_LandL     = XPlaneRef("sim/cockpit2/switches/landing_lights_switch[0]");    
  pan_LandR     = XPlaneRef("sim/cockpit2/switches/landing_lights_switch[1]");
  pan_LandN     = XPlaneRef("sim/cockpit2/switches/landing_lights_switch[2]");
  pan_Rwy       = XPlaneRef("sim/cockpit2/switches/runway_turnoff_lights_on");     
  pan_Seat      = XPlaneRef("sim/cockpit2/switches/fasten_seat_belts");    
  pan_Dome      = XPlaneRef("ckpt/oh/domeLight/anim"); 
  pan_Wiper     = XPlaneRef("sim/cockpit2/switches/wiper_speed"); 
  pan_Call      = XPlaneRef("AirbusFBW/OH/CallMech"); 
  pan_IceW      = XPlaneRef("sim/cockpit2/ice/ice_inlet_heat_on_per_engine[0]"); 
  pan_IceE1     = XPlaneRef("sim/cockpit2/ice/ice_inlet_heat_on_per_engine[0]");
  pan_IceE2     = XPlaneRef("sim/cockpit2/ice/ice_inlet_heat_on_per_engine[1]");
  pan_APUM      = XPlaneRef("sim/cockpit2/electrical/APU_master_switch");
  pan_APUS      = XPlaneRef("sim/cockpit2/electrical/APU_starter_switch");
  pan_APUB      = XPlaneRef("sim/cockpit2/bleedair/apu_bleed_on");
  pan_XBleed    = XPlaneRef("sim/cockpit2/bleedair/cross_tie_open"); 
  pan_Elec      = XPlaneRef("sim/cockpit2/hydraulics/actuators/electric_hydraulic_pump_on");
  pan_PTU       = XPlaneRef("sim/cockpit2/hydraulics/actuators/ptu_on");
  pan_Pack1     = XPlaneRef("sim/cockpit2/bleedair/bleed_air_on[0]");
  pan_Pack2     = XPlaneRef("sim/cockpit2/bleedair/bleed_air_on[1]");

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

  Serial.println("--- MOSAM v4.7 NAV ARRAY TEST ---");
}

void loop() {
  FlightSim.update(); 
  
  if (digitalRead(pinProgButton) == LOW) { delay(50); if (digitalRead(pinProgButton) == LOW) { if (millis() < 30000 && !demoHasRun) runDemoSequence(); else { analogWrite(pinEng1, 0); analogWrite(pinEng2, 0); Serial.println("PROG"); delay(100); asm("bkpt #251"); }}}

  unsigned long now = millis();
  if (!demoModeActive && !calMode) {
      if (now - lastOHPUpdate >= OHP_REFRESH_RATE) { lastOHPUpdate = now; updatePanelInputs(); }
  }

  if (!demoModeActive) { updateModelOutputs(); }

  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n'); cmd.trim();
    if (cmd.equalsIgnoreCase("debug")) debugMode = true; else if (cmd.equalsIgnoreCase("debugstop")) debugMode = false;
    else if (cmd.startsWith("cal_left")) { calMode = true; manualRollOffset = cmd.substring(9).toInt(); calTargetBank = -40; }
    else if (cmd == "cal_off") { calMode = false; manualRollOffset = 0; }
  }
  
  if (debugMode && (millis() - lastDebugTime > 500)) { 
      lastDebugTime = millis(); 
      printDebugTable();
  }
}

// =========================================================
// LOGIC: UPDATE PANEL 
// =========================================================
void updatePanelInputs() {
  int val; 

  // P03 Beacon
  val = (digitalRead(sw_Beacon) == LOW);
  if (val != last_Beacon) { pan_Beacon = val; last_Beacon = val; changeTimer[sw_Beacon] = millis(); }

  // P04 Strobe
  val = (digitalRead(sw_Strobe_On) == LOW);
  if (val != last_Strobe) { pan_Strobe = val; last_Strobe = val; changeTimer[sw_Strobe_On] = millis(); }

  // --- EXPERIMENT NAV ---
  val = (digitalRead(sw_Nav_Master) == LOW); // 1 if On
  if (val != last_Nav) { 
      // Wir senden 2 wenn an, 0 wenn aus (ToLiss Logic: 0=Off, 1=1, 2=2)
      pan_Nav = (val == 1) ? 2 : 0; 
      last_Nav = val; 
      changeTimer[sw_Nav_Master] = millis(); 
  }

  // P06/07 Nose
  int noseState = 0; 
  if (digitalRead(sw_Nose_TO) == LOW) noseState = 2;
  else if (digitalRead(sw_Nose_Taxi) == HIGH) noseState = 1; 
  if (noseState != last_Nose) { 
      pan_Nose = (noseState > 0) ? 1 : 0; 
      last_Nose = noseState; 
      changeTimer[sw_Nose_TO] = millis(); 
  }

  // P08 Rwy
  val = (digitalRead(sw_RwyTurnoff) == LOW);
  if (val != last_Rwy) { pan_Rwy = val; last_Rwy = val; changeTimer[sw_RwyTurnoff] = millis(); }

  // P09 Landing
  val = (digitalRead(sw_Landing_Master) == LOW);
  if (val != last_Land) { 
      pan_LandL = val; pan_LandR = val; pan_LandN = val;
      last_Land = val; 
      changeTimer[sw_Landing_Master] = millis();
  }

  // P10 Seatbelt
  val = (digitalRead(sw_Seatbelts) == LOW);
  if (val != last_Seat) { pan_Seat = val; last_Seat = val; changeTimer[sw_Seatbelts] = millis(); }

  // P11 Dome
  val = (digitalRead(sw_Dome_Dim) == LOW);
  if (val != last_Dome) { pan_Dome = val; last_Dome = val; changeTimer[sw_Dome_Dim] = millis(); }

  // P12/13 Wiper
  int wiperState = 0;
  if (digitalRead(sw_Wiper_Fast) == LOW) wiperState = 2; else if (digitalRead(sw_Wiper_Slow) == LOW) wiperState = 1;
  if (digitalRead(sw_Wiper_Slow) != last_WiperS || digitalRead(sw_Wiper_Fast) != last_WiperF) {
      pan_Wiper = (wiperState == 2) ? 35 : (wiperState == 1 ? 15 : 0);
      last_WiperS = digitalRead(sw_Wiper_Slow); last_WiperF = digitalRead(sw_Wiper_Fast);
      changeTimer[sw_Wiper_Slow] = millis();
  }

  // P14 Call
  val = (digitalRead(sw_Call_Btn) == LOW);
  if (val != last_Call) { pan_Call = val; last_Call = val; changeTimer[sw_Call_Btn] = millis(); }

  // P15/16 Ice
  val = (digitalRead(sw_Ice_Wing) == LOW);
  if (val != last_IceW) { pan_IceW = val; last_IceW = val; changeTimer[sw_Ice_Wing] = millis(); }
  val = (digitalRead(sw_Ice_Eng_Comb) == LOW);
  if (val != last_IceE) { pan_IceE1 = val; pan_IceE2 = val; last_IceE = val; changeTimer[sw_Ice_Eng_Comb] = millis(); }

  // P17 APU M
  val = (digitalRead(sw_APU_Master) == LOW);
  if (val != last_APUM) { pan_APUM = val; last_APUM = val; changeTimer[sw_APU_Master] = millis(); }

  // P18 APU S
  val = (digitalRead(sw_APU_Start) == LOW);
  if (val != last_APUS) { pan_APUS = val; last_APUS = val; changeTimer[sw_APU_Start] = millis(); }

  // P19 APU B
  val = (digitalRead(sw_APU_Bleed) == LOW);
  if (val != last_APUB) { pan_APUB = val; last_APUB = val; changeTimer[sw_APU_Bleed] = millis(); }

  // P20 XBleed
  val = (digitalRead(sw_XBleed_Open) == LOW);
  if (val != last_XBleed) { pan_XBleed = val; last_XBleed = val; changeTimer[sw_XBleed_Open] = millis(); }

  // P21 Elec
  val = (digitalRead(sw_ElecPump) == LOW);
  if (val != last_Elec) { pan_Elec = val; last_Elec = val; changeTimer[sw_ElecPump] = millis(); }

  // P22 PTU
  val = (digitalRead(sw_PTU_Off) == LOW) ? 0 : 1;
  if (val != last_PTU) { pan_PTU = val; last_PTU = val; changeTimer[sw_PTU_Off] = millis(); }

  // P23/24 Packs
  val = (digitalRead(sw_Pack1) == LOW);
  if (val != last_Pack1) { pan_Pack1 = val; last_Pack1 = val; changeTimer[sw_Pack1] = millis(); }
  val = (digitalRead(sw_Pack2) == LOW);
  if (val != last_Pack2) { pan_Pack2 = val; last_Pack2 = val; changeTimer[sw_Pack2] = millis(); }
}

void updateModelOutputs() {
  updateHydraulics();
  if ((FlightSim.isEnabled() || calMode) && !demoModeActive) { 
      bool isStrobe = (mod_Strobe == 1); 
      bool isBeacon = (mod_Beacon == 1);
      bool isNav    = (mod_Nav == 1);
      bool isLand   = (mod_Land == 1);
      bool isTaxi   = (mod_Taxi == 1);

      if (xGearHandle == 1) { 
          if (isLand) analogWrite(pinNoseLight, (int)(VAL_TO * brightnessScale)); 
          else if (isTaxi) analogWrite(pinNoseLight, (int)(VAL_TAXI * brightnessScale)); 
          else analogWrite(pinNoseLight, 0); 
      } else analogWrite(pinNoseLight, 0); 
      
      int t1=0; if (xEng1N1 > 1) t1 = map((int)(float)xEng1N1, 15, 100, 0, 50); analogWrite(pinEng1, t1);
      int t2=0; if (xEng2N1 > 1) t2 = map((int)(float)xEng2N1, 15, 100, 0, 50); analogWrite(pinEng2, t2);

      if (isNav) analogWrite(pinWingNavs, (int)(40*brightnessScale)); else analogWrite(pinWingNavs, 0);
      if (isLand) analogWrite(pinLanding, (int)(100*brightnessScale)); 
      else if (mod_Rwy > 0.1) analogWrite(pinLanding, (int)(40*brightnessScale)); 
      else analogWrite(pinLanding, 0);

      unsigned long mt = millis() % 1000; 
      if (isBeacon && mt >= 500 && mt < 600) analogWrite(pinBeacon, (int)(255*brightnessScale)); else analogWrite(pinBeacon, 0);
      int strOut = 0; if (isStrobe) { if ((mt < 30) || (mt >= 100 && mt < 130)) strOut = (int)(STROBE_FLASH_VAL * brightnessScale); }
      analogWrite(pinWingStrobes, strOut);
      int tailOut = 0; if (isStrobe && mt < 100) tailOut = (int)(STROBE_FLASH_VAL * brightnessScale);
      if (tailOut > 0) analogWrite(pinTailCombined, tailOut); else if (isNav) analogWrite(pinTailCombined, (int)(NAV_TAIL_DIM * brightnessScale)); else analogWrite(pinTailCombined, 0);
  }
}

void printRow(String sub, int pin, String name, int switchVal, int lastVal, long simVal) {
    String pS = (switchVal == 0) ? "ON " : "OFF"; 
    bool blocked = (millis() - changeTimer[pin] < BLOCK_TIME);
    String chg = blocked ? " + " : " - "; 
    
    Serial.print("P"); Serial.print(sub); Serial.print("/T"); Serial.print(pin); 
    Serial.print(" "); Serial.print(name); 
    while(name.length() < 9) { Serial.print(" "); name += " "; } 
    Serial.print("| "); Serial.print(pS); 
    Serial.print(" |  "); Serial.print(chg); 
    Serial.print("  | "); Serial.println(simVal);
}

void printDebugTable() {
    Serial.println("\n--- DEBUG STATUS (v4.7) ---"); 
    Serial.println("PIN/TEENSY/NAME      | PANEL | BLOCK | SIM (Read)");
    
    printRow("03", 18, "BEACON", digitalRead(sw_Beacon), last_Beacon, mod_Beacon);
    printRow("04", 16, "STROBE", digitalRead(sw_Strobe_On), last_Strobe, mod_Strobe);
    
    // NAV ROW (SPECIAL ATTENTION)
    printRow("05", 17, "NAV_ARR", digitalRead(sw_Nav_Master), last_Nav, mod_Nav);
    
    Serial.print("P06/07 NOSE (TX/TO)  | "); 
    if (digitalRead(sw_Nose_TO)==LOW) Serial.print("TO "); 
    else if (digitalRead(sw_Nose_Taxi)==HIGH) Serial.print("TAXI"); 
    else Serial.print("OFF ");
    bool nBlocked = (millis() - changeTimer[sw_Nose_TO] < BLOCK_TIME);
    Serial.print(" |  "); Serial.print(nBlocked ? " + " : " - "); Serial.print("  | "); Serial.println(mod_Taxi);

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