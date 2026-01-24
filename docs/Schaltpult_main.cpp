#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>  // I2C LCD-Bibliothek
//#include <Joystick.h>           // Falls verwendet
#include <math.h>
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

// LCD an Adresse 0x27 (üblich), 20 Spalten, 4 Zeilen
LiquidCrystal_I2C lcd(0x27, 20, 4);




// Button-Pin-Zuweisung
const int buttonPins[] = {
  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
   15,16,25, 28, 30, 31, 32,33,
  34, 35, 36, 37, 38, 39, 40, 41, 42, 43,44
};


int joystick_delay = 50;
int debugmode=1;
//int switched=0;

//Encoder
Encoder dreh_1(24,26);
Encoder dreh_2(27,29);
long dreh_1_oldpos = -999;
long dreh_2_oldpos = -999;
long dreh_1_pos;
long dreh_2_pos;
int dreh_1_diff;
int dreh_2_diff;


//DataRefs und Achsen- Zuweisungen und interne Variablen für jeweiligen Status Flugzeug und Status Pult
const int axisPins[] = {A0};
int a0old=0;
int a0joy=0;
int y;
float yjoy;
int a0;
bool lastButtonStates[sizeof(buttonPins)/sizeof(buttonPins[0])];
FlightSimInteger beacon;  // Globale Variable
int beacon_f;
int beacon_p;
FlightSimInteger strobe;  // Globale Variable
int strobe_f;
int strobe_p1;
int strobe_p2;
FlightSimInteger nose;  // Globale Variable
int nose_f;
int nose_p1;
int nose_p2;
FlightSimInteger seatbelt;  // Globale Variable
int seatbelt_f;
int seatbelt_p;
FlightSimInteger landinglights;  // Globale Variable
int landinglights_f;
int landinglights_p;
FlightSimInteger runway;  // Globale Variable
int runway_f;
int runway_p;
FlightSimInteger gear;  // Globale Variable
int gear_f;
int gear_p;
FlightSimFloat flaps;  // Globale Variable
FlightSimFloat flaps2;  // Globale Variable
double flaps_f_read;
double flaps_f2_read;
double flaps_f;
double flaps_f2;
int flaps_p;
int flaps_p1;
int flaps_p2;
int flaps_p3;
int flaps_p4;
unsigned long currentMillis;
unsigned long lastFlapsUpdate = 0;  // Zeitpunkt der letzten Ausführung
unsigned long lastLCDUpdate = 0;  // Zeitpunkt der letzten Ausführung
unsigned long lastSpeedbrakeUpdate = 0;  // Zeitpunkt der letzten Ausführung
unsigned long lastSwitchesUpdate = 0;  // Zeitpunkt der letzten Ausführung
FlightSimFloat speedbrake;  // Globale Variable
FlightSimFloat speedbrake_up;
double speedbrake_f;
int speedbrake_p_armed;
float speedbrake_f_soll;
int speedbrake_up_f;
int speedbrake_p1;
int speedbrake_p2;
FlightSimInteger groundspeed;  // Globale Variable
float groundspeed_f;
int reverse_p;
int reverse;
FlightSimInteger com1_active;
FlightSimInteger com2_active;
FlightSimInteger com1_stby_khz;
FlightSimInteger com1_stby_mhz;
FlightSimInteger com2_stby_khz;
FlightSimInteger com2_stby_mhz;
FlightSimInteger com1_left_khz;
FlightSimInteger com1_left_mhz;
FlightSimInteger com2_left_khz;
FlightSimInteger com2_left_mhz;
int selectedline;
int com1a_f;
int com2a_f;
int com1a_f1;
int com1a_f2;
int com2a_f1;
int com2a_f2;
int com1s_f1;
int com1s_f2;
int com2s_f1;
int com2s_f2;
int umsch_fcu_com;
int umsch_fcu;
int umsch_com;
int com1_flip_p;
int com2_flip_p;
FlightSimInteger hdg;
//FlightSimInteger hdg_fcu;
FlightSimInteger hdg_managed;
FlightSimInteger hdg_dashed;
FlightSimInteger spd;
FlightSimInteger spd_dashed;
FlightSimInteger spd_managed;
FlightSimInteger alt;
FlightSimInteger alt_managed;
FlightSimInteger alt_step;
FlightSimInteger vs;
FlightSimInteger vs_armed;
FlightSimInteger vs_dashed;
int hdg_f;
int hdg_managed_f;
int hdg_dashed_f;
int spd_f;
int spd_dashed_f;
int spd_managed_f;
int alt_f;
int alt_managed_f;
int alt_step_f;
int vs_f;
int vs_armed_f;
int vs_dashed_f;
int hdg_push_p;
int hdg_pull_p;
int alt_push_p;
int alt_pull_p;
int spd_push_p;
int spd_pull_p;
int vs_push_p;
int vs_pull_p;

char buffer[5];

void setup() {
  Serial.begin(115200);

  // LCD initialisieren
  lcd.init();
  lcd.backlight();

  //Encoder

  
  // LED aus
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Buttons konfigurieren
  for (unsigned int i = 0; i < sizeof(buttonPins)/sizeof(buttonPins[0]); i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
    lastButtonStates[i] = digitalRead(buttonPins[i]);
  }


  beacon        = XPlaneRef("sim/cockpit2/switches/beacon_on");
  strobe        = XPlaneRef("ckpt/oh/strobeLight/anim");
  nose          = XPlaneRef("ckpt/oh/taxiLight/anim");
  seatbelt      = XPlaneRef("AirbusFBW/SeatBeltSignsOn");
  landinglights = XPlaneRef("sim/cockpit2/switches/landing_lights_switch");
  runway        = XPlaneRef("ckpt/oh/rwyTurnOff/anim");
  flaps         = XPlaneRef("ckpt/flapMain/anim");
  gear          = XPlaneRef("AirbusFBW/GearLever");
  speedbrake    = XPlaneRef("sim/cockpit2/controls/speedbrake_ratio");
  speedbrake    = XPlaneRef("ckpt/speedbrake/anim");
  speedbrake_up  = XPlaneRef("ckpt/speedbrakeUp/anim");
  groundspeed   = XPlaneRef("sim/flightmodel/position/groundspeed");
  com1_active   = XPlaneRef("sim/cockpit2/radios/actuators/com1_left_frequency_hz_833");
  com2_active   = XPlaneRef("sim/cockpit2/radios/actuators/com2_left_frequency_hz_833");
  com1_left_khz = XPlaneRef("sim/cockpit2/radios/actuators/com1_frequency_khz");
  com1_left_mhz = XPlaneRef("sim/cockpit2/radios/actuators/com1_frequency_Mhz");
  com2_left_khz = XPlaneRef("sim/cockpit2/radios/actuators/com2_frequency_khz");
  com2_left_mhz = XPlaneRef("sim/cockpit2/radios/actuators/com2_frequency_Mhz");
  com1_stby_khz = XPlaneRef("sim/cockpit2/radios/actuators/com1_standby_frequency_khz");
  com1_stby_mhz = XPlaneRef("sim/cockpit2/radios/actuators/com1_standby_frequency_Mhz");
  com2_stby_khz = XPlaneRef("sim/cockpit2/radios/actuators/com2_standby_frequency_khz");
  com2_stby_mhz = XPlaneRef("sim/cockpit2/radios/actuators/com2_standby_frequency_Mhz");
  //hdg           =XPlaneRef("ckpt/fcu/hading/anim");
  hdg             =XPlaneRef("sim/cockpit/autopilot/heading_mag");
  hdg_managed   =XPlaneRef("AirbusFBW/HDGmanaged");
  hdg_dashed    =XPlaneRef("AirbusFBW/HDGdashed");
  spd           =XPlaneRef("sim/cockpit/autopilot/airspeed");
  spd_dashed    =XPlaneRef("AirbusFBW/SPDdashed");
  spd_managed   =XPlaneRef("AirbusFBW/SPDmanaged");
  alt           =XPlaneRef("sim/cockpit/autopilot/altitude");
  alt_managed   =XPlaneRef("AirbusFBW/ALTmanaged");
  alt_step      =XPlaneRef("ckpt/fcu/altitudeStep/anim");
  vs            =XPlaneRef("sim/cockpit/autopilot/vertical_velocity");
  vs_armed      =XPlaneRef("AirbusFBW/APVerticalArmed");
  vs_dashed     =XPlaneRef("AirbusFBW/VSdashed");

  //sim/cockpit2/autopilot/heading_dial_deg_mag_pilot
  //sim/cockpit/autopilot/heading_mag
  //AirbusFBW/PullHDGSel
  //AirbusFBW/PushHDGSel
  //toliss_airbus/hdgtrk_button_push ist der umschalter zw heading-track und VS-FPA

}



void checkGear() {
  FlightSim.update();
  gear_f=gear.read();
  gear_p = digitalRead(35);

  if (gear_p == 0){//am pult ausgefahren
    if (gear_f ==0){// im Flugzeug eingefahren
      // im Flugzeug schalten
      Joystick.button(9, 1);
      delay(joystick_delay);
      Joystick.button(9, 0);
    }
  } else { // beacon am pult ausgeschaltet

    if (gear_f ==1){// im Flugzeug ausgefahren
      // im Flugzeug schalten
      Joystick.button(9, 1);
      delay(joystick_delay);
      Joystick.button(9, 0);
    }
  }
}


void checkSpeedbrake() {
  speedbrake_p_armed = digitalRead(36);
  speedbrake_p1 = digitalRead(28);
  speedbrake_p2 = digitalRead(34);
  //speedbrake_f_soll
  //speedbrake soll
  if (speedbrake_p1==0){
    speedbrake_f_soll=0;
  } else if (speedbrake_p2==0){
    speedbrake_f_soll=4;
  } else {
    speedbrake_f_soll=2;
  }
  
  if ((speedbrake_f_soll-(speedbrake_f*4))>0){
    Joystick.button(16, 1);//extend one
    delay(joystick_delay);
    Joystick.button(16, 0);
  }
  
  if ((speedbrake_f*4)-speedbrake_f_soll>0){
    Joystick.button(15, 1);//retract one
    delay(joystick_delay);
    Joystick.button(15, 0);
  }
 
  if (speedbrake_f==0 ){
    if (speedbrake_p_armed==1 &&speedbrake_up ==0){//speedbrake up????? checken ob aktualisiert - Bedingungen prüfen - pos passt, up down noch nicht
     
      Joystick.button(14, 1);//toggle arm
      delay(joystick_delay);
      Joystick.button(14, 0);
    }
  } 
  if (speedbrake_up==1 && speedbrake_p_armed==0){
    Joystick.button(14, 1);//toggle arm
    delay(joystick_delay);
    Joystick.button(14, 0);
  }
}






void checkFlaps(){
  FlightSim.update();  // GANZ WICHTIG – sonst kommen keine Daten!
  flaps_f_read  = flaps.read();
  flaps_f= 4* flaps_f_read;
  flaps_p1 = digitalRead(40);
  flaps_p2 = digitalRead(38);
  flaps_p3 = digitalRead(39);
  flaps_p4 = digitalRead(37);

  if (flaps_p1==0) {
    flaps_p=1;
  }else if (flaps_p2==0) {
    flaps_p=2;
  }else if (flaps_p3==0) {
    flaps_p=3;
  }else if(flaps_p4==0) {
    flaps_p=4;
  }else {
    flaps_p=0;
  }
  if (flaps_f < flaps_p ) {
    Joystick.button(21, 1);
    delay(joystick_delay);
    Joystick.button(21, 0);
  }else if (flaps_f > flaps_p) {
    Joystick.button(22, 1);
    delay(joystick_delay);
    Joystick.button(22, 0);
  }
}








// H A U P T S C H L E I F E
void loop() {

  //_______________________________________________
  //  A B F R A G E   F L U G Z E U G - Z U S T Ä N D E 
  FlightSim.update();  // GANZ WICHTIG – sonst kommen keine Daten!


  a0 = analogRead(A0);//axisPins[0]);
  beacon_f=beacon.read();
  groundspeed_f=groundspeed.read();
  seatbelt_f = seatbelt.read();
  runway_f = runway.read();
  landinglights_f = landinglights.read();
  strobe_f=strobe.read();
  nose_f=nose.read();
  speedbrake_f=speedbrake.read();
  speedbrake_up_f=speedbrake_up.read();
  com1a_f1=com1_left_mhz.read();
  com1a_f2=com1_left_khz.read();
  com2a_f1=com2_left_mhz.read();
  com2a_f2=com2_left_khz.read();
  com1s_f1=com1_stby_mhz.read();
  com1s_f2=com1_stby_khz.read();
  com2s_f1=com2_stby_mhz.read();
  com2s_f2=com2_stby_khz.read();

  umsch_fcu_com=digitalRead(10);
  umsch_fcu=digitalRead(9);
  umsch_com=digitalRead(11);
 
  hdg_f=hdg.read();
  hdg_managed_f=hdg_managed.read();
  hdg_dashed_f=hdg_dashed.read();
  spd_f=spd.read();
  spd_dashed_f=spd_dashed.read();
  spd_managed_f=spd_managed.read();
  alt_f=alt.read();
  alt_managed_f=alt_managed.read();
  alt_step_f=alt_step.read();
  vs_f=vs.read();
  vs_armed_f=vs_armed.read();
  vs_dashed_f=vs_dashed.read();



  //Encoder
  dreh_1_diff=0;
  dreh_2_diff=0;
  //dreh_1_pos;
  

/*

int alt_push_p;
int alt_pull_p;
int spd:push_p;
int spd_pull_p;
int vs_push_p;
int vs_pull_p;
*/
  

  if (umsch_fcu_com ==0){//FCU selected
    if (umsch_fcu ==1) {//HDG Alt selected
      //selectedline=1;


      //if (switched!=1){
        //dreh_1.write(hdg_f);
        //dreh_1_oldpos=hdg_f;
        //dreh_2.write(alt_f);
        //dreh_2_oldpos=alt_f;
        //switched=1;

      //} else{
        dreh_1_pos = dreh_1.read()/4 ;/// 4; // Div durch 4 zur Feinanpassung (optional)
        if (dreh_1_pos != dreh_1_oldpos) {
          dreh_1_diff=dreh_1_oldpos-dreh_1_pos;
          dreh_1_oldpos = dreh_1_pos; 
        }

        if (dreh_1_diff+hdg>359){
          dreh_1_diff%=360;
        } else if (dreh_1_diff+hdg<0){
          dreh_1_diff+=360;
        }
        hdg=hdg+dreh_1_diff;
        hdg_push_p=digitalRead(25);
        hdg_pull_p=digitalRead(41);
        if (hdg_push_p==0){
          Joystick.button(24,1);
          delay(joystick_delay);
          Joystick.button(24,0);
        } else if (hdg_pull_p==0){
          Joystick.button(25,1);
          delay(joystick_delay);
          Joystick.button(25,0);
        }

        dreh_2_pos = dreh_2.read()/4 ;/// 4; // Div durch 4 zur Feinanpassung (optional)
        if (dreh_2_pos != dreh_2_oldpos) {
          dreh_2_diff=dreh_2_oldpos-dreh_2_pos;
          dreh_2_oldpos = dreh_2_pos;
          dreh_2_diff*= 100 * pow(10,alt_step_f);
        }

        if (dreh_2_diff+alt>49000){
          alt=49000;
        } else if (dreh_2_diff+alt<100){
          alt=100;
        }else{
        
          alt=alt+dreh_2_diff;
        }
        alt_push_p=digitalRead(32);
        alt_pull_p=digitalRead(16);
        if (alt_push_p==0){
          Joystick.button(26,1);
          delay(joystick_delay);
          Joystick.button(26,0);
        } else if (alt_pull_p==0){
          Joystick.button(27,1);
          delay(joystick_delay);
          Joystick.button(27,0);
        }
      //}

    }else {//speed & VertSpeed selected
      //selectedline=2;
      
      //if (switched!=2){
        //dreh_1.write(spd_f);
        //dreh_1_oldpos=spd_f;
        //dreh_2.write(vs_f);
        //dreh_2_oldpos=vs_f;
        //switched=2;
      //}else {
      
        dreh_1_pos = dreh_1.read()/4 ;/// 4; // Div durch 4 zur Feinanpassung (optional)
        if (dreh_1_pos != dreh_1_oldpos) {
          dreh_1_diff=dreh_1_oldpos-dreh_1_pos;
          dreh_1_oldpos = dreh_1_pos; 
        }
        if (dreh_1_diff+spd>399){
          spd=399;
        } else if (dreh_1_diff+spd<100){
          spd=100;
        }else{
        
          spd=spd+dreh_1_diff;
        }
        
        

        spd_push_p=digitalRead(25);
        spd_pull_p=digitalRead(41);
        if (spd_push_p==0){
          Joystick.button(28,1);
          delay(joystick_delay);
          Joystick.button(28,0);
        } else if (spd_pull_p==0){
          Joystick.button(29,1);
          delay(joystick_delay);
          Joystick.button(29,0);
        }


        dreh_2_pos = dreh_2.read()/4 ;/// 4; // Div durch 4 zur Feinanpassung (optional)
        if (dreh_2_pos != dreh_2_oldpos) {
          dreh_2_diff=dreh_2_oldpos-dreh_2_pos;
          dreh_2_oldpos = dreh_2_pos;
          dreh_2_diff*= 100 ;
        }

        if (dreh_2_diff+vs>6000){
          vs=6000;
        } else if (dreh_2_diff+alt<-6000){
          vs=-6000;
        }else{
        
          vs=vs+dreh_2_diff;
        }

        vs_push_p=digitalRead(32);
        vs_pull_p=digitalRead(16);
        if (vs_push_p==0){
          Joystick.button(30,1);
          delay(joystick_delay);
          Joystick.button(30,0);
        } else if (vs_pull_p==0){
          Joystick.button(31,1);
          delay(joystick_delay);
          Joystick.button(31,0);
        }
      //} 

    }

  }else {//COM selected
    if (umsch_com ==0) {//com1 selected
      //selectedline=3;
      
      //if (switched!=3){
        //dreh_1.write(com1s_f1);
        //dreh_1_oldpos=com1s_f1;
        //dreh_2.write(com1s_f2);
        //dreh_2_oldpos=com1s_f2;
        //switched=3;
      //}else {

        dreh_1_pos = dreh_1.read()/4 ;/// 4; // Div durch 4 zur Feinanpassung (optional)
        if (dreh_1_pos != dreh_1_oldpos) {
          dreh_1_diff=dreh_1_oldpos-dreh_1_pos;
          dreh_1_oldpos = dreh_1_pos; 
        }
        if (dreh_1_diff+com1_stby_mhz>136){
          //dreh_1_diff=0;
          com1_stby_mhz=118;
        } else if (dreh_1_diff+com1_stby_mhz<118){
          //dreh_1_diff=0;
          com1_stby_mhz=136;
        }else{
          com1_stby_mhz=com1_stby_mhz+dreh_1_diff;
        }

        dreh_2_pos = dreh_2.read()/4 ;/// 4; // Div durch 4 zur Feinanpassung (optional)
        if (dreh_2_pos != dreh_2_oldpos) {
          dreh_2_diff=dreh_2_oldpos-dreh_2_pos;
          dreh_2_oldpos = dreh_2_pos; 
          dreh_2_diff*=5;
        }
        if (dreh_2_diff+com1_stby_khz>990){
          //dreh_1_diff=0;
          com1_stby_khz=0;
        } else if (dreh_2_diff+com1_stby_khz<0){
          //dreh_1_diff=0;
          com1_stby_khz=990;
        }else{
          com1_stby_khz=com1_stby_khz+dreh_2_diff;
        }
        com1_flip_p=digitalRead(32);
        if (com1_flip_p==0){
          Joystick.button(10,1);
          delay(joystick_delay);
          Joystick.button(10,0);
          delay(joystick_delay);

          
        } 
      //}
    }


    else {//com2 selected
      //selectedline = 4;
      dreh_1_pos = dreh_1.read()/4 ;/// 4; // Div durch 4 zur Feinanpassung (optional)
        if (dreh_1_pos != dreh_1_oldpos) {
          dreh_1_diff=dreh_1_oldpos-dreh_1_pos;
          dreh_1_oldpos = dreh_1_pos; 
        }
        if (dreh_1_diff+com2_stby_mhz>136){
          //dreh_1_diff=0;
          com2_stby_mhz=118;
        } else if (dreh_1_diff+com2_stby_mhz<118){
          //dreh_1_diff=0;
          com2_stby_mhz=136;
        }else{
          com2_stby_mhz=com2_stby_mhz+dreh_1_diff;
        }

        dreh_2_pos = dreh_2.read()/4 ;/// 4; // Div durch 4 zur Feinanpassung (optional)
        if (dreh_2_pos != dreh_2_oldpos) {
          dreh_2_diff=dreh_2_oldpos-dreh_2_pos;
          dreh_2_oldpos = dreh_2_pos; 
          dreh_2_diff*=5;
        }
        if (dreh_2_diff+com2_stby_khz>990){
          //dreh_1_diff=0;
          com2_stby_khz=0;
        } else if (dreh_2_diff+com2_stby_khz<0){
          //dreh_1_diff=0;
          com2_stby_khz=990;
        }else{
          com2_stby_khz=com2_stby_khz+dreh_2_diff;
        }
        com2_flip_p=digitalRead(32);
        if (com2_flip_p==0){
          Joystick.button(11,1);
          delay(joystick_delay);
          Joystick.button(11,0);
        } 
    }
  }

  //
  //--------------------------------------
  //Analog-Achse Throttle
  //int a0 = analogRead(A0);//axisPins[0]);

    //wenn poti andersrum logarithmisch 
    if (a0>1000) {
      a0joy=1023;
      a0old=a0;
    } else if (a0>900){
      a0joy=740;
      a0old=a0;
    } else if (a0>780){
      a0joy=520;
      a0old=a0;
    } else if (a0<20){
      a0joy=0;
      a0old=a0;
    } else {
      y=a0;
      if (y < 6) y = 6;         // unterer Grenzwert

      //yjoy= 520*(-log((780.0 - y) / 774.0) / 6.0);
      yjoy=y;

      if (yjoy>520){
        yjoy=520;
      }
      if (abs(yjoy-a0old)>3) {
        a0joy=yjoy;
        a0old=yjoy;
      }
    }
  y=a0;
  Joystick.X(a0joy);

  //Serial.print("Throttle  ");Serial.print(a0); Serial.print("  ");Serial.println(a0joy);

  if (currentMillis - lastSwitchesUpdate >= 1500) {
    lastSwitchesUpdate = currentMillis;

    FlightSim.update();  // GANZ WICHTIG – sonst kommen keine Daten!

    a0 = analogRead(A0);//axisPins[0]);
    beacon_f=beacon.read();
    groundspeed_f=groundspeed.read();
    seatbelt_f = seatbelt.read();
    runway_f = runway.read();
    landinglights_f = landinglights.read();
    strobe_f=strobe.read();
    nose_f=nose.read();
    speedbrake_f=speedbrake.read();
    speedbrake_up_f=speedbrake_up.read();
    com1a_f1=com1_left_mhz.read();
    com1a_f2=com1_left_khz.read();
    com2a_f1=com2_left_mhz.read();
    com2a_f2=com2_left_khz.read();
    com1s_f1=com1_stby_mhz.read();
    com1s_f2=com1_stby_khz.read();
    com2s_f1=com2_stby_mhz.read();
    com2s_f2=com2_stby_khz.read();

    umsch_fcu_com=digitalRead(10);
    umsch_fcu=digitalRead(9);
    umsch_com=digitalRead(11);
  
    hdg_f=hdg.read();
    hdg_managed_f=hdg_managed.read();
    hdg_dashed_f=hdg_dashed.read();
    spd_f=spd.read();
    spd_dashed_f=spd_dashed.read();
    spd_managed_f=spd_managed.read();
    alt_f=alt.read();
    alt_managed_f=alt_managed.read();
    alt_step_f=alt_step.read();
    vs_f=vs.read();
    vs_armed_f=vs_armed.read();
    vs_dashed_f=vs_dashed.read();

    //Beacon
    //FlightSim.update();
    //beacon_f=beacon.read();
    beacon_p = digitalRead(4);

    if (beacon_p == 0){//am pult eingeschaltet
      //FlightSim.update();  // GANZ WICHTIG – sonst kommen keine Daten!
      //beacon_f=beacon.read();
      if (beacon_f ==0){// im Flugzeug ausgeschaltet
        // im Flugzeug schalten
        Joystick.button(4, 1);
        delay(joystick_delay);
        Joystick.button(4, 0);
        delay(joystick_delay);
      }
    } else { // beacon am pult ausgeschaltet
      //FlightSim.update();  // GANZ WICHTIG – sonst kommen keine Daten!
      //beacon_f=beacon.read();
      if (beacon_f ==1){// im Flugzeug eingeschaltet
        // im Flugzeug schalten
        Joystick.button(4, 1);
        delay(joystick_delay);
        Joystick.button(4, 0);
        delay(joystick_delay);
      }
    //FlightSim.update();
    //beacon_f=beacon.read();
    }
    

    //Reverse-Thrust
    //FlightSim.update();
    //groundspeed_f=groundspeed.read();
    reverse_p = digitalRead(30);
    //Serial.print(" Reverse button  ");Serial.print(reverse_p);

    if (reverse_p == 0) {
      Serial.println(" Reverse button pressed ");
      if (groundspeed>80 && a0joy==0) {
        Joystick.button(25, 1);
        reverse=1;
      }
    }
    if (reverse==1){
      if (groundspeed<70){
        Joystick.button(23, 0);
        reverse=0;
      }
    }

    //Seatbelts
    //FlightSim.update();
    //seatbelt_f = seatbelt.read();
    seatbelt_p = digitalRead(12);

    if (seatbelt_p + seatbelt_f !=1){// im Flugzeug ausgeschaltet
        // im Flugzeug schalten
         
        Joystick.button(12, 1);
        delay(joystick_delay);
        Joystick.button(12, 0);
        
      }
 

    //runway
    //FlightSim.update();
    //runway_f = runway.read();
    runway_p = digitalRead(7);

    if (runway_p == 0){//am pult eingeschaltet
      //FlightSim.update();  // GANZ WICHTIG – sonst kommen keine Daten!
      //runway_f=runway.read();
      if (runway_f ==0){// im Flugzeug ausgeschaltet
        // im Flugzeug schalten
        Joystick.button(7, 1);
        delay(joystick_delay);
        Joystick.button(7, 0);
      }
    } else { // seatbelt am pult ausgeschaltet
      //FlightSim.update();  // GANZ WICHTIG – sonst kommen keine Daten!
      //runway_f=runway.read();
      if (runway_f ==1){// im Flugzeug eingeschaltet
        // im Flugzeug schalten
        Joystick.button(7, 1);
        delay(joystick_delay);
        Joystick.button(7, 0);
      }
    }

    //landinglights
    //FlightSim.update();
    //landinglights_f = landinglights.read();
    landinglights_p = digitalRead(8);

    if (landinglights_p == 0){//am pult eingeschaltet
      //FlightSim.update();  // GANZ WICHTIG – sonst kommen keine Daten!
      //landinglights_f=landinglights.read();
      if (landinglights_f ==0){// im Flugzeug ausgeschaltet
        // im Flugzeug schalten
        Joystick.button(8, 1);
        Joystick.button(18, 1);
        delay(joystick_delay);
        Joystick.button(8, 0);
        Joystick.button(18, 0);

      }
    } else { // seatbelt am pult ausgeschaltet
      //FlightSim.update();  // GANZ WICHTIG – sonst kommen keine Daten!
      //landinglights_f=landinglights.read();
      if (landinglights_f ==1){// im Flugzeug eingeschaltet
        // im Flugzeug schalten
        Joystick.button(19, 1);
        Joystick.button(20, 1);
        delay(joystick_delay);
        Joystick.button(19, 0);
        Joystick.button(20, 0);
      }
    }

    //Strobe
    //FlightSim.update();  // GANZ WICHTIG – sonst kommen keine Daten!
    //strobe_f=strobe.read();
    strobe_p1 = digitalRead(2);//LOW: OFF
    strobe_p2 = digitalRead(3);//LOW: OFF    beide HIGH: AUTO
    //Serial.print("vor strobe-schleife  ");Serial.print(strobe_p1);Serial.print("  ");Serial.print(strobe_p2);

    if (strobe_p1 == 0){//am pult AUSgeschaltet
      //FlightSim.update();  // GANZ WICHTIG – sonst kommen keine Daten!
      //strobe_f=strobe.read();
      if (strobe_f >0){// im Flugzeug eingeschaltet
        // im Flugzeug schalten
        Joystick.button(2, 1);
        delay(joystick_delay);
        Joystick.button(2, 0);
        //delay(joystick_delay);
      }
    }else if (strobe_p2 == 0){//am pult EINgeschaltet
    //Serial.println("in strobe-p2-schleife");
      //FlightSim.update();
      //strobe_f=strobe.read();
      if (strobe_f <2){// im Flugzeug ausgeschaltet
        // im Flugzeug schalten
        Joystick.button(3, 1);
        delay(joystick_delay);
        Joystick.button(3, 0);
        //delay(joystick_delay);
      }

    } else { 
      
      if (strobe_f >1){// im Flugzeug ausgeschaltet
        // im Flugzeug schalten
        Joystick.button(2, 1);
        delay(joystick_delay);
        Joystick.button(2, 0);
      }else if (strobe_f <1){// im Flugzeug ausgeschaltet
        // im Flugzeug schalten
        Joystick.button(3, 1);
        delay(joystick_delay);
        Joystick.button(3, 0);
        //delay(joystick_delay);
      }
    }

    //NoseLight
    //FlightSim.update();  // GANZ WICHTIG – sonst kommen keine Daten!
    //nose_f=nose.read();
    nose_p1 = digitalRead(5);//LOW: OFF
    nose_p2 = digitalRead(6);//LOW: OFF    beide HIGH: AUTO
    

    if (nose_p1 == 0){//am pult AUSgeschaltet
    
      if (nose_f >0){// im Flugzeug eingeschaltet
        // im Flugzeug schalten
        Joystick.button(5, 1);
        delay(joystick_delay);
        Joystick.button(5, 0);
        //delay(joystick_delay);
      }
    }else if (nose_p2 == 0){//am pult EINgeschaltet
    //Serial.println("in strobe-p2-schleife");
    
      if (nose_f <2){// im Flugzeug ausgeschaltet
        // im Flugzeug schalten
        Joystick.button(6, 1);
        delay(joystick_delay);
        Joystick.button(6, 0);
        //delay(joystick_delay);
      }

    } else { 
      
      if (nose_f >1){// im Flugzeug ausgeschaltet
        // im Flugzeug schalten
        Joystick.button(5, 1);
        delay(joystick_delay);
        Joystick.button(5, 0);
      }else if (nose_f <1){// im Flugzeug ausgeschaltet
        // im Flugzeug schalten
        Joystick.button(6, 1);
        delay(joystick_delay);
        Joystick.button(6, 0);
        //delay(joystick_delay);
      }
    }
  }

  currentMillis = millis();

  // Prüfen, ob 1 Sekunde (1000 ms) vergangen ist
  if (currentMillis - lastFlapsUpdate >= 1500) {
    lastFlapsUpdate = currentMillis;
    
    // Funktion hier aufrufen
    checkFlaps();
    checkGear();
    checkSpeedbrake();
  }

  currentMillis = millis();
  if (currentMillis - lastSpeedbrakeUpdate >= 300) {
    lastSpeedbrakeUpdate = currentMillis;
    
    // Funktion hier aufrufen
   
   
  }

  //checkSpeedbrake();

  //_____________________________________________
  // A U S G A B E   L C D   A M   P U L T
  // wenn Timer dafür abgelaufen, nicht jedes mal...
  // alle 200ms ?
  //FlightSim.update();  // GANZ WICHTIG – sonst kommen keine Daten!
  umsch_fcu_com=digitalRead(10);
  umsch_fcu=digitalRead(9);
  umsch_com=digitalRead(11);
  
  FlightSim.update();
  com1a_f1 = com1_left_mhz.read();
  com1a_f2 = com1_left_khz.read();
  com2a_f1 = com2_left_mhz.read();
  com2a_f2 = com2_left_khz.read();
  com1s_f1 = com1_stby_mhz.read();
  com1s_f2 = com1_stby_khz.read();
  com2s_f1 = com2_stby_mhz.read();
  com2s_f2 = com2_stby_khz.read();
  

  //Serial.print(" FCU/COM.  FCU.  COM.  "); Serial.print(umsch_fcu_com); Serial.print("  "); Serial.print(umsch_fcu); Serial.print("  "); Serial.println(umsch_com);

  if (currentMillis - lastLCDUpdate >= 100) {
    lastLCDUpdate = currentMillis;

    
    //HDG

    lcd.setCursor(0, 0);
    lcd.print("H");
    lcd.setCursor(2, 0);
    buffer[3];
    sprintf(buffer, "%0*d", 3, hdg_f);  // * holt sich die Breite aus dem Argument davor
    lcd.print(buffer);
    lcd.setCursor(5,0);
    if (hdg_managed_f==1){
      lcd.print("*");
    } else {
      lcd.print(" ");
    }

    //Altitude
    lcd.setCursor(7, 0);
    lcd.print("A");
    lcd.setCursor(9, 0);
    buffer[5];
    sprintf(buffer, "%0*d", 5, alt_f);  // * holt sich die Breite aus dem Argument davor
    lcd.print(buffer);
    lcd.setCursor(14,0);
    if (alt_managed_f==1){
      lcd.print("*");
    } else {
      lcd.print(" ");
    }



    //Gear
    lcd.setCursor(17,0);
    //if (gear_p == 0){
    if (gear_f == 1){  
      lcd.print("ooo");
    } else {
      lcd.print("^^^");
    }
    
    //Thrust
    lcd.setCursor(16,1);
    if (a0joy == 1023){
      //lcd.setCursor(16,1);
      lcd.print("TOGA");
    } else if (a0joy == 740){
      //lcd.setCursor(16,1);
      lcd.print("Flex");
    }else if (a0joy == 520){
      //lcd.setCursor(16,1);
      lcd.print("  CL");
    }else {
      //lcd.setCursor(16,1);
      lcd.print("    ");
      lcd.setCursor(18,1);
      buffer[2];
      sprintf(buffer, "%0*d", 2,a0joy*100/520);  // * holt sich die Breite aus dem Argument davor
      lcd.print(buffer);
      //lcd.print(a0joy*100/520);
    }

    //Speed
    lcd.setCursor(0, 1);
    lcd.print("S");
    lcd.setCursor(2, 1);
    buffer[3];
    sprintf(buffer, "%0*d", 3, spd_f);  // * holt sich die Breite aus dem Argument davor
    lcd.print(buffer);
    lcd.setCursor(5,1);
    if (spd_managed_f==1){
      lcd.print("*");
    } else {
      lcd.print(" ");
    }
    //Serial.print("vertical armed, dashed, speed  ");Serial.print(vs_armed_f);Serial.print("  ");Serial.print(vs_dashed_f);Serial.print("  ");Serial.println(vs_f);
    //VertikalSpeed
    lcd.setCursor(7, 1);
    lcd.print("VS");
    lcd.setCursor(10, 1);
    if (vs_armed_f>2){

      
      buffer[5];
      sprintf(buffer, "%0*d", 5, vs_f);  // * holt sich die Breite aus dem Argument davor
      lcd.print(buffer);
      if (vs_f>0){
        lcd.setCursor(10, 1);
        lcd.print("+");
      }
    } else {
      lcd.print("-----");
    }
    //int vs_armed_f;
    //int vs_dashed_f;

    
    
    //Com1
    lcd.setCursor(0, 2);
    lcd.print("C1");
    lcd.setCursor(6, 2);
    lcd.print(".");
    if (selectedline !=3){
      lcd.setCursor(15, 2);
      lcd.print(".");
    }
    lcd.setCursor(3, 2);
    lcd.print(com1a_f1);
    lcd.setCursor(7, 2);
    lcd.print(com1a_f2);
    lcd.setCursor(12, 2);
    lcd.print(com1s_f1);
    lcd.setCursor(16, 2);


    buffer[3];
    sprintf(buffer, "%0*d", 3,com1s_f2);  // * holt sich die Breite aus dem Argument davor
    lcd.print(buffer);

    //lcd.print(com1s_f2);

    //Com2
    lcd.setCursor(0, 3);
    lcd.print("C2");
    lcd.setCursor(6, 3);
    lcd.print(".");
    if (selectedline !=4){
      lcd.setCursor(15, 3);
      lcd.print(".");
    }
    lcd.setCursor(3, 3);
    lcd.print(com2a_f1);
    lcd.setCursor(7, 3);
    lcd.print(com2a_f2);
    lcd.setCursor(12, 3);
    lcd.print(com2s_f1);
    lcd.setCursor(16, 3);
    lcd.print(com2s_f2);
    
    
    //selectedline
    if (selectedline ==1){
      lcd.setCursor(1, 0);
      lcd.print(">");
      lcd.setCursor(8, 0);
      lcd.print(">");
      lcd.setCursor(1, 1);
      lcd.print(" ");
      lcd.setCursor(9, 1);
      lcd.print(" ");
      lcd.setCursor(11, 2);
      lcd.print(" ");
      //lcd.setCursor(15, 2);
      //lcd.print(" ");
      lcd.setCursor(11, 3);
      lcd.print(" ");
      //lcd.setCursor(15, 3);
      //lcd.print(" ");


    } else if (selectedline ==2){
      lcd.setCursor(1, 0);
      lcd.print(" ");
      lcd.setCursor(8, 0);
      lcd.print(" ");
      lcd.setCursor(1, 1);
      lcd.print(">");
      lcd.setCursor(9, 1);
      lcd.print(">");
      lcd.setCursor(11, 2);
      lcd.print(" ");
      //lcd.setCursor(15, 2);
      //lcd.print(" ");
      lcd.setCursor(11, 3);
      lcd.print(" ");
      //lcd.setCursor(15, 3);
      //lcd.print(" ");

    } else if (selectedline ==3){
      lcd.setCursor(1, 0);
      lcd.print(" ");
      lcd.setCursor(8, 0);
      lcd.print(" ");
      lcd.setCursor(1, 1);
      lcd.print(" ");
      lcd.setCursor(9, 1);
      lcd.print(" ");
      lcd.setCursor(11, 2);
      lcd.print(">");
      lcd.setCursor(15, 2);
      lcd.print(">");
      lcd.setCursor(11, 3);
      lcd.print(" ");
      //lcd.setCursor(15, 3);
      //lcd.print(" ");
    } else {//if (selectedline ==4){
      lcd.setCursor(1, 0);
      lcd.print(" ");
      lcd.setCursor(8, 0);
      lcd.print(" ");
      lcd.setCursor(1, 1);
      lcd.print(" ");
      lcd.setCursor(9, 1);
      lcd.print(" ");
      lcd.setCursor(11, 2);
      lcd.print(" ");
      //lcd.setCursor(15, 2);
      //lcd.print(" ");
      lcd.setCursor(11, 3);
      lcd.print(">");
      lcd.setCursor(15, 3);
      lcd.print(">");
    }
  }

  //_____________________________________________
  // A U S G A B E   M O N I T O R 


  //Serial.print("Beacon Flugzeug: "); Serial.print(beacon_f); Serial.print("   Beacon Pult: "); Serial.print(beacon_p); 
  //Serial.print("    A0joy "); Serial.print(a0joy); //Serial.print(" "); Serial.print(log(a0));Serial.print(" "); Serial.print(log10(a0));Serial.print(" "); Serial.print(log2(a0));Serial.print(" "); Serial.print(yjoy);
  //Serial.print("Strobe Flugzeug: "); Serial.print(strobe_f); Serial.print("   Strobe Pult 1: "); Serial.print(strobe_p1); Serial.print("   Strobe Pult 2: "); Serial.print(strobe_p2); 
 // Serial.print("nose Flugzeug: "); Serial.print(nose_f); Serial.print("   nose Pult 1: "); Serial.print(nose_p1); Serial.print("   nose Pult 2: "); Serial.print(nose_p2); 
  

  //Serial.println("]");

  // Debug-Ausgabe aller digitalen Eingänge aus buttonPins[]
  //Serial.print("Pins: ");
  if (debugmode==1){
    for (unsigned int i = 0; i < sizeof(buttonPins)/sizeof(buttonPins[0]); i++) {
      int val = digitalRead(buttonPins[i]);
      Serial.print("P");
      Serial.print(buttonPins[i]);
      Serial.print(" ");
      Serial.print(val);
      if (i < sizeof(buttonPins)/sizeof(buttonPins[0]) - 1) {
        Serial.print(", ");
      }
    }
    Serial.println();
  }


  delay(10);
}
