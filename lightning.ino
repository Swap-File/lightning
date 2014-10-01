#include <ArduinoNunchuk.h>
#include <Wire.h>
#include <MovingAverage.h>

/*************************************************
 * Public Constants
 *************************************************/

#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978

//dpad masks
#define DPAD_LEFT B00001000
#define DPAD_RIGHT B0000010
#define DPAD_UP B00000001
#define DPAD_DOWN B00000100
#define DPAD_UP_LEFT B00001001
#define DPAD_DOWN_RIGHT B00000110
#define DPAD_UP_RIGHT B00000011
#define DPAD_DOWN_LEFT B00001100
#define DPAD_DEADZONE B00010000

//stick data
unsigned long nunchuk_polling_timer=0;
byte dpad_previous = 0x00; 
byte dpad = 0x00;
unsigned int xtilt;
unsigned int ytilt;
int ytilt_auto=0;
byte ytilt_one_way=0;
unsigned long ytilt_one_way_timer=0;

#define BUTTONDELAY 50  //milliseconds it takes for a button press to count
#define DOUBLETAPTIME 500  //milliseconds between double taps for it to count
//detecting doubletaps from the nunchuck
byte zc_doubletap_status = 0;
unsigned long zc_doubletap_time;
boolean cButtonDelayed = false;
boolean zButtonDelayed = false;
boolean cButtonLast = false;
boolean zButtonLast = false;
unsigned long cButtonTimer;
unsigned long zButtonTimer;
unsigned long dpadTimer;
//displaying gesture data on lcd
boolean beat_completed=false;
boolean beat_completed_raw = false;
boolean beat_completed_auto =false;

int bpm_period = 1000;  //period in milliseconds
unsigned long bpm_starting_time=0; //keeps track of when the bmp is sampled
byte auto_pump_mode = 0; //0 is off, 1 is on, and higher than that is turbo modes
boolean auto_pump = false;   //autopump mode, do not set this variable, it is automatically set based on auto_pump_mode at the end of a beat
byte auto_pump_multiplier=0;  //autopump mode, do not set this variable, it is automatically set based on auto_pump_mode at the end of a beat

boolean input_processed = true;


int start_mode_request = 0;
int start_mode = 0;
int num_lit_request = 0;
int num_lit = 0;
int starting_light = 0;
int light_mode = 0;
int light_mode_request = 0;
boolean enabled[4];

int red = 5;
int green = 6;
int blue = 9;
int yellow = 10;
int red_button = 2;
int green_button = 3;
int blue_button = 7;
int yellow_button = 8;
//input  2 3 7 8

int previous_x_absolute = 0;
int previous_z_absolute = 0;
int x_filtered = 0;
int z_filtered = 0;

int xz_magnitude =0;
int xz_filtered_magnitude =0;
int xz_angle = 0;
int xz_filtered_angle = 0;
int previous_xz_magnitude =0;
int previous_xz_filtered_magnitude =0;
int previous_xz_angle = 0;
int previous_xz_filtered_angle = 0;


boolean accel_enabled = false;
//tone on 11
ArduinoNunchuk nunchuk = ArduinoNunchuk();
MovingAverage xfilter = MovingAverage();
MovingAverage yfilter = MovingAverage();

boolean spin[4];

void setup() {


  Serial.begin(115200);  //debug
  Serial.println("Nunchuck");
  nunchuk.init();

  Serial.println("pinmode");
  pinMode(red,OUTPUT);
  pinMode(green,OUTPUT);
  pinMode(blue,OUTPUT);
  pinMode(yellow,OUTPUT);
  pinMode(red_button,INPUT_PULLUP);
  pinMode(green_button,INPUT_PULLUP);
  pinMode(blue_button,INPUT_PULLUP);
  pinMode(yellow_button,INPUT_PULLUP);

  analogWrite(red, 255);
  analogWrite(green, 255);
  analogWrite(blue, 255);
  analogWrite(yellow, 255);
}

void loop() {


  if (millis()-nunchuk_polling_timer > 20){
    nunchuk.update(); //read data from nunchuk
    nunchuk_polling_timer=millis();
  }
  nunchuckparse(); 


  if (nunchuk.zButton == 0 && nunchuk.cButton == 0 ){
    input_processed=false;
  }

  if (cButtonDelayed  ){

    Serial.println("C");
    switch (dpad){
    case DPAD_LEFT:
      light_mode_request = 0;
      num_lit_request = 0;
      num_lit = 0;
      light_mode = 0;
      accel_enabled = true;
      break;
    case DPAD_RIGHT:

      break;
    case DPAD_UP:
      if( input_processed == false && num_lit_request < 4){
        num_lit_request = num_lit_request +1;
      }
      
      if (accel_enabled == true){
        accel_enabled = false;
        num_lit_request = 1;
        num_lit = 1;
             light_mode_request = 0;
        light_mode = 0;
      }


      break;
    case DPAD_DOWN:
      if( input_processed == false && num_lit_request > 0){
        num_lit_request = num_lit_request - 1;
      }

      if (accel_enabled == true){
        accel_enabled = false;
        light_mode_request = 0;
        light_mode = 0;
      }

      break;
    case DPAD_UP_LEFT:
      start_mode_request = 4;
      break;
    case DPAD_DOWN_RIGHT:
      start_mode_request = 3;
      break;
    case DPAD_UP_RIGHT:
      start_mode_request = 2;
      break;
    case DPAD_DOWN_LEFT:
      start_mode_request =1;
      break;
    }


    //generate one change
    if((dpad & 0x0F) != 0x00){
      input_processed = true;
    }
    else {
      input_processed = false;
    }

  }

  //basic setttings and span gestures
  if (zButtonDelayed){
    switch (dpad){
    case DPAD_LEFT: //arm chest switch


      break;
    case DPAD_RIGHT: //chest chest seitch

      break;
    case DPAD_UP:
      if( input_processed == false){
        auto_pump_mode=0;
      }
      break;
    case DPAD_DOWN:
      if( input_processed == false){
        auto_pump_mode++;
      }
      break;
    case DPAD_UP_LEFT:
      light_mode_request = 4;
      break;
    case DPAD_DOWN_RIGHT:
      light_mode_request = 3;
      break;
    case DPAD_UP_RIGHT:
      light_mode_request = 2;
      break;
    case DPAD_DOWN_LEFT:
      light_mode_request = 1;
      break;
    }

    //generate one change
    if((dpad & 0x0F) != 0x00){
      input_processed = true;
    }
    else {
      input_processed = false;
    }
  }

  boolean output[4];
  output[0] = false;
  output[1] = false;
  output[2] = false;
  output[3] = false;

  for (int i = 0; i < 4; i++){
    int calculated_led = (starting_light+ i) % 4  ;

    if ((calculated_led == 0) && (enabled[i] == true)){
      output[0]= true;
    }

    if ((calculated_led == 1 )&& (enabled[i] == true)){
      output[1] = true;
    }

    if ((calculated_led == 2 )&&( enabled[i] == true)){
      output[2]= true;
    }

    if ((calculated_led == 3) && (enabled[i] == true)){
      output[3]= true;
    }
  }


  if (output[0] || (spin[0] && accel_enabled)){
    analogWrite(red, ytilt);
  }
  else{
    analogWrite(red, 255);
  }

  if (output[1] || (spin[1]&& accel_enabled)){
    analogWrite(blue, ytilt);
  }
  else{
    analogWrite(blue, 255);
  }
  if (output[2] || (spin[2]&& accel_enabled)){
    analogWrite(yellow, ytilt);
  }
  else{
    analogWrite(yellow, 255);
  }

  if (output[3] || (spin[3]&& accel_enabled)){
    analogWrite(green, ytilt);
  }
  else{
    analogWrite(green, 255);
  }

}


void nunchuckparse(){

  byte dpadtemp =0x00;
  if(nunchuk.analogMagnitude > 40){
    if (nunchuk.analogAngle < 10 && nunchuk.analogAngle > -10){
      dpadtemp = DPAD_LEFT;
    }
    else if (nunchuk.analogAngle < 55 && nunchuk.analogAngle > 35){
      dpadtemp =  DPAD_DOWN_LEFT;
    }
    else if (nunchuk.analogAngle < -35 && nunchuk.analogAngle > -55){
      dpadtemp =  DPAD_UP_LEFT;
    }
    else if (nunchuk.analogAngle < -80 && nunchuk.analogAngle > -100){
      dpadtemp =  DPAD_UP;
    }
    else if (nunchuk.analogAngle < 100 && nunchuk.analogAngle > 80){
      dpadtemp =  DPAD_DOWN;
    }
    else if (nunchuk.analogAngle < 145 && nunchuk.analogAngle > 125){
      dpadtemp =  DPAD_DOWN_RIGHT;
    }
    else if (nunchuk.analogAngle < -125 && nunchuk.analogAngle > -145){
      dpadtemp =  DPAD_UP_RIGHT;
    }
    else if (nunchuk.analogAngle < -170 || nunchuk.analogAngle > 170){
      dpadtemp =  DPAD_RIGHT;
    }
    else{
      dpadtemp = DPAD_DEADZONE;
    }
  }

  //dpad noise removal / delay code
  if (dpadtemp == 0x00 || dpadtemp !=dpad_previous){
    dpadTimer = millis();
    dpad = 0x00;
  }
  if (millis() - dpadTimer > BUTTONDELAY ){
    dpad = dpadtemp;
  }
  dpad_previous = dpadtemp;

  //nunchuck unplugged code
  if(nunchuk.pluggedin == false){
    // effect_mode = 0;
    // output_mode = 0;  
    dpad = 0x00;
  }

  //double tap code
  if ( nunchuk.zButton == 1 &&  nunchuk.cButton == 1){
    if (zc_doubletap_status == 0){
      zc_doubletap_time = millis();
      zc_doubletap_status =1;
    }
    else if (zc_doubletap_status == 2){
      if (millis() - zc_doubletap_time < DOUBLETAPTIME){
        zc_doubletap_status = 3;
      }
    }
  }
  else if ( nunchuk.zButton == 0 && nunchuk.cButton == 0){
    if (millis() - zc_doubletap_time > DOUBLETAPTIME){
      zc_doubletap_status = 0;
    }
    if (zc_doubletap_status == 1){
      zc_doubletap_status =2;
    }
  }

  //z button noise removal / delay code
  if( nunchuk.zButton && zButtonLast == false || nunchuk.cButton){
    zButtonTimer=millis();
  }

  if (nunchuk.zButton && (millis() - zButtonTimer > BUTTONDELAY&& nunchuk.cButton == false)){
    zButtonDelayed = true;
  }
  else{
    zButtonDelayed = false;
  }
  zButtonLast = nunchuk.zButton;

  //c button noise removal / delay code
  if( nunchuk.cButton && cButtonLast == false || nunchuk.zButton){
    cButtonTimer=millis();
  }

  if (nunchuk.cButton && (millis() - cButtonTimer > BUTTONDELAY && nunchuk.zButton == false)){
    cButtonDelayed = true;
  }
  else{
    cButtonDelayed = false;
  }
  cButtonLast = nunchuk.cButton;



  spin[0]=false;
  spin[1]=false;
  spin[2]=false;
  spin[3]=false;


  if (nunchuk.accelX < 300){
    spin[0]=true;
  }
  else if (nunchuk.accelX > 800){
    spin[2]=true;
  }
  else if (nunchuk.accelZ < 600){
    spin[1]=true;
  } 
  else if (nunchuk.accelZ > 800){
    spin[3]=true;
  }




  xtilt =  xfilter.process(constrain(nunchuk.accelX, 350, 650));
  xtilt= map(xtilt, 350, 650,0, 254); 

  ytilt = yfilter.process(constrain(nunchuk.accelY, 500, 600));
  ytilt = map(ytilt, 500, 600,254, 0);


  //calculate auto value
  ytilt_auto = map((millis()-bpm_starting_time )% (bpm_period >> auto_pump_multiplier), 0, (bpm_period >> auto_pump_multiplier), 0,254);
  ytilt_auto = ytilt_auto * 4;
  if( ytilt_auto < 256){
    ytilt_auto = 0;
  }
  else{
    ytilt_auto = ytilt_auto -256;
    if( ytilt_auto > 254){
      ytilt_auto = 254;
    }
  }

  //indicator for raw
  if (ytilt == 0){
    beat_completed_raw=false;
  }
  else if(ytilt == 254){
    beat_completed_raw=true;
  }
  //indicator for auto
  if (ytilt_auto == 0){
    beat_completed_auto=false;
  }
  else if(ytilt_auto == 254){
    beat_completed_auto=true;
  }

  //reassign ytilt to raw or auto based on autopump mode
  if (auto_pump == true){
    ytilt = ytilt_auto;
  }

  if (ytilt == 0){
    if (ytilt_one_way != 0){
      ytilt_one_way_timer = millis();
    }
    ytilt_one_way = 0;

    //run oncon peak of pump
    if(beat_completed == true){

      switch(auto_pump_mode){
      case 0:
        auto_pump = false;
        break;
      case 1:
        auto_pump = true;
        auto_pump_multiplier = 0;
        break;
      case 2:
        auto_pump_multiplier = 1;
        break;
      case 3:
        auto_pump_multiplier = 2;
        break;
      case 4:
        auto_pump_multiplier = 3;
        break;
      case 5:
        auto_pump_multiplier = 4;
        break;
      default:
        auto_pump_multiplier = 4;
        auto_pump_mode = 5;
      }

      if(auto_pump == true){
        //only stay in turbo modes while holding button
        if(auto_pump_multiplier > 0 && (dpad & 0x0F) == 0x00){
          auto_pump_mode=1;
          auto_pump_multiplier = 0;
        }
       if (num_lit == 7){
          auto_pump_mode=0;
          auto_pump = false;
        }
        //avoid time travel into the future when entering auto mode
        //if the manual entry beat comes in a few milliseconds late when transitioning it would look ugly
        if (bpm_starting_time+bpm_period > millis()){
          bpm_starting_time=millis();
        }
        else{
          bpm_starting_time=bpm_starting_time+bpm_period;
        }
      }
      else{
        //60 is minimum bpm
        bpm_period = constrain(bpm_period,200,1000);

        //filter the bpm
        bpm_period = bpm_period * .6 + (millis()-bpm_starting_time) *.4;
        bpm_starting_time= millis(); 
      }



    }
    beat_completed=false;
  }
  else if (ytilt == 254 ){
    //run once on peak of pump
    if(beat_completed== false){
      //beats++;

      start_mode = start_mode_request;
      num_lit = num_lit_request;

      light_mode = light_mode_request;


      if (start_mode==1 ){
        starting_light = starting_light + 1;
        if (starting_light>3){
          starting_light = 0;
        }
      }
      if (start_mode == 3){
        if (starting_light==0){
          starting_light = 1;
        }
        else if (starting_light==1){
          starting_light = 3;
        }
        else if (starting_light==3){
          starting_light =2;
        }
        else if (starting_light==2){
          starting_light = 0;
        }
      }

      if (start_mode==2 ){
        starting_light = starting_light - 1;
        if (starting_light<0){
          starting_light = 3;
        }
      }

      if (start_mode == 4){
        if (starting_light==0){
          starting_light = 2;
        }
        else if (starting_light==2){
          starting_light = 3;
        }
        else if (starting_light==3){
          starting_light =1;
        }
        else if (starting_light==1){
          starting_light = 0;
        }
      }
      if (light_mode == 3 || light_mode == 4){

        if (num_lit == 0){
          enabled[0] = false; 
          enabled[1] = false; 
          enabled[2] = false; 
          enabled[3] = false; 
        }
        if (num_lit == 1){
          enabled[0] = true; 
          enabled[1] = false; 
          enabled[2] = false; 
          enabled[3] = false; 
        }
        if (num_lit == 2){
          enabled[0] = true; 
          enabled[1] = false; 
          enabled[2] = true; 
          enabled[3] = false; 
        }
        if (num_lit == 3){
          enabled[0] = true; 
          enabled[1] = true; 
          enabled[2] = true; 
          enabled[3] = false; 
        }
        if (num_lit == 4){
          enabled[0] = true; 
          enabled[1] = true; 
          enabled[2] = true; 
          enabled[3] = true; 
        }

      }
      else{
        if (num_lit == 0){
          enabled[0] = false; 
          enabled[1] = false; 
          enabled[2] = false; 
          enabled[3] = false; 
        }
        if (num_lit == 1){
          enabled[0] = true; 
          enabled[1] = false; 
          enabled[2] = false; 
          enabled[3] = false; 
        }
        if (num_lit == 2){
          enabled[0] = true; 
          enabled[1] = true; 
          enabled[2] = false; 
          enabled[3] = false; 
        }
        if (num_lit == 3){
          enabled[0] = true; 
          enabled[1] = true; 
          enabled[2] = true; 
          enabled[3] = false; 
        }
        if (num_lit == 4){
          enabled[0] = true; 
          enabled[1] = true; 
          enabled[2] = true; 
          enabled[3] = true; 
        }
      }


      Serial.println("Beat!");
      Serial.println(num_lit);
      Serial.println(light_mode);
      Serial.println(starting_light);
      
      
      ytilt_one_way_timer = millis();
    }
    beat_completed = true;
  }

  ytilt_one_way = min(ytilt_one_way,ytilt);  

}


























