/* -----------------------------------------------------------------------------
  - Original Project: Body Movement Crawling robot
  - Original Author:  panerqiang@sunfounder.com
  - Date:  2015/1/27

  - Remix Project by Wilmar
  - Date: 2018/09/25
  - Change log: 
      > Add more movement and combine demo and controlled move in one
      > Using PCA9685 16 channel Servo controller for Arduino for motor driver
      > Add NodeMCU for Wifi Controller as AP
   -----------------------------------------------------------------------------
  - Overview
  - This project was written for the Crawling robot desigened by Sunfounder.
    This version of the robot has 4 legs, and each leg is driven by 3 servos.
  This robot is driven by a Ardunio Nano Board with an expansion Board.
  We recommend that you view the product documentation before using.
  - Request
  - This project requires some library files, which you can find in the head of
    this file. Make sure you have installed these files.
  - How to
  - Before use,you must to adjust the robot,in order to make it more accurate.
    - Adjustment operation
    1.uncomment ADJUST, make and run
    2.comment ADJUST, uncomment VERIFY
    3.measure real sites and set to real_site[4][3], make and run
    4.comment VERIFY, make and run
  The document describes in detail how to operate.

/* Includes ------------------------------------------------------------------*/

#include <Adafruit_PWMServoDriver.h>
#include <FlexiTimer2.h>//to set a timer to manage all servos
#include <SerialCommand.h>
SerialCommand SCmd;   // The demo SerialCommand object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

/* Servos --------------------------------------------------------------------*/
//define 12 servos for 4 legs

//Servo servo[4][3];
//define servos' ports
const int servo_pin[4][3] = { {0, 1, 2}, {4, 5, 6}, {8, 9, 10}, {12, 13, 14} };
/* Size of the robot ---------------------------------------------------------*/
const float length_a = 55;
const float length_b = 77.5;
const float length_c = 27.5;
const float length_side = 71;
const float z_absolute = -28;
/* Constants for movement ----------------------------------------------------*/
const float z_default = -50, z_up = -30, z_boot = z_absolute;
const float x_default = 62, x_offset = 0;
const float y_start = 0, y_step = 40;
const float y_default = x_default;
/* variables for movement ----------------------------------------------------*/
volatile float site_now[4][3];    //real-time coordinates of the end of each leg
volatile float site_expect[4][3]; //expected coordinates of the end of each leg
float temp_speed[4][3];   //each axis' speed, needs to be recalculated before each movement
float move_speed =1.4;     //movement speed
float speed_multiple = 1; //movement speed multiple
const float spot_turn_speed = 4;
const float leg_move_speed = 8;
const float body_move_speed = 3;
const float stand_seat_speed = 1;
volatile int rest_counter;      //+1/0.02s, for automatic rest
//functions' parameter
const float KEEP = 255;
//define PI for calculation
const float pi = 3.1415926;
/* Constants for turn --------------------------------------------------------*/
//temp length
const float temp_a = sqrt(pow(2 * x_default + length_side, 2) + pow(y_step, 2));
const float temp_b = 2 * (y_start + y_step) + length_side;
const float temp_c = sqrt(pow(2 * x_default + length_side, 2) + pow(2 * y_start + y_step + length_side, 2));
const float temp_alpha = acos((pow(temp_a, 2) + pow(temp_b, 2) - pow(temp_c, 2)) / 2 / temp_a / temp_b);
//site for turn
const float turn_x1 = (temp_a - length_side) / 2;
const float turn_y1 = y_start + y_step / 2;
const float turn_x0 = turn_x1 - temp_b * cos(temp_alpha);
const float turn_y0 = temp_b * sin(temp_alpha) - turn_y1 - length_side;
/* ---------------------------------------------------------------------------*/
boolean Demo_mode=true;
String lastComm="";
int ledPulse =0;

int FRFoot = 0;
int FRElbow = 0;
int FRShdr=0; 
int FLFoot = 0;
int FLElbow = 0;
int FLShdr=0; 
int RRFoot = 0;
int RRElbow = 0;
int RRShdr=0; 
int RLFoot = 0;
int RLElbow = 0; 
int RLShdr=0;

/*
  - setup function
   ---------------------------------------------------------------------------*/
void setup()
{
  //start serial for debug
  
  Serial.begin(115200);
  Serial.println("Robot starts initialization");
  
  pinMode(LED_BUILTIN, OUTPUT);
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  SCmd.addCommand("w", action_cmd);

  SCmd.setDefaultHandler(unrecognized);

  //initialize default parameter
  set_site(0, x_default - x_offset, y_start + y_step, z_boot);
  set_site(1, x_default - x_offset, y_start + y_step, z_boot);
  set_site(2, x_default + x_offset, y_start, z_boot);
  set_site(3, x_default + x_offset, y_start, z_boot);
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      site_now[i][j] = site_expect[i][j];
    }
  }
  //start servo service
  FlexiTimer2::set(20, servo_service);
  FlexiTimer2::start();
  Serial.println("Servo service started");
  //initialize servos
 // servo_attach();
  Serial.println("Servos initialized");
  Serial.println("Robot initialization Complete");
  sit();
  b_init();
}


// you can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. its not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 60;   // 60 Hz
  pulselength /= 4096;  // 12 bits of resolution
  pulse *= 1000000;  // convert to us
  pulse /= pulselength;
  pwm.setPWM(n, 0, pulse);
}
/*
  - loop function
   ---------------------------------------------------------------------------*/
void loop()
{
  //-----------led blink status
    if (ledPulse <= 500){
      digitalWrite(LED_BUILTIN, LOW); 
    }
    if (ledPulse > 1000){
      digitalWrite(LED_BUILTIN, HIGH); 
    }
    if (ledPulse >= 1500){
      ledPulse = 0;
    }
    ledPulse++;
  //-------------------
    
    SCmd.readSerial();
    if (lastComm=="FWD"){
      step_forward(1);
    }
    if (lastComm=="BWD"){
      step_back(1);
    }
    if (lastComm=="LFT"){
      turn_left(1);
    }
    if (lastComm=="RGT"){
      turn_right(1);
    }
    Serial.println(lastComm);
    // turn_right(1); //test
}

// w 0 2: body init
// w 0 1: stand
// w 0 0: sit
// w 1 x: forward x step
// w 2 x: back x step
// w 3 x: right turn x step
// w 4 x: left turn x step
// w 5 x: hand shake x times
// w 6 x: hand wave x times
// w 7 : dance
// w 8 x: head up x times
// w 9 x: head down x times
// w 10 x: body right x times
// w 11 x: body left x times
// w 12: Body init pose
// w 13: body up
// w 14: body down
// w 15: reset pos
// w 16: body twist right
// w 17: body twist left

#define W_STAND_SIT    0
#define W_FORWARD      1
#define W_BACKWARD     2
#define W_LEFT         3
#define W_RIGHT        4
#define W_SHAKE        5
#define W_WAVE         6
#define W_DANCE        7
#define W_HEAD_UP      8
#define W_HEAD_DOWN    9
#define W_B_RIGHT      10
#define W_B_LEFT       11
#define W_B_INIT       12
#define W_HIGHER       13
#define W_LOWER        14
#define W_SET          15
#define W_TW_R         16
#define W_TW_L         17

void action_cmd(void)
{
  char *arg;
  int action_mode, n_step;
  Serial.println("Action:");
  arg = SCmd.next();
  action_mode = atoi(arg);
  arg = SCmd.next();
  n_step = atoi(arg);
  Demo_mode=false;

  switch (action_mode)
  {
    case W_FORWARD:
      Serial.println("Step forward");
      lastComm="FWD";
      if (!is_stand())
        stand();
        step_forward(n_step);
      break;
      
    case W_BACKWARD:
      Serial.println("Step back");
      lastComm="BWD";
      if (!is_stand())
        stand();
      step_back(n_step);
      break;
      
    case W_LEFT:
      Serial.println("Turn left");
      lastComm="LFT";
      if (!is_stand())
        stand();
      turn_left(n_step);
      break;
      
    case W_RIGHT:
      Serial.println("Turn right");
      lastComm="RGT";
      if (!is_stand())
        stand();
      turn_right(n_step);
      break;
      
    case W_STAND_SIT:
      Serial.println("1:up,0:dn");
      lastComm="";
      if (n_step)
        stand();
      else
        sit();
      break;
      
    case W_SHAKE:
      Serial.println("Hand shake");
      lastComm="";
      hand_shake(n_step);
      break;

    case W_WAVE:
      Serial.println("Hand wave");
      lastComm="";
      hand_wave(n_step);
      break;

    case W_DANCE:
      Serial.println("Lets rock baby");
      lastComm="";
      //body_dance(n_step);
      body_dance(10);
      break;

    case W_SET:
      Serial.println("Higher");
      FLElbow =0;FRElbow =0;
      RLElbow =0;RRElbow =0;
      FLFoot =0;FRFoot =0;
      RLFoot =0;RRFoot =0;
      FLShdr=0; FRShdr=0;
      RLShdr=0;RRShdr=0;
      stand();
      break;  

    case W_HIGHER:
      Serial.println("Higher");
      FLElbow -= 4;FRElbow -= 4;
      RLElbow -= 4;RRElbow -= 4;
      FLFoot += 4;FRFoot += 4;
      RLFoot += 4;RRFoot += 4;
      stand();
      break;

    case W_LOWER:
      Serial.println("Lower");
      FLElbow += 4;FRElbow += 4;
      RLElbow += 4;RRElbow += 4;
      FLFoot -= 4;FRFoot -= 4;
      RLFoot -= 4;RRFoot -= 4;
      stand();
      break;

    case W_HEAD_UP:
      Serial.println("Head up");
      FLElbow -= 4;FRElbow -= 4;
      RLElbow += 4;RRElbow += 4;
      FLFoot += 4;FRFoot += 4;
      RLFoot -= 4;RRFoot -= 4;
      stand();
      break;

    case W_HEAD_DOWN:
      Serial.println("Head down");
      FLElbow += 4;FRElbow += 4;
      RLElbow -= 4;RRElbow -= 4;
      FLFoot -= 4;FRFoot -= 4;
      RLFoot += 4;RRFoot += 4;
      stand();
      break;

    case W_B_RIGHT:
      Serial.println("body right");
      if (!is_stand()) stand();
      FLElbow -= 4;FRElbow += 4;
      RLElbow -= 4;RRElbow += 4;
      FLFoot += 4;FRFoot -= 4;
      RLFoot += 4;RRFoot -= 4;
      stand();
      break; 

    case W_B_LEFT:
      Serial.println("body left");
      if (!is_stand()) stand();
      FLElbow += 4;FRElbow -= 4;
      RLElbow += 4;RRElbow -= 4;
      FLFoot -= 4;FRFoot += 4;
      RLFoot -= 4;RRFoot += 4;
      stand();
      break; 

    case W_B_INIT:
      Serial.println("Body init");
      lastComm="";
      sit();
      b_init();
      FLElbow =0;FRElbow =0;
      RLElbow =0;RRElbow =0;
      FLFoot =0;FRFoot =0;
      RLFoot =0;RRFoot =0;
      FLShdr=0; FRShdr=0;
      RLShdr=0;RRShdr=0;
      stand();
      break;

    case W_TW_R:
      Serial.println("Body twist right");
      FLShdr-= 4; FRShdr+= 4;
      RLShdr+= 4;RRShdr-= 4;
      stand();
      break;
      
    case W_TW_L:
      Serial.println("Body twist left");
      FLShdr+= 4; FRShdr-= 4;
      RLShdr-= 4;RRShdr+= 4;
      stand();
      break;

    default:
      Serial.println("Error");
      break;
  }
}

// This gets set as the default handler, and gets called when no other command matches.
void unrecognized(const char *command) {
  Serial.println("What?");
}

/*
  - is_stand
   ---------------------------------------------------------------------------*/
bool is_stand(void)
{
  if (site_now[0][2] == z_default)
    return true;
  else
    return false;
}

/*
  - sit
  - blocking function
   ---------------------------------------------------------------------------*/
void sit(void)
{
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++)
  {
    set_site(leg, KEEP, KEEP, z_boot);
  }
  wait_all_reach();
}

/*
  - stand
  - blocking function
   ---------------------------------------------------------------------------*/
void stand(void)
{
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++)
  {
    set_site(leg, KEEP, KEEP, z_default);
  }
  wait_all_reach();
}


/*
  - Body init
  - blocking function
   ---------------------------------------------------------------------------*/
void b_init(void)
{
  //stand();
  set_site(0, x_default, y_default, z_default);
  set_site(1, x_default, y_default, z_default);
  set_site(2, x_default, y_default, z_default);
  set_site(3, x_default, y_default, z_default);
  wait_all_reach();
}

/*
  - spot turn to left
  - blocking function
  - parameter step steps wanted to turn
   ---------------------------------------------------------------------------*/
void turn_left(unsigned int step)
{
  move_speed = spot_turn_speed;
  while (step-- > 0)
  {
    if (site_now[3][1] == y_start)
    {
      //leg 3&1 move
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start, z_up);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 0&2 move
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_up);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      set_site(2, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

/*
  - spot turn to right
  - blocking function
  - parameter step steps wanted to turn
   ---------------------------------------------------------------------------*/
void turn_right(unsigned int step)
{
  move_speed = spot_turn_speed;
  while (step-- > 0)
  {
    if (site_now[2][1] == y_start)
    {
      //leg 2&0 move
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_up);
      set_site(1, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 1&3 move
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

/*
  - go forward
  - blocking function
  - parameter step steps wanted to go
   ---------------------------------------------------------------------------*/
void step_forward(unsigned int step)
{
  move_speed = leg_move_speed;
  while (step-- > 0)
  {
    if (site_now[2][1] == y_start)
    {
      //leg 2&1 move
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
//      leg 0&3 move
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

/*
  - go back
  - blocking function
  - parameter step steps wanted to go
   ---------------------------------------------------------------------------*/
void step_back(unsigned int step)
{
  move_speed = leg_move_speed;
  while (step-- > 0)
  {
    if (site_now[3][1] == y_start)
    {
      //leg 3&0 move
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(1, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 1&2 move
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

// add by RegisHsu

void body_left(int i)
{
  set_site(0, site_now[0][0] + i, KEEP, KEEP);
  set_site(1, site_now[1][0] + i, KEEP, KEEP);
  set_site(2, site_now[2][0] - i, KEEP, KEEP);
  set_site(3, site_now[3][0] - i, KEEP, KEEP);
  wait_all_reach();
}

void body_right(int i)
{
  set_site(0, site_now[0][0] - i, KEEP, KEEP);
  set_site(1, site_now[1][0] - i, KEEP, KEEP);
  set_site(2, site_now[2][0] + i, KEEP, KEEP);
  set_site(3, site_now[3][0] + i, KEEP, KEEP);
  wait_all_reach();
}

void hand_wave(int i)
{
  float x_tmp;
  float y_tmp;
  float z_tmp;
  move_speed = 1;
  if (site_now[3][1] == y_start)
  {
    body_right(15);
    x_tmp = site_now[2][0];
    y_tmp = site_now[2][1];
    z_tmp = site_now[2][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(2, turn_x1, turn_y1, 50);
      wait_all_reach();
      set_site(2, turn_x0, turn_y0, 50);
      wait_all_reach();
    }
    set_site(2, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_left(15);
  }
  else
  {
    body_left(15);
    x_tmp = site_now[0][0];
    y_tmp = site_now[0][1];
    z_tmp = site_now[0][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(0, turn_x1, turn_y1, 50);
      wait_all_reach();
      set_site(0, turn_x0, turn_y0, 50);
      wait_all_reach();
    }
    set_site(0, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_right(15);
  }
}

void hand_shake(int i)
{
  float x_tmp;
  float y_tmp;
  float z_tmp;
  move_speed = 1;
  if (site_now[3][1] == y_start)
  {
    body_right(15);
    x_tmp = site_now[2][0];
    y_tmp = site_now[2][1];
    z_tmp = site_now[2][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(2, x_default - 30, y_start + 2 * y_step, 55);
      wait_all_reach();
      set_site(2, x_default - 30, y_start + 2 * y_step, 10);
      wait_all_reach();
    }
    set_site(2, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_left(15);
  }
  else
  {
    body_left(15);
    x_tmp = site_now[0][0];
    y_tmp = site_now[0][1];
    z_tmp = site_now[0][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(0, x_default - 30, y_start + 2 * y_step, 55);
      wait_all_reach();
      set_site(0, x_default - 30, y_start + 2 * y_step, 10);
      wait_all_reach();
    }
    set_site(0, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_right(15);
  }
}


void head_up(int i)
{
  set_site(0, KEEP, KEEP, site_now[0][2] - i);
  set_site(1, KEEP, KEEP, site_now[1][2] + i);
  set_site(2, KEEP, KEEP, site_now[2][2] - i);
  set_site(3, KEEP, KEEP, site_now[3][2] + i);
  wait_all_reach();
}

void head_down(int i)
{
  set_site(0, KEEP, KEEP, site_now[0][2] + i);
  set_site(1, KEEP, KEEP, site_now[1][2] - i);
  set_site(2, KEEP, KEEP, site_now[2][2] + i);
  set_site(3, KEEP, KEEP, site_now[3][2] - i);
  wait_all_reach();
}

void body_dance(int i)
{
  float x_tmp;
  float y_tmp;
  float z_tmp;
  float body_dance_speed = 2;
  sit();
  move_speed = 1;
  set_site(0, x_default, y_default, KEEP);
  set_site(1, x_default, y_default, KEEP);
  set_site(2, x_default, y_default, KEEP);
  set_site(3, x_default, y_default, KEEP);
  wait_all_reach();
  stand();
  set_site(0, x_default, y_default, z_default - 20);
  set_site(1, x_default, y_default, z_default - 20);
  set_site(2, x_default, y_default, z_default - 20);
  set_site(3, x_default, y_default, z_default - 20);
  wait_all_reach();
  move_speed = body_dance_speed;
  head_up(30);
  for (int j = 0; j < i; j++)
  {
    if (j > i / 4)
      move_speed = body_dance_speed * 2;
    if (j > i / 2)
      move_speed = body_dance_speed * 3;
    set_site(0, KEEP, y_default - 20, KEEP);
    set_site(1, KEEP, y_default + 20, KEEP);
    set_site(2, KEEP, y_default - 20, KEEP);
    set_site(3, KEEP, y_default + 20, KEEP);
    wait_all_reach();
    set_site(0, KEEP, y_default + 20, KEEP);
    set_site(1, KEEP, y_default - 20, KEEP);
    set_site(2, KEEP, y_default + 20, KEEP);
    set_site(3, KEEP, y_default - 20, KEEP);
    wait_all_reach();
  }
  move_speed = body_dance_speed;
  head_down(30);
  b_init();
}

/*
  - microservos service /timer interrupt function/50Hz
  - when set site expected,this function move the end point to it in a straight line
  - temp_speed[4][3] should be set before set expect site,it make sure the end point
   move in a straight line,and decide move speed.
   ---------------------------------------------------------------------------*/
void servo_service(void)
{
  sei();
  static float alpha, beta, gamma;

  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      if (abs(site_now[i][j] - site_expect[i][j]) >= abs(temp_speed[i][j]))
        site_now[i][j] += temp_speed[i][j];
      else
        site_now[i][j] = site_expect[i][j];
    }

    cartesian_to_polar(alpha, beta, gamma, site_now[i][0], site_now[i][1], site_now[i][2]);
    polar_to_servo(i, alpha, beta, gamma);
  }

  rest_counter++;
}

/*
  - set one of end points' expect site
  - this founction will set temp_speed[4][3] at same time
  - non - blocking function
   ---------------------------------------------------------------------------*/
void set_site(int leg, float x, float y, float z)
{
  float length_x = 0, length_y = 0, length_z = 0;

  if (x != KEEP)
    length_x = x - site_now[leg][0];
  if (y != KEEP)
    length_y = y - site_now[leg][1];
  if (z != KEEP)
    length_z = z - site_now[leg][2];

  float length = sqrt(pow(length_x, 2) + pow(length_y, 2) + pow(length_z, 2));

  temp_speed[leg][0] = length_x / length * move_speed * speed_multiple;
  temp_speed[leg][1] = length_y / length * move_speed * speed_multiple;
  temp_speed[leg][2] = length_z / length * move_speed * speed_multiple;

  if (x != KEEP)
    site_expect[leg][0] = x;
  if (y != KEEP)
    site_expect[leg][1] = y;
  if (z != KEEP)
    site_expect[leg][2] = z;
}

/*
  - wait one of end points move to expect site
  - blocking function
   ---------------------------------------------------------------------------*/
void wait_reach(int leg)
{
  while (1)
    if (site_now[leg][0] == site_expect[leg][0])
      if (site_now[leg][1] == site_expect[leg][1])
        if (site_now[leg][2] == site_expect[leg][2])
          break;
}

/*
  - wait all of end points move to expect site
  - blocking function
   ---------------------------------------------------------------------------*/
void wait_all_reach(void)
{
  for (int i = 0; i < 4; i++)
    wait_reach(i);
}

/*
  - trans site from cartesian to polar
  - mathematical model 2/2
   ---------------------------------------------------------------------------*/
void cartesian_to_polar(volatile float &alpha, volatile float &beta, volatile float &gamma, volatile float x, volatile float y, volatile float z)
{
  //calculate w-z degree
  float v, w;
  w = (x >= 0 ? 1 : -1) * (sqrt(pow(x, 2) + pow(y, 2)));
  v = w - length_c;
  alpha = atan2(z, v) + acos((pow(length_a, 2) - pow(length_b, 2) + pow(v, 2) + pow(z, 2)) / 2 / length_a / sqrt(pow(v, 2) + pow(z, 2)));
  beta = acos((pow(length_a, 2) + pow(length_b, 2) - pow(v, 2) - pow(z, 2)) / 2 / length_a / length_b);
  //calculate x-y-z degree
  gamma = (w >= 0) ? atan2(y, x) : atan2(-y, -x);
  //trans degree pi->180
  alpha = alpha / pi * 180;
  beta = beta / pi * 180;
  gamma = gamma / pi * 180;
}

/*
  - trans site from polar to microservos
  - mathematical model map to fact
  - the errors saved in eeprom will be add
   ---------------------------------------------------------------------------*/
void polar_to_servo(int leg, float alpha, float beta, float gamma)
{
  if (leg == 0) //Front Right
  {
    alpha = 85 - alpha - FRElbow; //elbow (- is up)
    beta = beta + 40 - FRFoot; //foot (- is up)
    gamma += 115 - FRShdr;    // shoulder (- is left)
  }
  else if (leg == 1) //Rear Right
  {
    alpha += 90 + RRElbow; //elbow (+ is up)
    beta = 115 - beta + RRFoot; //foot (+ is up)
    gamma = 115 - gamma + RRShdr; // shoulder (+ is left)
  }
  else if (leg == 2) //Front Left
  {
    alpha += 75 + FLElbow; //elbow (+ is up)
    beta = 140 - beta + FLFoot; //foot (+ is up)
    gamma = 115 - gamma + FLShdr;// shoulder (+ is left)
  }
  else if (leg == 3) // Rear Left
  {
    alpha = 90 - alpha - RLElbow; //elbow (- is up)
    beta = beta + 50 - RLFoot; //foot; (- is up)
    gamma += 100 - RLShdr;// shoulder (- is left)
  }
  int AL = ((850/180)*alpha);if (AL > 580) AL=580;if (AL < 125) AL=125;pwm.setPWM(servo_pin[leg][0],0,AL);
  int BE = ((850/180)*beta);if (BE > 580) BE=580;if (BE < 125) BE=125;pwm.setPWM(servo_pin[leg][1],0,BE);
  int GA = ((580/180)*gamma);if (GA > 580) GA=580;if (GA < 125) GA=125;pwm.setPWM(servo_pin[leg][2],0,GA);
}


