// Rotary Table controller 
// Written for Salk Institute, Azim lab
// Written by Mark Stambaugh (UCSD) based on code by Masakazu Igarashi & others
// Last edited 2020-03-13

/*This program controls one DC motor with attached rotary encoder, with position 
  given by a MATLAB script via Serial Port. The actual control is done via PID controller, 
  and the position of the motor is fed back to MATLAB. 
*/

#define VERSION "2.00"
#define EDIT_DATE "2020/06/25"

#define BAUDRATE 115200 //must match value in MATLAB serial declaration

//encoder parameters
#define COUNTS_PER_ROTATION 2048

#define PID_OUTPUT_MIN -80
#define PID_OUTPUT_MAX 80
#define PID_KP 0.500 * 360.0/COUNTS_PER_ROTATION // * 360/2048 due to switch from encoder counts to degrees
#define PID_KI 0.000 // KI != 0 causes drifting over time
#define PID_KD 0.003 * 360.0/COUNTS_PER_ROTATION // * 360/2048 due to switch from encoder counts to degrees

//pin connections to encoder
#define ENCODER_A_PIN 2
#define ENCODER_B_PIN 3
#define ENCODER_I_PIN 18

//pin connections to motor driver
#define MOTOR_SLEEP_PIN 13
#define MOTOR_PHASE_PIN 12
#define MOTOR_ENABLE_PIN 11

//pin to enable the solenoid valve
#define REWARD_TRIGGER_PIN 8

//pin to the Opto (unknown device --MPS-20200219)
#define OPTO_PIN 7  //TODO 

#include <Arduino.h>
#include "motor_controller.h"

//encoder overhead functions. Moved to top level for visibility in setup
//ISRs must be top level (global) anyway
volatile int encoder_count, a_val, b_val;
void encoder_a_changed_ISR();
void encoder_b_changed_ISR();
void initEncoder();

void command_tree();
void run_trial();
void deliverReward(int duration_ms);

//parameters to be set by matlab before trial begins
float zero_position_angle_deg = 30;
float target_angle_deg = 60;
float window_play_deg = 20;
float start_angle_deg = 100;
int hold_time_ms = 500;
int trial_duration_ms = 7000;
bool reward_anyway = 0;
int reward_duration_ms = 120;
int data_res_ms = 4;
int recording_preamble_ms = 500;
int recording_postamble_ms = 500;
bool opto_enabled = 0;
int opto_pre_trial_ms = 350;
int opto_post_trial_ms = 350;

//instantiate motor
TI_DRV8838_with_encoder motorController(MOTOR_SLEEP_PIN, MOTOR_PHASE_PIN, MOTOR_ENABLE_PIN, &encoder_count, COUNTS_PER_ROTATION);

void setup() {

  while(1){
    pinMode(ENCODER_A_PIN , OUTPUT);
    pinMode(ENCODER_B_PIN , OUTPUT);
    pinMode(ENCODER_I_PIN , OUTPUT);
    pinMode(MOTOR_SLEEP_PIN , OUTPUT);
    pinMode(MOTOR_PHASE_PIN , OUTPUT);
    pinMode(MOTOR_ENABLE_PIN , OUTPUT);
    pinMode(REWARD_TRIGGER_PIN , OUTPUT);
    pinMode(OPTO_PIN , OUTPUT);

    digitalWrite(ENCODER_A_PIN , HIGH);
    delay(500);
    digitalWrite(ENCODER_A_PIN , LOW);

    digitalWrite(ENCODER_B_PIN , HIGH);
    delay(500);
    digitalWrite(ENCODER_B_PIN , LOW);
    
    digitalWrite(ENCODER_I_PIN , HIGH);
    delay(500);
    digitalWrite(ENCODER_I_PIN , LOW);

    digitalWrite(MOTOR_SLEEP_PIN , HIGH);
    delay(500);
    digitalWrite(MOTOR_SLEEP_PIN , LOW);

    digitalWrite(MOTOR_PHASE_PIN , HIGH);
    delay(500);
    digitalWrite(MOTOR_PHASE_PIN , LOW);

    digitalWrite(MOTOR_ENABLE_PIN , HIGH);
    delay(500);
    digitalWrite(MOTOR_ENABLE_PIN , LOW);

    digitalWrite(REWARD_TRIGGER_PIN , HIGH);
    delay(500);
    digitalWrite(REWARD_TRIGGER_PIN , LOW);

    digitalWrite(OPTO_PIN , HIGH);
    delay(500);
    digitalWrite(OPTO_PIN , LOW); 

    digitalWrite(MOTOR_SLEEP_PIN, HIGH);
    digitalWrite(MOTOR_ENABLE_PIN, HIGH);
    digitalWrite(MOTOR_PHASE_PIN, HIGH);
    delay(500);
    digitalWrite(MOTOR_PHASE_PIN, LOW);
    delay(500);
    digitalWrite(MOTOR_SLEEP_PIN, LOW);
    digitalWrite(MOTOR_ENABLE_PIN, LOW);
  }
  

  //initial output so MATLAB can identify the Arduino controlling the rotary table
  //the Arduino resets every time the Serial port is opened
  Serial.begin(BAUDRATE);
  Serial.setTimeout(10);
  
  Serial.println(F("Project: rotary table"));
  Serial.print(F("Version "));
  Serial.println(VERSION);
  Serial.print(F("Last edited "));
  Serial.println(EDIT_DATE);
  Serial.println(F("Code by Mark Stambaugh, Masakazu Igarashi"));

  //initialize pin directions & values
  pinMode(REWARD_TRIGGER_PIN, OUTPUT);
  pinMode(OPTO_PIN, OUTPUT);
  digitalWrite(REWARD_TRIGGER_PIN, LOW);
  digitalWrite(OPTO_PIN, LOW);

  initEncoder(); // initialize encoder
  motorController.silentOperation();//If this is not called, the motor will produce audible tones at 490Hz
  motorController.turnToObstacle(REVERSE, 2000); // turn table slowly until physical obstacle is hit
  motorController.setAngleToZero(); // zero the rotary counter in the current position

  Serial.println(F("'h' for help"));
}

void loop() {
  //wait until a command arrives from the PC
  while(!Serial.available());
  
  //parse the command and take action
  command_tree();

  //clear residual data from serial buffer
  while(Serial.available())
    Serial.read();
}

void command_tree(){
  char cmd, arg, arg2;
  float dummy_angle, dummy_speed;
  bool dummy_bool;
  int dummy_int;

  delay(10); //10ms delay to make sure all data is in
  cmd = Serial.read();
  switch(cmd){
    case 's': //Set parameters
    case 'S': 
      arg = Serial.read();
      switch(arg){
        case 'z': //zero position angle
        case 'Z':
          zero_position_angle_deg = Serial.parseFloat();
          Serial.print(F("ZERO ANGLE (deg): "));
          Serial.println(zero_position_angle_deg, 1);
          break;
        case 't': //target angle
        case 'T':
          target_angle_deg = Serial.parseFloat();
          Serial.print(F("TARGET ANGLE (deg): "));
          Serial.println(target_angle_deg, 1);
          break;
        case 'w': //window width on either side of target angle
        case 'W':
          window_play_deg = Serial.parseFloat();
          Serial.print(F("WINDOW ANGLE (deg): "));
          Serial.println(window_play_deg, 1);
          break;
        case 's': //start angle
        case 'S': 
          start_angle_deg = Serial.parseFloat();
          Serial.print(F("START ANGLE (deg): "));
          Serial.println(start_angle_deg, 1);
          break;
        case 'h': //hold time
        case 'H':
          hold_time_ms = Serial.parseInt();
          Serial.print(F("HOLD TIME (ms): "));
          Serial.println(hold_time_ms);
          break;
        case 'd': //trial duration
        case 'D': 
          trial_duration_ms = Serial.parseInt();
          Serial.print(F("TRIAL DURATION (ms): "));
          Serial.println(trial_duration_ms);
          break;
        case 'p': //recording preamble
        case 'P': 
          recording_preamble_ms = Serial.parseInt();
          Serial.print(F("PREAMBLE (ms): "));
          Serial.println(recording_preamble_ms);
          break;
        case 'o': //recording postamble
        case 'O': 
          recording_preamble_ms = Serial.parseInt();
          Serial.print(F("PREAMBLE (ms): "));
          Serial.println(recording_preamble_ms);
          break;
        case 'a': //reward anyway
        case 'A': 
          reward_anyway = Serial.parseInt();
          Serial.print(F("REWARD ANYWAY: "));
          Serial.println(reward_anyway);
          break;
        case 'r': //reward duration
        case 'R': 
          reward_duration_ms = Serial.parseInt();
          Serial.print(F("REWARD DURATION (ms): "));
          Serial.println(reward_duration_ms);
          break;
        case 'm':
        case 'M':
          data_res_ms = Serial.parseInt();
          Serial.print(F("DATA RESOLUTION (ms): "));
          Serial.println(data_res_ms);
          break;
        default:
          Serial.print(F("ERROR: "));
          Serial.print(cmd);
          Serial.println(arg);
      }
      break;
    case 'g': //get current parameters
    case 'G': 
      Serial.print(F("ZERO ANGLE (deg): "));
      Serial.println(zero_position_angle_deg, 1);
      Serial.print(F("TARGET ANGLE (deg): "));
      Serial.println(target_angle_deg, 1);
      Serial.print(F("WINDOW ANGLE (deg): "));
      Serial.println(window_play_deg, 1);
      Serial.print(F("START ANGLE (deg): "));
      Serial.println(start_angle_deg, 1);
      Serial.print(F("HOLD TIME (ms): "));
      Serial.println(hold_time_ms);
      Serial.print(F("TRIAL DURATION (ms): "));
      Serial.println(trial_duration_ms);
      Serial.print(F("PREAMBLE (ms): "));
      Serial.println(recording_preamble_ms);
      Serial.print(F("POSTAMBLE (ms): "));
      Serial.println(recording_postamble_ms);
      Serial.print(F("REWARD ANYWAY: "));
      Serial.println(reward_anyway);
      Serial.print(F("REWARD DURATION (ms): "));
      Serial.println(reward_duration_ms);
      Serial.print(F("DATA RESOLUTION (ms): "));
      Serial.println(data_res_ms);
      Serial.print(F("OPTO ENABLED: "));
      Serial.println(opto_enabled);
      Serial.print(F("OPTO PRE-TRIAL LENGTH (ms): "));
      Serial.println(opto_pre_trial_ms);
      Serial.print(F("OPTO POST-TRIAL LENGTH (ms): "));
      Serial.println(opto_post_trial_ms);
      Serial.println();
      break;
    
    case 'h': //print help menu
    case 'H': 
      Serial.println(F("Commands are NOT case-sensitive"));
      Serial.println(F("'sz #' Set Zero angle (deg)"));
      Serial.println(F("'st #' Set Target angle (deg)"));
      Serial.println(F("'sw #' Set Window half-width (deg)"));
      Serial.println(F("'ss #' Set Start angle (deg)"));
      Serial.println(F("'sh #' Set Hold time (ms)"));
      Serial.println(F("'sd #' Set trial Duration (ms)"));
      Serial.println(F("'sp #' Set recording preamble (ms)"));
      Serial.println(F("'so #' Set recording postamble (ms)"));
      Serial.println(F("'sa #' Set reward Anyway (0/1)"));
      Serial.println(F("'sr #' Set Reward duration (ms)"));
      Serial.println(F("'sm #' Set data resolution (Ms)"));
      Serial.println(F("'g' Get current parameters"));
      Serial.println(F("'r' Run trial with current parameters"));
      Serial.println(F("'oe #' Opto Enable (0/1)"));
      Serial.println(F("'op #' Opto pre-trial length (ms)"));
      Serial.println(F("'oo #' Opto post-trial length (ms)"));
      Serial.println(F("'dbf/r' Debug - turn motor until Obstacle is hit, Forward/Reverse"));
      Serial.println(F("'da #' Debug - turn motor to Angle"));
      Serial.println(F("'dr #' Debug - deliver Reward (ms)"));
      Serial.println(F("'do #' debug - turn opto on (ms)"));
      Serial.println(F("'dz' Debug - set encoder Zero in current position"));
      Serial.println(F("'dp' Debug - read current rotor Position in counts, deg"));
      Serial.println(F("'dt' Debug - set motor default Timeout (ms)"));
      Serial.println(F("'dmf/r # #' Debug - turn Motor Forward/Reverse at # duty cycle for # ms"));
      Serial.println(F("'h' Help (print this menu)"));
      Serial.println();
      break;
      
    case 'r': //run trial
    case 'R': 
      run_trial();
      break;
    case 'o': //opto enable/disable
    case 'O':
      arg = Serial.read();
      switch(arg) {
        case 'e':
        case 'E':
          opto_enabled = Serial.parseInt();
          Serial.print(F("OPTO ENABLED: "));
          Serial.println(opto_enabled);
          break;
        case 'p':
        case 'P':
          opto_pre_trial_ms = Serial.parseInt();
          Serial.print(F("OPTO PRE-TRIAL LENGTH (ms): "));
          Serial.println(opto_pre_trial_ms);
          break;
        case 'o':
        case 'O':
          opto_post_trial_ms = Serial.parseInt();
          Serial.print(F("OPTO POST-TRIAL LENGTH (ms): "));
          Serial.println(opto_post_trial_ms);
          break;
        default:
          Serial.print(F("ERROR: "));
          Serial.print(cmd);
          Serial.println(arg);
      }
      break;
    
    case 'd': //debug menu
    case 'D': 
      arg = Serial.read();
        switch(arg){
          case 'b': //turn motor until it hits an obstacle
          case 'B': 
            arg2 = Serial.read();
            if(arg2 == 'f' || arg2 == 'F'){
              dummy_bool = motorController.turnToObstacle(FORWARD);
            }
            else if(arg2 == 'r' || arg2 == 'R'){
              dummy_bool = motorController.turnToObstacle(REVERSE);
            }
            else{
              Serial.print(F("INVALID DIRECTION: "));
              Serial.println(arg2);
            } 
            if(dummy_bool == 1){
              dummy_angle = motorController.getCurrentAngleDeg();
              Serial.print(F("TURNED TO OBSTACLE AT "));
              Serial.println(dummy_angle, 1);
            }
            else{
              Serial.println(F("NO OBSTACLE FOUND"));  
            }
            break;
          case 'a': //turn motor to specified angle
          case 'A': 
            dummy_angle = Serial.parseFloat();
            motorController.turnToAngleDeg(dummy_angle);
            dummy_angle = motorController.getCurrentAngleDeg();
            Serial.print(F("TURNED TO ANGLE: "));
            Serial.println(dummy_angle, 1);
            break; 
          case 'r': //issue reward immediately
          case 'R': 
            dummy_int = Serial.parseInt();
            deliverReward(dummy_int);
            Serial.println(F("REWARD DELIVERED"));
            break;
          case 'o': //turn opto on
          case 'O': 
            dummy_int = Serial.parseInt();
            digitalWrite(OPTO_PIN, HIGH);
            delay(dummy_int);
            digitalWrite(OPTO_PIN, LOW);
            Serial.println(F("OPTO TRIGGERED"));
            break;
          case 'z': //zero the encoder count in the current position
          case 'Z': 
            motorController.setAngleToZero();
            Serial.println(F("ENCODER ZEROED"));
            break;
          case 'p': //get current motor position
          case 'P': 
            dummy_angle = motorController.getCurrentAngleDeg();
            Serial.print(F("ENCODER: "));
            Serial.println(encoder_count);
            Serial.print(F("ANGLE (deg): "));
            Serial.println(dummy_angle, 1);
            break;
          case 't':
          case 'T':
            motorController.default_timeout_ms = Serial.parseInt();
            Serial.print(F("MOTOR DEFAULT TIMEOUT (ms): "));
            Serial.print(motorController.default_timeout_ms);
            break;
          case 'm':
          case 'M':
            arg2 = Serial.read();
            if(arg2 == 'f' || arg2 == 'F'){
              dummy_speed = Serial.parseFloat();
              dummy_int = Serial.parseInt();
              motorController.forward(dummy_speed);
              delay(dummy_int);
              motorController.release();
            }
            else if(arg2 == 'r' || arg2 == 'R'){
              dummy_speed = Serial.parseFloat();
              dummy_int = Serial.parseInt();
              motorController.reverse(dummy_speed);
              delay(dummy_int);
              motorController.release();
            }
            else{
              Serial.print(F("INVALID DIRECTION: "));
              Serial.println(arg2);
            } 
            break;
          default:
            Serial.print(F("ERROR: "));
            Serial.print(cmd);
            Serial.println(arg);
        }
      break;
    default:
    Serial.print(F("ERROR: "));
    Serial.println(byte(cmd));
  }
}

void run_trial(){
  float target_angle_min_deg = target_angle_deg - window_play_deg;
  float target_angle_max_deg = target_angle_deg + window_play_deg;
  
  //initialize trial variables
  float angle_deg = 0;
  bool outcome = 0;
  bool in_window = 0;
  bool opto_on = 0;
  unsigned long time_in_window_ms = 0;
  unsigned long trial_time_ms = 0;
  unsigned long time_in_window_start_ms;
  unsigned long trial_start_ms;
  unsigned long trial_complete_ms;


  //signal matlab to start of trial
  Serial.println(F("Trial start."));
  trial_start_ms = millis();

  //report preamble & turn on opto if enabled
  while(trial_time_ms <= recording_preamble_ms){
    trial_time_ms = millis() - trial_start_ms;

    if (opto_enabled & !opto_on & (trial_time_ms > recording_preamble_ms - opto_pre_trial_ms)){
      digitalWrite(OPTO_PIN, HIGH);
      opto_on = 1;
      Serial.println(F("Opto on"));
    }
    
    Serial.print(trial_time_ms);
    Serial.print(' ');
    Serial.println(motorController.getCurrentAngleDeg(),1);
    while(millis() < (trial_start_ms + trial_time_ms + data_res_ms)){};
  }
  
  //move motor to start angle
  motorController.turnToAngleDeg(zero_position_angle_deg + start_angle_deg);
  
  
  time_in_window_start_ms =  millis();
  //take trial start time, wait for duration to elapse or rotary counter in proper range for long enough
  //reporting position all the while
  while(trial_time_ms <= trial_duration_ms + recording_preamble_ms){
    trial_time_ms = millis() - trial_start_ms;
    
    angle_deg = motorController.getCurrentAngleDeg();
    
    //check position
    in_window = (angle_deg <= target_angle_max_deg) && (angle_deg >= target_angle_min_deg);
    
    if(!in_window){
      time_in_window_start_ms = millis();
    } 
    else{
      time_in_window_ms = millis() - time_in_window_start_ms;
      //check for success critera
      if(time_in_window_ms > hold_time_ms){
        outcome = 1;
        break;
      }
    }
    
    //report time and position to matlab
    Serial.print(trial_time_ms);
    Serial.print(' ');
    Serial.println(motorController.getCurrentAngleDeg(),1);

    //wait until current millis increments to avoid spamming matlab with redundant data
    while(millis() < (trial_start_ms + trial_time_ms + data_res_ms)){};
  }
  
  //trial is complete. Deliver reward and continue reporting throughout postamble
  trial_complete_ms = millis();
  
  if(outcome == 1)
    Serial.println(F("Trial success!"));
  else
    Serial.println(F("Trial failure."));
  
  //reward mouse based on performance & override
  if(outcome || reward_anyway){
    digitalWrite(REWARD_TRIGGER_PIN, HIGH);
  }
  
  //continue reporting throughout postamble and retract reward
  while(millis() < trial_complete_ms + recording_postamble_ms){
    trial_time_ms = millis() - trial_start_ms;
    //report time and position to matlab
    Serial.print(trial_time_ms);
    Serial.print(' ');
    Serial.println(motorController.getCurrentAngleDeg(),1);

    if(millis() > trial_complete_ms + reward_duration_ms){
      digitalWrite(REWARD_TRIGGER_PIN, LOW);
    }

    if (opto_on & (millis() > trial_complete_ms + opto_post_trial_ms)){
      digitalWrite(OPTO_PIN, LOW);
      opto_on = 0;
      Serial.println(F("Opto off"));
    }
    
    //wait until current millis increments to avoid spamming matlab with redundant data
    while(millis() < (trial_start_ms + trial_time_ms + data_res_ms)){};
  }
  //report to matlab and turn off reward if it is still on
  Serial.println(F("Trial end."));
  if(millis() > trial_complete_ms + reward_duration_ms){
    digitalWrite(REWARD_TRIGGER_PIN, LOW);
  }
}

//Reward for success
void deliverReward(int duration_ms){
  digitalWrite(REWARD_TRIGGER_PIN, HIGH);                        
  delay(duration_ms);
  digitalWrite(REWARD_TRIGGER_PIN, LOW);    
}

void encoder_a_changed_ISR() {
  a_val = digitalRead(ENCODER_A_PIN);
  //if a_val == b_val, then a is "trailing" b; motor is moving CCW
  encoder_count += (a_val == b_val) ? -1 : 1;
}
void encoder_b_changed_ISR() {
  b_val = digitalRead(ENCODER_B_PIN);
  //if b_val == a_val, then b is "trailing" a; motor is moving CW
  encoder_count += (b_val == a_val) ? 1 : -1;
}
void initEncoder() {
  //set the data direction of the encoder pins of the motor on the mcu
  pinMode(ENCODER_A_PIN, INPUT);
  pinMode(ENCODER_B_PIN, INPUT);

  //initialize state variables
  a_val = digitalRead(ENCODER_A_PIN);
  b_val = digitalRead(ENCODER_B_PIN);
  encoder_count = 0;
  
  //register interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoder_a_changed_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), encoder_b_changed_ISR, CHANGE); 
}
