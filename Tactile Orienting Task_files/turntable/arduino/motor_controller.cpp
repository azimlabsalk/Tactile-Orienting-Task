// motor controller with encoder feedback. Brushed motor driven by DRV8838
// includes a PID controller
// Written for Salk Institute, Azim lab
// Created by by Masakazu Igarashi & others
// Last edited by Mark Stambaugh (UCSD) on 2020-02-29

#include "motor_controller.h"

TI_DRV8838::TI_DRV8838(const unsigned int sleepPin, const unsigned int phasePin, const unsigned int enablePin) {
  m_sleepPin = sleepPin;
  m_phasePin = phasePin;
  m_enablePin = enablePin;

  //initialize
  pinMode(m_sleepPin, OUTPUT);
  pinMode(m_phasePin, OUTPUT);
  pinMode(m_enablePin, OUTPUT);

  //pull sleepPin high to take it out of sleep mode
  digitalWrite(m_sleepPin, HIGH);
  digitalWrite(m_phasePin, LOW);
  digitalWrite(m_enablePin, LOW);
}

//returns the current speed (0-100%)
double TI_DRV8838::getCurrentSpeed() {
  return m_speed;
}

//move forward at max speed
void TI_DRV8838::forward() {
  digitalWrite(m_phasePin, LOW);
  digitalWrite(m_sleepPin, HIGH);
  digitalWrite(m_enablePin, HIGH);
}

//move forward at a set speed (0-100%)
void TI_DRV8838::forward(double speed) {
  setSpeed(speed);
  digitalWrite(m_phasePin, LOW);
  digitalWrite(m_sleepPin, HIGH);
}

//reverse at max speed
void TI_DRV8838::reverse() {
  digitalWrite(m_phasePin, HIGH);
  digitalWrite(m_enablePin, HIGH);
  digitalWrite(m_sleepPin, HIGH);
}

//reverse at a set speed (0-100%)
void TI_DRV8838::reverse(double speed) {
  setSpeed(speed);
  digitalWrite(m_phasePin, HIGH);
  digitalWrite(m_sleepPin, HIGH);
}

//brake
void TI_DRV8838::brake() {
  digitalWrite(m_enablePin, LOW);
}

//stop controlling the motor
void TI_DRV8838::release() {
  digitalWrite(m_sleepPin, LOW);
}

//change the PWM frequency (default 490Hz) out of audible range
void TI_DRV8838::silentOperation() {
  switch (digitalPinToTimer(m_enablePin)) {
    //arduino.h sets the prescaler of all the timers to 64 (initially pwm freq=490Hz)
    //clock 16Mhz (arduino mega 2560)
    //clock / prescaler(1 + TOP) => clock of pwm
    //if you want higher frequencies for pwm set the prescaler to 1
    case TIMER1A:
    case TIMER1B:
      TCCR1B = (TCCR1B & B11111000) | B00000001;
      break;
    case TIMER2A:
    case TIMER2B:
      TCCR2B = (TCCR2B & B11111000) | B00000001;
      break;
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    case TIMER3A:
    case TIMER3B:
    case TIMER3C:
      TCCR3B = (TCCR3B & B11111000) | B00000001;
      break;
    case TIMER4A:
    case TIMER4B:
    case TIMER4C:
    case TIMER4D:
      TCCR4B = (TCCR4B & B11111000) | B00000001;
      break;
    case TIMER5A:
    case TIMER5B:
    case TIMER5C:
      TCCR5B = (TCCR5B & B11111000) | B00000001;
      break;
#endif
    default:
      break;
  }
}

//set speed to value in percent
void TI_DRV8838::setSpeed(double speed) {
  //make sure speed is in bounds
  if (speed >= 100.0) {
    //set it high to go fast
    digitalWrite(m_enablePin, HIGH);
    return;
  }
  if (speed <= 0.0) {
    //set it low to brake
    digitalWrite(m_enablePin, LOW);
    return;
  }
  int intSpeed = (int) ((speed / 100.0) * 255);
  analogWrite(m_enablePin, intSpeed);
  m_speed = speed;
}


//subclass which includes PID and pointer to previously-declared encoder
TI_DRV8838_with_encoder::TI_DRV8838_with_encoder(const unsigned int sleepPin, const unsigned int phasePin, const unsigned int enablePin, volatile int *encoder_count, int counts_per_rotation)
: TI_DRV8838::TI_DRV8838(sleepPin, phasePin, enablePin),
  m_PID(&m_PID_input_deg, &m_PID_setpoint_deg, &m_PID_output_dc, PID_OUTPUT_MIN_DEFAULT, PID_OUTPUT_MAX_DEFAULT, PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT)
{
  m_encoder_count = encoder_count;
  m_counts_per_rotation = counts_per_rotation;

  //interval for PID recalculation
  m_PID.setTimeStep(PID_TIMESTEP_MS_DEFAULT);
}

//set PID parameters if desired different from default
void TI_DRV8838_with_encoder::set_PID_params(double Kp, double Ki, double Kd, double output_min, double output_max, unsigned long timestep, double threshold_deg){
  m_PID.setGains(Kp, Ki, Kd);
  m_PID.setOutputRange(output_min, output_max);
  m_PID.setTimeStep(timestep);
}

//zero the encoder at the current position
void TI_DRV8838_with_encoder::setAngleToZero(){
  *m_encoder_count = 0;
  m_angle = 0.0;
}

//Input encoder count, output angle in degrees. Takes 4usec
float TI_DRV8838_with_encoder::angleCountToDeg(int count){
  return(float(*m_encoder_count)*360.0/m_counts_per_rotation);
}

//Input angle degrees, output encoder count. Takes 48usec
int TI_DRV8838_with_encoder::angleDegToCount(float angle_deg){
  return int(angle_deg*m_counts_per_rotation/360.0);
}

//calculate current angle and return 
double TI_DRV8838_with_encoder::getCurrentAngleDeg(){
  m_angle = angleCountToDeg(*m_encoder_count);
  return m_angle;
}

//turns at low speed until movement stops
bool TI_DRV8838_with_encoder::turnToObstacle(bool turn_direction, long unsigned timeout_ms){
  int start_count = *m_encoder_count;
  int prev_encoder_count = *m_encoder_count+1;
  long unsigned time_start_ms = millis();

  //rotate at low speed until motor stops due to physical barrier
  if(turn_direction == FORWARD)
    forward(6);
  else
    reverse(6);
    
  while(*m_encoder_count != prev_encoder_count){  
    //release motor and return failure if timeout is reached
    if (millis() - time_start_ms > timeout_ms){
      release();
      return 0;
    }
    prev_encoder_count = *m_encoder_count;
    //Release and return error if full rotation made without encountering obstacle.
    if ((*m_encoder_count <= start_count - m_counts_per_rotation) || (*m_encoder_count >= start_count + m_counts_per_rotation)){
      release();
      return 0;  
    }
    delay(50);
  }

  //release and return success
  release();
  return 1;
}


//turn to obstacle using default timeout
bool TI_DRV8838_with_encoder::turnToObstacle(bool turn_direction){
  return turnToObstacle(turn_direction, default_timeout_ms);
}

//turn to absolute position, allowing for some slack
bool TI_DRV8838_with_encoder::turnToAngleDeg(float setpoint_deg, long unsigned timeout_ms){
  bool already_in_range = false;
  long unsigned time_entered_range_ms;
  long unsigned time_spent_in_range_ms = 0;
  long unsigned time_start_ms = millis();
  
  m_PID_setpoint_deg = setpoint_deg;
  
  while (time_spent_in_range_ms < MOTOR_STABILITY_THRESHOLD_MS) {
    //update output according to PID
    m_PID_input_deg = angleCountToDeg(*m_encoder_count);
    m_PID.run();
    if (m_PID_output_dc > 0) 
      forward(m_PID_output_dc);
    else 
      reverse(-1 * m_PID_output_dc);

    //update loop exit condition
    if(m_PID.atSetPoint(PID_threshold_deg)) {
      if (!already_in_range) {
        time_entered_range_ms = millis();
        already_in_range = true;
      }
      else{
        time_spent_in_range_ms = millis() - time_entered_range_ms;
      }
    }
    else{
      time_spent_in_range_ms = 0;
      already_in_range = false;
    }

    //release motor and return failure if timeout is reached
    if (millis() - time_start_ms > timeout_ms){
      release();
      return 0;
    }
  }

  //release and return success
  release();
  return 1;
}

bool TI_DRV8838_with_encoder::turnToAngleDeg(float setpoint_deg){
  return turnToAngleDeg(setpoint_deg, default_timeout_ms);
}
