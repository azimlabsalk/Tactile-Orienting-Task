// motor controller with encoder feedback. Brushed motor driven by DRV8838
// includes a PID controller
// Written for Salk Institute, Azim lab
// Created by by Masakazu Igarashi & others
// Last edited by Mark Stambaugh (UCSD) on 2020-02-29

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

//PID parameters
#define PID_OUTPUT_MIN_DEFAULT -80
#define PID_OUTPUT_MAX_DEFAULT 80
#define PID_KP_DEFAULT 0.500 //* 360.0/COUNTS_PER_ROTATION // * 360/2048 due to switch from encoder counts to degrees
#define PID_KI_DEFAULT 0.000 // KI != 0 causes drifting over time
#define PID_KD_DEFAULT 0.003 //* 360.0/COUNTS_PER_ROTATION // * 360/2048 due to switch from encoder counts to degrees
#define PID_TIMESTEP_MS_DEFAULT 1
#define PID_THRESHOLD_DEG_DEFAULT 5.0

//motor control parameters
#define MOTOR_TIMEOUT_DEFAULT_MS 2000
#define MOTOR_STABILITY_THRESHOLD_MS 50

#define FORWARD 1
#define REVERSE 0

#include <AutoPID.h> //https://r-downing.github.io/AutoPID/

//DRV8838 by Texas Instruments: Brushed Motor Driver. Open-loop control only
class TI_DRV8838 {
  // declare as (sleep, phase, enable)
  public:
    TI_DRV8838(const unsigned int sleepPin, const unsigned int phasePin, const unsigned int enablePin);
    
    //returns the current speed (0-100%)
    double getCurrentSpeed();
    
    //move forward at max speed
    void forward();
    
    //move forward at a set speed (0-100%)
    void forward(double speed);
    
    //reverse at max speed
    void reverse();
    
    //reverse at a set speed (0-100%)
    void reverse(double speed);

    //brake
    void brake();
    
    //stop controlling the motor
    void release();

    //change the PWM frequency (default 490Hz) out of audible range
    void silentOperation();
    
  protected:
    void setSpeed(double speed);
    
    int m_sleepPin, m_phasePin, m_enablePin;
    double m_speed;
};

//DRV8838 by Texas Instruments: Brushed Motor Driver. Constructor includes PID controller. 
class TI_DRV8838_with_encoder : public TI_DRV8838 {
  // declare as (sleep, phase, enable, encoder, encoder resolution)
  public:
    TI_DRV8838_with_encoder(const unsigned int sleepPin, const unsigned int phasePin, const unsigned int enablePin, volatile int *encoder_count, int counts_per_rotation);
    
    //set PID parameters if desired different from default
    void set_PID_params(double Kp, double Ki, double Kd, double output_min, double output_max, unsigned long timestep, double threshold_deg);

    //zero the encoder at the current position
    void setAngleToZero();
    
    //Input angle degrees, output encoder count. Takes 48usec
    int angleDegToCount(float angle_deg);
    
    //Input encoder count, output angle in degrees. Takes 4usec
    float angleCountToDeg(int count);
    
    //returns the current angle in degrees
    double getCurrentAngleDeg();
    
    //turns at low speed until movement stops
    bool turnToObstacle(bool turn_direction, long unsigned timeout_ms);
    bool turnToObstacle(bool turn_direction); //uses default timeout
    
    //turn to absolute position, allowing for some slack
    bool turnToAngleDeg(float setpoint_deg, long unsigned timeout_ms);
    bool turnToAngleDeg(float setpoint_deg); //uses default timeout
    
    long unsigned default_timeout_ms = MOTOR_TIMEOUT_DEFAULT_MS;
    double PID_threshold_deg = PID_THRESHOLD_DEG_DEFAULT;
    
  protected:
    double m_angle;
    int *m_encoder_count;
    int m_counts_per_rotation;
    double m_PID_input_deg, m_PID_setpoint_deg, m_PID_output_dc;
    AutoPID m_PID;
};

#endif //MOTOR_CONTROLLER_H
