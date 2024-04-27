#ifndef CARBI_MOTORCONTROL_H
#define CARBI_MOTORCONTROL_H

#define encoder1_A 13
#define encoder1_B 12
#define encoder1_X 14
#define encoder2_A 27
#define encoder2_B 26
#define encoder2_X 25

#define GEAR_RATIO 0.2

#define MOTOR_A 0
#define MOTOR_B 1
#define MOTOR_C 2
#define MOTOR_D 3

#define PWM_CHANNEL_A 0
#define PWM_CHANNEL_B 1
#define PWM_CHANNEL_C 2
#define PWM_CHANNEL_D 3

#define PWM_RESOLUTION 8 // 8-bit resolution for PWM (0-255)
#define PWM_FREQ 1000    // PWM frequency in Hz

#define MOTOR_DRIVE_1_DIR 4
#define MOTOR_DRIVE_1_PWM 5

#define MOTOR_DRIVE_2_DIR 2
#define MOTOR_DRIVE_2_PWM 15

#include <Arduino.h>
#include "esp32-hal-gpio.h"

class CARBI_MotorControl{

public: 

  CARBI_MotorControl();
  void valocityControl(uint8_t motorIndex,float valocity);
  void positionControl(uint8_t motorIndex,float position ,float valocity);
  void motordrive(uint8_t index,uint8_t dir,uint8_t pwm);

private:
  // void motordrive(uint8_t dir,uint8_t pwm);
};

#endif