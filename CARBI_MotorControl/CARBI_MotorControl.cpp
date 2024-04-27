#include "CARBI_MotorControl.h"

CARBI_MotorControl::CARBI_MotorControl(){
  pinMode(encoder1_A, INPUT);
  pinMode(encoder1_B, INPUT);
  pinMode(encoder1_X, INPUT);
  pinMode(encoder2_A, INPUT);
  pinMode(encoder2_B, INPUT);
  pinMode(encoder2_X, INPUT);

  pinMode(MOTOR_DRIVE_1_DIR,OUTPUT);
  pinMode(MOTOR_DRIVE_2_DIR,OUTPUT);

  ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_DRIVE_1_PWM, PWM_CHANNEL_A);

  ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_DRIVE_2_PWM, PWM_CHANNEL_B);


}

void CARBI_MotorControl::valocityControl(uint8_t motorIndex,float valocity){
}
void CARBI_MotorControl::positionControl(uint8_t motorIndex,float position ,float valocity){

}
void CARBI_MotorControl::motordrive(uint8_t index,uint8_t dir,uint8_t pwm){
  switch(index){
    case 0:
      digitalWrite(MOTOR_DRIVE_1_DIR,dir);
      ledcWrite(PWM_CHANNEL_A, pwm);
      break;

    case 1:
      digitalWrite(MOTOR_DRIVE_2_DIR,dir);
      ledcWrite(PWM_CHANNEL_B, pwm);
      break;
  }
  
}