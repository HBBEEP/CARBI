#include "CARBI_MotorControl.h"
// #include <ESP32Encoder.h>
ESP32Encoder encoder_A;
ESP32Encoder encoder_B;
ESP32Encoder encoder_C;
ESP32Encoder encoder_D;

CARBI_MotorControl::CARBI_MotorControl(){
  pinMode(encoder1_A_Pin, INPUT);
  pinMode(encoder1_B_Pin, INPUT);
  pinMode(encoder1_X_Pin, INPUT);
  pinMode(encoder2_A_Pin, INPUT);
  pinMode(encoder2_B_Pin, INPUT);
  pinMode(encoder2_X_Pin, INPUT);

  encoder_A.attachHalfQuad( encoder1_A_Pin, encoder1_B_Pin);
  encoder_B.attachHalfQuad( encoder2_A_Pin, encoder2_B_Pin);


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
  motorDirStatus[index] = dir;
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

float CARBI_MotorControl::getDegree(uint8_t motorIndex){
  float degree=0.0;
  switch(motorIndex){
    case MOTOR_A :
      degree = encoder_A.getCount() / 2;
      degree = (degree/2048)*360.0;
      break;
    case MOTOR_B :
      degree = encoder_B.getCount() / 2;
      degree = (degree/2048)*360.0;
      break;
  }
  return degree;
}

float CARBI_MotorControl::getSpeed(uint8_t motorIndex){
  switch(motorIndex){
    case MOTOR_A :
      // encoder_A.clearCount();
      return getDegree(motorIndex); 
      break;
  }
}

float CARBI_MotorControl::PID(float setPoint ,float nowSpeed){
  float e = setPoint-nowSpeed; 
  float speed = e*P;
  if(speed>255){
    speed = 255;
  }
  else if(speed<100){
    speed = 100;
  }
  motordrive(MOTOR_A,CW,255);
  return e;
}

float CARBI_MotorControl::degreeToRadian(float degree){
  return degree*(PI/180);
}




