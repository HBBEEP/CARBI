#include "CARBI_MotorControl.h"
#include <iostream>
#include <string>

#define D_STATE           0 
#define MOTOR_PWM         1  
#define MOTOR_VELOCITY    2    
#define RESET             3  
#define SENSOR_CALLBACK   4

//control
float prevRadian[4] = {0,0,0,0};
float motorSpeedCallback[4] = {0,0,0,0};
unsigned long prev_time[4];

//communication (read)
String incomingByte ;
uint8_t state = RESET;
//motor Velocity Mode
float setPoint[4] = {0,0,0,0};
//motor PWM Mode
uint8_t motorDir = CW;
uint8_t motorIndex;
//sensor Callback Mode
uint8_t sensorIndex;

// uint8_t motorPWM = 0;

//communication (write)
unsigned long prev_time_pub;

//other
int delay_t = 50;

CARBI_MotorControl carbiMotorPWM;
CARBI_MotorControl carbiMotor_A;
CARBI_MotorControl carbiMotor_B;
CARBI_MotorControl carbiMotor_D;

void setup() {
  Serial.begin(115200);
  Serial.println("CARBI Start!");
}

void loop() {
  cmd_sub();
  // motor_speed_pub();
  motor_speed_callback();
  if(state == MOTOR_VELOCITY){
    carbiMotor_A.motordrive(MOTOR_A, carbiMotor_A.motorDir,carbiMotor_A.motorSpeed);
    carbiMotor_B.motordrive(MOTOR_B, carbiMotor_B.motorDir,carbiMotor_B.motorSpeed);
    carbiMotor_D.motordrive(MOTOR_D, carbiMotor_D.motorDir,carbiMotor_D.motorSpeed);
  }
}

void cmd_sub(){
  if (Serial.available() > 0) {
    incomingByte = Serial.readStringUntil('\n');
    switch(incomingByte[0]){
      case 'r': 
        state = RESET;
        break;
      case 'm':
        state = MOTOR_PWM;
        motorIndex = incomingByte.substring(2).toInt();
        motorDir = incomingByte.substring(4).toInt();
        setPoint[motorIndex] = incomingByte.substring(6).toFloat();
        break;
      case 'v':
        state = MOTOR_VELOCITY;
        setPoint[MOTOR_A] = incomingByte.substring(2).toFloat();
        setPoint[MOTOR_B] = incomingByte.substring(7).toFloat();
        setPoint[MOTOR_C] = incomingByte.substring(12).toFloat();
        setPoint[MOTOR_D] = incomingByte.substring(17).toFloat();
        Serial.println(setPoint[MOTOR_A]);
        Serial.println(setPoint[MOTOR_B]);
        Serial.println(setPoint[MOTOR_C]);
        Serial.println(setPoint[MOTOR_D]);
        break;
      case 's':
        state = SENSOR_CALLBACK;

      default: 
        break;
    }
    control_state();
  }
}

void control_state(){
  switch(state){
    case RESET :
      Serial.println("Reset Mode!");
      carbiMotor_A.motorSpeed = 0;
      carbiMotor_B.motorSpeed = 0;
      carbiMotor_D.motorSpeed = 0;
      state = D_STATE;
      break;
    case MOTOR_PWM :
      Serial.println("Motor PWM Mode!");
      Serial.println(motorIndex);
      Serial.println(motorDir);
      Serial.println(setPoint[motorIndex]);
      carbiMotorPWM.motordrive(motorIndex, motorDir, setPoint[motorIndex]);
      state = D_STATE;
      break;
    case MOTOR_VELOCITY :
      Serial.println("Motor Velocity Mode!");
      motor_control_A();
      motor_control_B();
      motor_control_D();
      break;
    case SENSOR_CALLBACK :
      Serial.println("Sensor Callback Mode!");
      Serial.println(motorSpeedCallback[sensorIndex]);
      state = D_STATE;
      break;
    case D_STATE :
      break;
  }
}

void PID_control(float setPoint,float currentSpeed ,uint8_t* dirOut ,uint8_t* pwmOut){
  if(setPoint<1.5 && setPoint>0){
    setPoint = 1.5;
  }
  if(setPoint>6.1){
    setPoint = 6.1;
  }
  float e = setPoint-currentSpeed;  
  uint8_t dir = CW;
  uint8_t pwm = e*43.0;
  if(setPoint<0){
    dir = CCW;
    pwm = pwm*-1;
  }
  else{
    dir = CW;
  }
  if(pwm>255){
    pwm = 255;
  }
  if(setPoint==0.0){
    pwm = 0;
  } 
  *pwmOut = pwm;
  *dirOut = dir;
}

float get_speed(uint8_t motorIndex,float currentRadian){
  float speed = (currentRadian-prevRadian[motorIndex])/0.05;
  prevRadian[motorIndex] = currentRadian;
  return speed;
}

void motor_speed_callback(){
  motorSpeedCallback[MOTOR_A] = get_speed(MOTOR_A,carbiMotor_A.degreeToRadian(carbiMotor_A.getDegree(MOTOR_A)));
  motorSpeedCallback[MOTOR_B] = get_speed(MOTOR_B,carbiMotor_B.degreeToRadian(carbiMotor_B.getDegree(MOTOR_B)));
  motorSpeedCallback[MOTOR_D] = get_speed(MOTOR_D,carbiMotor_D.degreeToRadian(carbiMotor_D.getDegree(MOTOR_D)));
}

void motor_speed_pub(){
    // Serial.println("Pub");
  if(millis() - prev_time_pub > delay_t){
    prev_time_pub = millis();
    Serial.print(motorSpeedCallback[MOTOR_A]);
    Serial.print("/");
    Serial.print(motorSpeedCallback[MOTOR_B]);
    Serial.print("/");
    // Serial.print(motorSpeedCallback[MOTOR_C]);
    // Serial.print("/");
    Serial.println(motorSpeedCallback[MOTOR_D]);
  }
}

void motor_control_A(){
  if(millis() - prev_time[MOTOR_A] > delay_t){
    prev_time[MOTOR_A] = millis();
    PID_control(setPoint[MOTOR_A],motorSpeedCallback[MOTOR_A], &carbiMotor_A.motorDir, &carbiMotor_A.motorSpeed);
  }
}

void motor_control_B(){
  if(millis() - prev_time[MOTOR_B] > delay_t){
    prev_time[MOTOR_B] = millis();
    PID_control(setPoint[MOTOR_B],motorSpeedCallback[MOTOR_B], &carbiMotor_B.motorDir, &carbiMotor_B.motorSpeed);
  }
}

void motor_control_D(){
  if(millis() - prev_time[MOTOR_D] > delay_t){
    prev_time[MOTOR_D] = millis();
    PID_control(setPoint[MOTOR_D],motorSpeedCallback[MOTOR_D], &carbiMotor_D.motorDir, &carbiMotor_D.motorSpeed);
  }
}