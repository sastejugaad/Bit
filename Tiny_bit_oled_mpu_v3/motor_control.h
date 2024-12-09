#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

class MotorControl {
public:
  MotorControl(int pwmPin1, int pwmPin2, int ain1, int ain2, int bin1, int bin2, int stbyPin);
  void forward();
  void back();
  void left();
  void right();
  void Stop();
  void LeftWheelSpeed(int speed);
  void RightWheelSpeed(int speed);

private:
  int pwmPin1;
  int pwmPin2;
  int ain1;
  int ain2;
  int bin1;
  int bin2;
  int stbyPin;
  int LeftMotor;
  int RightMotor;

  void setMotorSpeed(int pwmPin, int speed);
};

#endif
