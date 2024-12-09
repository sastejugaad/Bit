#include "motor_control.h"
#include <driver/ledc.h>

MotorControl::MotorControl(int pwmPin1, int pwmPin2, int ain1, int ain2, int bin1, int bin2, int stbyPin) {
  this->pwmPin1 = pwmPin1;
  this->pwmPin2 = pwmPin2;
  this->ain1 = ain1;
  this->ain2 = ain2;
  this->bin1 = bin1;
  this->bin2 = bin2;
  this->stbyPin = stbyPin; 
  this->RightMotor = 0;
  this->LeftMotor = 0;

  // Initialize pins
  pinMode(ain1, OUTPUT);
  pinMode(ain2, OUTPUT);
  pinMode(bin1, OUTPUT);
  pinMode(bin2, OUTPUT);
  pinMode(stbyPin, OUTPUT);

  // Initialize the LEDC timer
  ledcSetup(0, 1000, 8); // Channel 0, 1000 Hz, 8-bit resolution
  ledcSetup(1, 1000, 8); // Channel 1, 1000 Hz, 8-bit resolution
  ledcAttachPin(pwmPin1, 0); // Attach PWM1 pin to channel 0
  ledcAttachPin(pwmPin2, 1); // Attach PWM2 pin to channel 1

  // Standby mode OFF (motors enabled)
  digitalWrite(stbyPin, HIGH);
}

void MotorControl::setMotorSpeed(int pwmPin, int speed) {
  // Set motor speed using PWM
  ledcWrite(pwmPin == pwmPin1 ? 0 : 1, speed);
}

void MotorControl::LeftWheelSpeed(int speed) {
  this->LeftMotor = speed;
  setMotorSpeed(pwmPin1, speed);
}

void MotorControl::RightWheelSpeed(int speed) {
  this->RightMotor = speed;
  setMotorSpeed(pwmPin2, speed);
}

void MotorControl::forward() {
  digitalWrite(ain1, LOW);
  digitalWrite(ain2, HIGH);
  digitalWrite(bin1, LOW);
  digitalWrite(bin2, HIGH);
  setMotorSpeed(pwmPin1, LeftMotor);
  setMotorSpeed(pwmPin2, RightMotor);
}

void MotorControl::back() {
  digitalWrite(ain1, HIGH);
  digitalWrite(ain2, LOW);
  digitalWrite(bin1, HIGH);
  digitalWrite(bin2, LOW);
  setMotorSpeed(pwmPin1, LeftMotor);
  setMotorSpeed(pwmPin2, RightMotor);
}

void MotorControl::left() {
  digitalWrite(ain1, HIGH);
  digitalWrite(ain2, LOW);
  digitalWrite(bin1, LOW);
  digitalWrite(bin2, HIGH);
  setMotorSpeed(pwmPin1, LeftMotor);
  setMotorSpeed(pwmPin2, RightMotor);
}

void MotorControl::right() {


   digitalWrite(ain1, LOW);
  digitalWrite(ain2, HIGH);
  digitalWrite(bin1, HIGH);
  digitalWrite(bin2, LOW);
  setMotorSpeed(pwmPin1, LeftMotor);
  setMotorSpeed(pwmPin2, RightMotor);
}

void MotorControl::Stop() {
  digitalWrite(ain1, LOW);
  digitalWrite(ain2, LOW);
  digitalWrite(bin1, LOW);
  digitalWrite(bin2, LOW);
  setMotorSpeed(pwmPin1, 0);
  setMotorSpeed(pwmPin2, 0);
}
