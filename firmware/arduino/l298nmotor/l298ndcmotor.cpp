#include "L298NDCMotor.h"

L298NDCMotor::L298NDCMotor(int in1, int in2, int en)
  : in1Pin(in1), in2Pin(in2), enPin(en) {}

L298NDCMotor::L298NDCMotor(int in1, int in2, int en, int lowPWM)
  : in1Pin(in1), in2Pin(in2), enPin(en), motorLow(lowPWM) {}

void L298NDCMotor::begin() {
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(enPin, OUTPUT);
  run(RELEASE);
  setSpeed(0);
}

void L298NDCMotor::setSpeed(int speedPercent) {
  speedPercent = constrain(speedPercent, 0, userMax);
  currentSpeed = speedPercent;
  apply();
}

void L298NDCMotor::run(MotorDirection dir) {
  currentDir = dir;
  apply();
}

void L298NDCMotor::apply() {
  int pwm = map(currentSpeed, 0, userMax, 0, motorHigh);
  if (pwm < motorLow && currentSpeed > 0) pwm = motorLow;

  switch (currentDir) {
    case FORWARD:
      digitalWrite(in1Pin, LOW);
      digitalWrite(in2Pin, HIGH);
      break;
    case BACKWARD:
      digitalWrite(in1Pin, HIGH);
      digitalWrite(in2Pin, LOW);
      break;
    case RELEASE:
    default:
      digitalWrite(in1Pin, LOW);
      digitalWrite(in2Pin, LOW);
      pwm = 0;
      break;
  }

  analogWrite(enPin, pwm);
}

int L298NDCMotor::getSpeed() const {
  return currentSpeed;
}

MotorDirection L298NDCMotor::getDirection() const {
  return currentDir;
}
