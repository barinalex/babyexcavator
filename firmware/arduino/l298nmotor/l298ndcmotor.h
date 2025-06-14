#ifndef L298NDCMotor_h
#define L298NDCMotor_h

#include <Arduino.h>

enum MotorDirection {
  FORWARD,
  BACKWARD,
  RELEASE
};

class L298NDCMotor {
  private:
    int in1Pin;
    int in2Pin;
    int enPin;

    int motorHigh = 255;     // Max PWM for motor
    int motorLow = 125;      // Min PWM that moves the motor
    int userMax = 100;       // Max input speed (percent)

    int currentSpeed = 0;    // 0–100
    MotorDirection currentDir = RELEASE;

    void apply();

  public:
    L298NDCMotor(int in1, int in2, int en);
    L298NDCMotor(int in1, int in2, int en, int motorLow);

    void begin();                       // Setup pinModes
    void setSpeed(int speedPercent);   // 0–100
    void run(MotorDirection dir);      // FORWARD/BACKWARD/RELEASE

    int getSpeed() const;
    MotorDirection getDirection() const;
};

#endif
