#include "l298ndcmotor.h"
#include <Arduino.h>

#define BAUD 9600

// platform
#define IN1_1 22
#define IN2_1 23
#define EN_1 2

// right track
#define IN1_2 28
#define IN2_2 29
#define EN_2 5

// left track
#define IN1_3 26
#define IN2_3 27
#define EN_3 4

// shoulder
#define IN1_4 24
#define IN2_4 25
#define EN_4 3

// arm
#define IN1_5 30
#define IN2_5 31
#define EN_5 6

// bucket
#define IN1_6 32
#define IN2_6 33
#define EN_6 7

L298NDCMotor motors[] = {
  L298NDCMotor(IN1_1, IN2_1, EN_1),
  L298NDCMotor(IN1_2, IN2_2, EN_2),
  L298NDCMotor(IN1_3, IN2_3, EN_3),
  L298NDCMotor(IN1_4, IN2_4, EN_4, 80),
  L298NDCMotor(IN1_5, IN2_5, EN_5, 80),
  L298NDCMotor(IN1_6, IN2_6, EN_6)
};

const char* motorNames[] = {
  "platform", "righttrack", "lefttrack", "shoulder", "arm", "bucket"
};

const int N_MOTORS = sizeof(motors) / sizeof(motors[0]);;

L298NDCMotor* getMotorByName(const String& name) {
  for (int i = 0; i < N_MOTORS; i++) {
    if (name.equals(motorNames[i])) {
      return &motors[i];
    }
  }
  return nullptr;
}

MotorDirection parseDirection(char c) {
  switch (c) {
    case 'F': return FORWARD;
    case 'B': return BACKWARD;
    default:  return RELEASE;
  }
}

void setup() {
  Serial.begin(BAUD);
  for (int i = 0; i < N_MOTORS; i++) {
    motors[i].begin();
  }
  Serial.println("Motor controller ready");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();  // Remove whitespace/newline

    int firstColon = cmd.indexOf(':');
    int lastColon = cmd.lastIndexOf(':');

    if (firstColon == -1 || lastColon == -1 || firstColon == lastColon) {
      Serial.println("Invalid command format. Use: name:D:SPEED");
      return;
    }

    String name = cmd.substring(0, firstColon);
    char dirChar = cmd.charAt(firstColon + 1);
    int speed = cmd.substring(lastColon + 1).toInt();

    L298NDCMotor* motor = getMotorByName(name);
    if (!motor) {
      Serial.println("Unknown motor: " + name);
      return;
    }

    motor->setSpeed(speed);
    motor->run(parseDirection(dirChar));
  
    Serial.print("Processed cmd: ");
    Serial.println(cmd);
  }
}
