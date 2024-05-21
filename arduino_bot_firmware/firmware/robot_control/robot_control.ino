#include <Servo.h>

Servo base;
Servo shoulder;
Servo elbow;
Servo gripper;

uint8_t idx = 0;

char value[4] = "000";
uint8_t value_idx = 0;

#define base_start 90
#define shoulder_start 90
#define elbow_start 90
#define gripper_start 0

#define base_pin 9
#define shoulder_pin 10
#define elbow_pin 11
#define gripper_pin 12

void reach_goal(Servo& motor, int goal) {
  if (goal >= motor.read()) {
    for (int pos = motor.read(); pos <= goal; pos++) {
      motor.write(pos);
      delay(5);
    }
  } else {
    for (int pos = motor.read(); pos >= goal; pos--) {
      motor.write(pos);
      delay(5);
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  base.attach(base_pin);
  shoulder.attach(shoulder_pin);
  elbow.attach(elbow_pin);
  gripper.attach(gripper_pin);

  base.write(base_start);
  shoulder.write(shoulder_start);
  elbow.write(elbow_start);
  gripper.write(gripper_start);

  Serial.begin(115200);
  Serial.setTimeout(1);
}

void loop() {
  // put your main code here, to run repeatedly:

  if (Serial.available()) {
    char chr = Serial.read();
    if (chr == 'b') {
      idx = 0;
      value_idx = 0;
    } else if (chr == 's') {
      idx = 1;
      value_idx = 1;
    } else if (chr == 'e') {
      idx = 2;
      value_idx = 2;
    } else if (chr == 'g') {
      idx = 3;
      value_idx = 3;
    } else if (chr == ',') {
      int val = atoi(value);
      if (idx == 0) {
        reach_goal(base, val);
      } else if (idx == 1) {
        reach_goal(shoulder, val);
      } else if (idx == 2) {
        reach_goal(elbow, val);
      } else if (idx == 3) {
        reach_goal(gripper, val);
      }
      value[0] = '0';
      value[1] = '0';
      value[2] = '0';e

    } else {
      value[value_idx] = chr;
      value_idx++;
    }
  }
}
