//
// Created by Maciek on 19.01.2021.
//
#include <Arduino.h>
#include "LinearServo.h"

LinearServo servo;

void setup() {
    Serial.begin(9600);
    servo.directMotorControl(20);
}

void loop() {
    Serial.println(servo.getPosition());
    delay(500);
}