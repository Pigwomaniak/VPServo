//
// Created by Maciek on 20.01.2021.
//

#include <Arduino.h>
#include "LinearServo.h"
#define DISPLAY_TIME (100000)

#define POWER 40

LinearServo servo;
unsigned long prevTime1 = 0;

void setup() {
    Serial.begin(9600);
    delay(1000);
    servo.directMotorControl(POWER);
    prevTime1 = micros();
}

void loop() {
    if((micros() - prevTime1) > DISPLAY_TIME){
        Serial.println(servo.velocity());
        prevTime1 = micros();
    }
    servo.velocity();
}
