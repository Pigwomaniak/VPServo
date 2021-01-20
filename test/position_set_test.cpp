//
// Created by Maciek on 20.01.2021.
//

#include <Arduino.h>
#include "LinearServo.h"

#define OSC_TIME (2000000)
#define DISPLAY_TIME (100000)

LinearServo servo;
int signal = 1500;
unsigned long prevTime = 0;
unsigned long prevTime1 = 0;
//double pos = 1000;
int signalStep = 200;
//double posStep = 500;

void setup() {
    Serial.begin(9600);
    delay(1000);
    prevTime = micros();
    prevTime1 = micros();
}

void loop() {
    if((micros() - prevTime) > OSC_TIME){
        servo.inputSignal = signal;
        signal -= signalStep;
        signalStep = -signalStep;
        prevTime = micros();
        /*
        pos -= posStep;
        posStep = -posStep;
        servo.directPosDest(pos);
        */
    }
    if((micros() - prevTime1) > DISPLAY_TIME){
        Serial.println(servo.getPosition());
    }
    servo.compute();
}