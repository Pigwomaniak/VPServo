//
// Created by Maciek on 20.01.2021.
//

#include <Arduino.h>
#include "LinearServo.h"

#define OSC_TIME (2000000)

LinearServo servo;
int signal = 1500;
long prevTime = 0;
//double pos = 1000;
int signalStep = 200;
//double posStep = 500;

void setup() {
    Serial.begin(9600);
    delay(1000);
}

void loop() {
    if((micros() - prevTime) > OSC_TIME){
        servo.inputSignal = signal;
        signal -= signalStep;
        signalStep = -signalStep;
        /*
        pos -= posStep;
        posStep = -posStep;
        servo.directPosDest(pos);
        */
    }
    servo.compute();
}