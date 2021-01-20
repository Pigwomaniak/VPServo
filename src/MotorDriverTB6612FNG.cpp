//
// Created by Maciek on 05.01.2021.
//

#include "MotorDriverTB6612FNG.h"

MotorDriverTB6612FNG::MotorDriverTB6612FNG(int in1Pin, int in2Pin, int pwmPin, int stbyPin)
        : in1Pin(in1Pin), in2Pin(in2Pin), pwmPin(pwmPin), stbyPin(stbyPin) {
    pinMode(in1Pin, OUTPUT);
    pinMode(in2Pin, OUTPUT);
    pinMode(pwmPin, OUTPUT);
    pinMode(stbyPin,OUTPUT);
    of();
}

void MotorDriverTB6612FNG::on() {
    if(!stateOn){
        digitalWrite(stbyPin, HIGH);
        stateOn = true;
    }
}

void MotorDriverTB6612FNG::of() {
    if (stateOn){
        digitalWrite(stbyPin, LOW);
        stateOn = false;
    }
    analogWrite(pwmPin, 0);
}

void MotorDriverTB6612FNG::run(int power) {
    if (power == 0){
        shortBrake();
    }else if (power > 0){
        cw();
    } else{
        ccw();
    }
    if (abs(power) > PWM_MAX){
        power = PWM_MAX;
    }
    analogWrite(pwmPin, abs(power));
}

void MotorDriverTB6612FNG::ccw() const {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
}

void MotorDriverTB6612FNG::cw() const {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
}

void MotorDriverTB6612FNG::shortBrake() const {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
}

MotorDriverTB6612FNG::MotorDriverTB6612FNG() {
    in1Pin = 6;
    in2Pin = 7;
    pwmPin = 9;
    stbyPin = 5;
}
