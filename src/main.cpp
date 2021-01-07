#include <Arduino.h>
#include "LinearServo.h"
#include "PinChangeInt-master/PinChangeInt.h"


#define KP (10)
#define KI (1)
#define KD (0.5)
#define BASE_PWM (30)

LinearServo servo;
LinearServo* servoPtr = &servo;
void rising();
void falling();
void inputInterruptSetUp();

void setup() {
// write your initialization code here
    inputInterruptSetUp();
    servo.tune(KP, KI, KD);
    servo.base(BASE_PWM);
}

void loop() {
// write your code here
    servo.compute();

}

void rising() {
    servoPtr->latestInterruptedPin = PCintPort::arduinoPin;
    PCintPort::attachInterrupt(servoPtr->latestInterruptedPin, falling, FALLING);
    servoPtr->prevTime = micros();
}

void falling() {
    servoPtr->latestInterruptedPin = PCintPort::arduinoPin;
    PCintPort::attachInterrupt(servoPtr->latestInterruptedPin, rising, FALLING);
    unsigned long pulseLen = micros() - servoPtr->prevTime;
    if ((pulseLen >= 800) && (pulseLen <= 2200)){
        servoPtr->inputSignal = pulseLen;
    }
}

void inputInterruptSetUp() {
    pinMode(INPUT_PULSE_PIN, INPUT);
    digitalWrite(INPUT_PULSE_PIN, HIGH);
    PCintPort::attachInterrupt(INPUT_PULSE_PIN, rising, RISING);
}