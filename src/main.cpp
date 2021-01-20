#include <Arduino.h>
#include "LinearServo.h"
#include "PinChangeInt-master/PinChangeInt.h"

#define UART_ON true
#define BASE_PWM (30)
#define KP (0.001)
#define KI (0)
#define KD (0)

LinearServo servo;
LinearServo* servoPtr = &servo;

String inputString = "";

void rising();
void falling();
void inputInterruptSetUp();

void setup() {
    if (!UART_ON){
        inputInterruptSetUp();
    } else{
        Serial.begin(9600);
    }
    servo.tune(KP, KI, KD);
    servo.base(BASE_PWM);
}

void loop() {
    servo.compute();
    if (UART_ON){
        if (Serial.available()){
            inputString = Serial.readStringUntil('\n');
            servo.inputSignal = inputString.toInt();
            Serial.println(servo.inputSignal);
        }
    }
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
