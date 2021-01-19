#include <Arduino.h>
#include "LinearServo.h"
#include "PinChangeInt-master/PinChangeInt.h"

#define UART_ON true
#define KP (10)
#define KI (1)
#define KD (0.5)
#define BASE_PWM (30)

LinearServo servo;
LinearServo* servoPtr = &servo;

String inputString = "";
bool inputComplete = false;

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
        while (Serial.available()){
            char inChar = (char)Serial.read();
            inputString += inChar;
            if (inChar == '\n'){
                inputComplete = true;
            }
        }
        if (inputComplete){
            servoPtr->inputSignal = inputString.toInt();
            inputString = "";
            inputComplete = false;
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
