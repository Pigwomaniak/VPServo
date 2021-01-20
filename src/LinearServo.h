//
// Created by Maciek on 05.01.2021.
//

#ifndef PROGRAMSKOKSERVA_LINEARSERVO_H
#define PROGRAMSKOKSERVA_LINEARSERVO_H


#include <Encoder.h>
#include <PIDController.h>
#include "MotorDriverTB6612FNG.h"


#define PWM_H_LIMIT (255)
#define PWM_L_LIMIT (-255)
#define ENCODER_1_PIN (2)
#define ENCODER_2_PIN (3)
#define INPUT_PULSE_PIN (1)
#define MIN_SIGNAL_INPUT (1000.0)
#define MAX_SIGNAL_INPUT (2000.0)
#define MIN_BASE_VEL (5)

class LinearServo {
public:

    LinearServo();
    LinearServo(unsigned int pwmLowLimit, unsigned int pwmHighLimit, unsigned int impulsesPerEncoderRevolution,
                unsigned int maxRevolutions, unsigned int gearRatio);

    volatile unsigned int inputSignal = 0;
    volatile unsigned long prevTime = 0;
    uint8_t latestInterruptedPin = 0;

    void base(int basePwm);
    void tune(double _kp, double _ki, double _kd);
    void compute();
    int32_t getPosition();
    void directMotorControl(int power);



private:
PIDController pidController;
Encoder encoder = Encoder(ENCODER_1_PIN, ENCODER_2_PIN);
MotorDriverTB6612FNG motorDriver;


unsigned int impulsesPerEncoderRevolution;
unsigned int maxRevolutions;
double gearRatio;
unsigned long lastPosMeasureTime = 0;
long lastPos = 0;

unsigned long positionDestination() const;
double velocity();
};


#endif //PROGRAMSKOKSERVA_LINEARSERVO_H
