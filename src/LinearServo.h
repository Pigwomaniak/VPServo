//
// Created by Maciek on 05.01.2021.
//

#ifndef PROGRAMSKOKSERVA_LINEARSERVO_H
#define PROGRAMSKOKSERVA_LINEARSERVO_H


#include <Encoder.h>
#include <PID_v1.h>
#include "MotorDriverTB6612FNG.h"


#define PWM_H_LIMIT (255)
#define PWM_L_LIMIT (-255)
#define ENCODER_1_PIN (2)
#define ENCODER_2_PIN (3)
#define INPUT_PULSE_PIN (1)
#define MIN_SIGNAL_INPUT (1000.0)
#define MAX_SIGNAL_INPUT (2000.0)
#define MIN_BASE_VEL (5)
#define KP_DEFAULT (6)
#define KI_DEFAULT (0.8)
#define KD_DEFAULT (0)
#define PID_SAMPLING_TIME_MS (10)
#define VELOCITY_MIN_TIME (100000)


class LinearServo {
public:

    LinearServo();
    LinearServo(unsigned int pwmLowLimit, unsigned int pwmHighLimit, unsigned int impulsesPerEncoderRevolution,
                unsigned int maxRevolutions, unsigned int gearRatio);
    ~LinearServo();

    volatile unsigned int inputSignal = 0;
    volatile unsigned long prevTime = 0;
    uint8_t latestInterruptedPin = 0;

    void base(int basePwm);
    void tune(double _kp, double _ki, double _kd);
    void compute();
    int32_t getPosition();
    void directMotorControl(int power);
    void  directPosDest(double _positionDestination);
    double getVelocity() const {return velocity;}
    void velocityCompute();


private:
PID* pidController;
Encoder encoder = Encoder(ENCODER_1_PIN, ENCODER_2_PIN);
MotorDriverTB6612FNG motorDriver;

unsigned int impulsesPerEncoderRevolution;
unsigned int maxRevolutions;
double gearRatio;
unsigned long lastPosMeasureTime = 0;
long lastPos = 0;
double positionDestination = 0;
double actualPosition = 0;
double output = 0;
double velocity = 0;
unsigned long positionDestinationCompute() const;

};


#endif //PROGRAMSKOKSERVA_LINEARSERVO_H
