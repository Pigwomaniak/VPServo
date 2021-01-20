//
// Created by Maciek on 05.01.2021.
//

#include "LinearServo.h"

LinearServo::LinearServo() {
    pidController.begin();
    pidController.limit(PWM_L_LIMIT, PWM_H_LIMIT);
    impulsesPerEncoderRevolution = 12;
    maxRevolutions = 15;
    gearRatio = 100;
}

LinearServo::LinearServo(unsigned int pwmLowLimit, unsigned int pwmHighLimit, unsigned int impulsesPerEncoderRevolution,
                         unsigned int maxRevolutions, unsigned int gearRatio) :
                         impulsesPerEncoderRevolution(impulsesPerEncoderRevolution), maxRevolutions(maxRevolutions),
                         gearRatio(gearRatio) {
    pidController.begin();
    pidController.limit(pwmLowLimit, pwmHighLimit);
}

void LinearServo::tune(double _kp, double _ki, double _kd) {
    pidController.tune(_kp, _ki, _kd);
}

void LinearServo::base(int basePWM) {
    motorDriver.run(-basePWM);
    delay(500);
    lastPos = encoder.read();
    lastPosMeasureTime = micros();
    while (velocity() > MIN_BASE_VEL)
    encoder.write(0);
}

void LinearServo::compute() {
    if(inputSignal < 960){
        motorDriver.of();
    } else {
        if (inputSignal < MIN_SIGNAL_INPUT){
            inputSignal = MIN_SIGNAL_INPUT;
        }
        if (inputSignal > MAX_SIGNAL_INPUT){
            inputSignal = MAX_SIGNAL_INPUT;
        }
        motorDriver.on();
        pidController.setpoint(positionDestination());
        motorDriver.run(int(pidController.compute(encoder.read())));
    }
}

unsigned long LinearServo::positionDestination() const {
    return unsigned ((inputSignal - MIN_SIGNAL_INPUT) * ((impulsesPerEncoderRevolution * gearRatio * maxRevolutions)
    / (MAX_SIGNAL_INPUT - MIN_SIGNAL_INPUT)));
}

double LinearServo::velocity() {
    double vel = double (encoder.read() - lastPos) / (micros() - lastPosMeasureTime) / 1000000;
    lastPos = encoder.read();
    lastPosMeasureTime = micros();
    return vel;
}

int32_t LinearServo::getPosition() {
    return encoder.read();
}

void LinearServo::directMotorControl(int power) {
    motorDriver.on();
    motorDriver.run(power);
}



