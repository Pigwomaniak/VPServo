//
// Created by Maciek on 05.01.2021.
//

#ifndef PROGRAMSKOKSERVA_MOTORDRIVERTB6612FNG_H
#define PROGRAMSKOKSERVA_MOTORDRIVERTB6612FNG_H

#include <Arduino.h>
#define PWM_MAX (255)

class MotorDriverTB6612FNG {
public:
    MotorDriverTB6612FNG();

    MotorDriverTB6612FNG(int in1Pin, int in2Pin, int pwmPin, int stbyPin);

    void on();
    void of();
    void run(int power);

private:
    bool stateOn = false;
    int in1Pin;
    int in2Pin;
    int pwmPin;
    int stbyPin;
    void ccw() const;
    void cw() const;
    void shortBrake() const;
};


#endif //PROGRAMSKOKSERVA_MOTORDRIVERTB6612FNG_H
