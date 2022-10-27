#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdio.h>
#include <stdint.h>

typedef struct pidController
{
    //Gains
    float Kp;
    float Ki;
    float Kd;

    //Limits
    float limMax;
    float limMin;

    //Execution period
    float deltaT;

    //Memory
    float prevError;
    float intTerm;

    //Output
    float out;
    
}pidController;

void initPID (pidController *pid);

void calculatePID (float reference, float measurement, float feedforward, pidController *pid);

void precomputePID (pidController *pid);

float limitValue (float input, float highLimit, float lowLimit);

#endif // PID_CONTROLLER_H
