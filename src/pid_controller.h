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

    //Precomputed gains
    float escaledKi;
    float escaledKd;

    //Memory
    float prevError;
    float intTerm;

    //Output
    float out;
    
}pidController;

void resetPID (pidController *pid);

void calculatePID (float reference,
                   float measurement,
                   float feedforward,
                   pidController *pid);

void updatePID (pidController *pid,
                float kp,
                float ki,
                float kd,
                float highLimit,
                float lowLimit,
                float deltaTime);


float limitValue (float input,
                  float highLimit,
                  float lowLimit);

#endif // PID_CONTROLLER_H
