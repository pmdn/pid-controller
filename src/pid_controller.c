#include "pid_controller.h"

/* Refs:
 * https://github.com/pms67/PID
 * https://github.com/br3ttb/Arduino-PID-Library
 * https://www.youtube.com/watch?v=DtGrdB8wQ_8
 */

void initPID (pidController *pid)
{
    pid->prevError = 0.0f;
    pid->intTerm = 0.0f;
    pid->out = 0.0f;
}

void calculatePID (float reference, float measurement, pidController *pid)
{
    float error = reference - measurement;
    float propTerm = pid->Kp * error;
    float auxOut = propTerm;
    auxOut = limitValue(auxOut, pid->limMax, pid->limMin);
    pid->out = auxOut;
}

float limitValue (float input, float highLimit, float lowLimit)
{
    if (input >= highLimit)
    {
        return highLimit;
    }
    else if (input <= lowLimit)
    {
        return lowLimit;
    }
    return input;
}
