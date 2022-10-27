#include "pid_controller.h"

/* Refs:
 * https://github.com/pms67/PID
 * https://www.youtube.com/watch?v=zOByx3Izf5U&feature=emb_title
 * https://github.com/br3ttb/Arduino-PID-Library
 * http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
 * https://github.com/akharsa/qPID
 * https://github.com/uLipe/PidControlTemplate/tree/master/lib
 */

void initPID (pidController *pid)
{
    pid->prevError = 0.0f;
    pid->intTerm = 0.0f;
    pid->out = 0.0f;
}

void calculatePID (float reference, float measurement, float feedforward, pidController *pid)
{
    float error = reference - measurement;

    float propTerm = pid->Kp * error;

    if (((pid->out >= pid->limMax) || (pid->out <= pid->limMin)) && (error * pid->out > 0.0f))
    {
        pid->intTerm += 0.0f;
    }
    else
    {
        pid->intTerm += pid->Ki * (error-pid->prevError) * 0.5f * pid->deltaT;
    }

    //TODO Filtrado
    float derivTerm = pid->Kd * (error-pid->prevError) / pid->deltaT;
   
    float auxOut = propTerm + pid->intTerm + derivTerm + feedforward;
    auxOut = limitValue(auxOut, pid->limMax, pid->limMin);
    pid->out = auxOut;

    pid->prevError = error;
}

void precomputePID (pidController *pid)
{
    //TODO
    //pid->Ki * 0.5f * pid->deltaT;
    //pid->Kd / pid->deltaT;
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
