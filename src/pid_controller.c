/* PID library for C
 *
 * Revision 0.1
 *
 * Description: PID controller written in C with integrator anti-windup,
 * filtered derivative and feed-forward input.
 * 
 */

#include "pid_controller.h"

void resetPID (pidController *pid)
{
    pid->prevError = 0.0f;
    pid->intTerm = 0.0f;
    pid->derivTerm = 0.0f;
    pid->out = 0.0f;
}

void updatePID (pidController *pid,
                float kp,
                float ki,
                float kd,
                float highLimit,
                float lowLimit,
                float deltaTime,
                int nFilter)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->limMax = highLimit;
    pid->limMin = lowLimit;
    pid->deltaT = deltaTime;
    pid->filterN = nFilter;
    pid->escaledKi = pid->Ki * 0.5f * pid->deltaT;
    pid->escaledKd1 = 1.0f / (1.0f + (float) pid->filterN * pid->deltaT);
    pid->escaledKd2 = pid->Kd * ((float) pid->filterN / (1.0f + (float) pid->filterN * pid->deltaT));
}


void calculatePID (float reference,
                   float measurement,
                   float feedforward,
                   pidController *pid)
{
    float error = reference - measurement;

    float propTerm = pid->Kp * error;

    if (((pid->out >= pid->limMax) || (pid->out <= pid->limMin)) && (error * pid->out > 0.0f))
    {
        pid->intTerm += 0.0f;
    }
    else
    {
        pid->intTerm += pid->escaledKi * (error-pid->prevError);
    }

    pid->derivTerm = pid->escaledKd1 * pid->derivTerm + pid->escaledKd2 * (error-pid->prevError);
   
    float auxOut = propTerm + pid->intTerm + pid->derivTerm + feedforward;
    auxOut = limitValue(auxOut, pid->limMax, pid->limMin);
    pid->out = auxOut;

    pid->prevError = error;
}

float limitValue (float input,
                  float highLimit,
                  float lowLimit)
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
