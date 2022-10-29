#include "pid_controller.h"

/* Refs:
 * https://github.com/pms67/PID
 * https://www.youtube.com/watch?v=zOByx3Izf5U&feature=emb_title
 * https://github.com/br3ttb/Arduino-PID-Library
 * http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
 * https://github.com/akharsa/qPID
 * https://github.com/uLipe/PidControlTemplate/tree/master/lib
 * https://github.com/tcleg/PID_Controller
 * https://github.com/geekfactory/PID
 */

void resetPID (pidController *pid)
{
    pid->prevError = 0.0f;
    pid->intTerm = 0.0f;
    pid->out = 0.0f;
}

void updatePID (pidController *pid,
                float kp,
                float ki,
                float kd,
                float highLimit,
                float lowLimit,
                float deltaTime)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->limMax = highLimit;
    pid->limMin = lowLimit;
    pid->deltaT = deltaTime;
    pid->escaledKi = pid->Ki * 0.5f * pid->deltaT;
    pid->escaledKd = pid->Kd / pid->deltaT;
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

    //TODO Filtrado
    float derivTerm = pid->escaledKd * (error-pid->prevError);
   
    float auxOut = propTerm + pid->intTerm + derivTerm + feedforward;
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
