/**
 * \file            pid_controller.c
 * \brief           PID controller written in C with integrator anti-windup,
 *                  filtered derivative and feed-forward input.
 *
 * Author:          pmdn <pmdn@mailbox.org>
 * Version:         0.1
 */

#include "pid_controller.h"

void pid_reset(pid_controller_t *pid)
{
    pid->error_previous = 0.0f;
    pid->integral_term = 0.0f;
    pid->derivative_term = 0.0f;
    pid->out = 0.0f;
}

void pid_update(pid_controller_t *pid,
                float kp,
                float ki,
                float kd,
                float limit_high,
                float limit_low,
                float time_delta,
                int32_t filter_n)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->limit_max = limit_high;
    pid->limit_min = limit_low;
    pid->time_diff = time_delta;
    pid->filter_n = filter_n;
    pid->ki_escaled = pid->ki * 0.5f * pid->time_diff;
    pid->kd1_escaled = 1.0f / (1.0f + (float) pid->filter_n * pid->time_diff);
    pid->kd2_escaled = pid->kd * ((float) pid->filter_n / (1.0f + (float) pid->filter_n * pid->time_diff));
}

void pid_calculate(float reference,
                   float measurement,
                   float feedforward,
                   pid_controller_t *pid)
{
    float error = reference - measurement;

    float proportional_term = pid->kp * error;

    if (((pid->out >= pid->limit_max) || (pid->out <= pid->limit_min)) && (error * pid->out > 0.0f))
    {
        pid->integral_term += 0.0f;
    }
    else
    {
        pid->integral_term += pid->ki_escaled * (error-pid->error_previous);
    }

    pid->derivative_term = pid->kd1_escaled * pid->derivative_term + pid->kd2_escaled * (error-pid->error_previous);
   
    float out_aux = proportional_term + pid->integral_term + pid->derivative_term + feedforward;
    out_aux = value_limit(out_aux, pid->limit_max, pid->limit_min);
    pid->out = out_aux;

    pid->error_previous = error;
}

float value_limit(float input,
                  float limit_high,
                  float limit_low)
{
    if (input >= limit_high)
    {
        return limit_high;
    }
    else if (input <= limit_low)
    {
        return limit_low;
    }
    return input;
}
