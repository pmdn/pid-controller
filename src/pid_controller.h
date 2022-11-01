/**
 * \file            pid_controller.h
 * \brief           PID controller written in C with integrator anti-windup,
 *                  filtered derivative and feed-forward input.
 *
 * Author:
 * Version:         0.1
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


#include <stdio.h>
#include <stdint.h>

/**
 * \brief           This is the pid controller struct
 * \note            This structure holds all the PID controller data,
 *                  multiple instances are allowed.
 */    

typedef struct pid_controller
{
    //Gains
    float kp;
    float ki;
    float kd;

    //Limits
    float limit_max;
    float limit_min;

    //Execution period
    float time_diff;

    //Derivative filter coefficient. Should be high, e.g 100.
    int32_t filter_n;

    //Precomputed gains
    float ki_escaled;
    float kd1_escaled;
    float kd2_escaled;

    //Memory
    float error_previous;
    float integral_term;
    float derivative_term;

    //Output
    float out;
    
}pid_controller_t;

/**
 * /brief           Resets the PID controller.
 * /param[in]       pid: pointer the PID structure.
 */ 

void pid_reset(pid_controller_t *pid);

/**
 * /brief           Calculates the output of the PID controller.
 * /param[in]       reference: reference for the controlled variable.
 * /param[in]       measurement: measurement of the controlled variable.
 * /param[in]       feedforward: value of the feed-forward compensation.
 * /param[in]       pid: pointer the PID structure.
 */ 

void pid_calculate(float reference,
                   float measurement,
                   float feedforward,
                   pid_controller_t *pid);

/**
 * /brief           Creates/updates the PID controller.
 * /param[in]       pid: pointer the PID structure.
 * /param[in]       kp: value of the proportional gain.
 * /param[in]       ki: value of the integral gain.
 * /param[in]       kd: value of the derivative gain.
 * /param[in]       limit_hig: higher limit for the output.
 * /param[in]       limit_low: lower limit for the output.
 * /param[in]       time_delta: execution period of the control loop.
 * /param[in]       filter_n: derivative filter coefficient.
 */ 

void pid_update(pid_controller_t *pid,
                float kp,
                float ki,
                float kd,
                float limit_high,
                float limit_low,
                float time_delta,
                int32_t filter_n);

/**
 * /brief           Limits the value between the desired interval.
 * /param[in]       input: value to be limitted.
 * /param[in]       limit_high: higher limit for the output.
 * /param[in]       limit_low: lower limit for the output.
 * /return          returns the limitted value.
 */

float value_limit(float input,
                  float limit_high,
                  float limit_low);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // PID_CONTROLLER_H
