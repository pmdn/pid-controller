/* PID library for C
 *
 * Revision 0.1
 *
 * Description: PID controller written in C with integrator anti-windup,
 * filtered derivative and feed-forward input.
 * 
 */


#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdio.h>
#include <stdint.h>

/**
 * Structure that holds all the PID controller data, multiple instances are allowed.
 */

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

/**
 * @brief Resets the PID controller.
 * @param[in] pid - pointer the PID structure.
 */ 

void resetPID (pidController *pid);

/**
 * @brief Calculates the output of the PID controller.
 * @param[in] reference - reference for the controlled variable.
 * @param[in] measurement - measurement of the controlled variable.
 * @param[in] feedforward - value of the feed-forward compensation.
 * @param[in] pid - pointer the PID structure.
 */ 

void calculatePID (float reference,
                   float measurement,
                   float feedforward,
                   pidController *pid);

/**
 * @brief Creates/updates the PID controller.
 * @param[in] pid - pointer the PID structure.
 * @param[in] kp - value of the proportional gain.
 * @param[in] ki - value of the integral gain.
 * @param[in] kd - value of the derivative gain.
 * @param[in] highLimit - higher limit for the output.
 * @param[in] lowLimit - lower limit for the output.
 * @param[in] deltaTime - execution period of the control loop.
 */ 

void updatePID (pidController *pid,
                float kp,
                float ki,
                float kd,
                float highLimit,
                float lowLimit,
                float deltaTime);

/**
 * @brief Limits the value between the desired interval.
 * @param[in] input - value to be limitted.
 * @param[in] highLimit - higher limit for the output.
 * @param[in] lowLimit - lower limit for the output.
 * @return returns the limitted value.
 */

float limitValue (float input,
                  float highLimit,
                  float lowLimit);

#endif // PID_CONTROLLER_H
