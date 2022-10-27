#include "pid_controller.h"

/* Refs:
 * https://github.com/pms67/PID
 * https://github.com/br3ttb/Arduino-PID-Library
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
//    if ((( iqs_ref.t >= pParametrosAlgCtrl->f32IqMax ) || ( iqs_ref.t <= -pParametrosAlgCtrl->f32IqMax )) && ((error_w.t*iqs_ref.t)>0.0))
//    {
//        PI_integr_w += 0;
//    }
//    else  //No está en ninguno de los límites o el error y la salida son de signos contrarios
//    {
//        PI_integr_w += paramControlPreproc.auxKiWxCte*(error_w.t+error_w.t_1);
//    }
//    PI_deriv_w = (paramControlPreproc.auxKdWxCte*(error_w.t-error_w.t_1));
//
//    t_ref=PI_integr_w+PI_propor_w+PI_deriv_w+iq_comp+J_alfa.t+Comp_Roz;    /*Con compensación de muelle, inercia y rozamiento(en zona patín compensación de iq)*/
//    /*Limite de slew rate*/
//    t_refLim=limitarSlewRate (t_ref, t_refLim, COMPENSAC_INCREMENTO_PAR, Rate);
//
//    iqs_ref_1.t=t_refLim*paramControlPreproc.auxFpmxCteVelocidad;
//    //iqs_ref_1.t=t_ref*paramControlPreproc.auxFpmxCteVelocidad;
//    iqs_ref.t= iqs_ref_1.t;
//
//    /* Check Límites */
//    iqs_ref.t=limitar (iqs_ref.t,-pParametrosAlgCtrl->f32IqMax, pParametrosAlgCtrl->f32IqMax);

    float error = reference - measurement;

    float propTerm = pid->Kp * error;

    if (((pid->out >= pid->limMax) || (pid->out <= pid->limMin)) && (error * pid->out > 0.0f))
    {
        pid->intTerm += 0.0f;
    }
    else
    {
        //TODO Preprocesado valores
        pid->intTerm += pid->Ki * (error-pid->prevError) * 0.5f * pid->deltaT;
    }

    //TODO Filtrado
    float derivTerm = pid->Kd * (error-pid->prevError) / pid->deltaT;
   
    float auxOut = propTerm + pid->intTerm + derivTerm + feedforward;
    auxOut = limitValue(auxOut, pid->limMax, pid->limMin);
    pid->out = auxOut;

    pid->prevError = error;
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
