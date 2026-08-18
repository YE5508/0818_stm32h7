#include "pid.h"

float PID_Caculate(PIDType* pid)
{
    pid->err[0] = pid->SetVal-pid->CurVal;

    switch (pid->mode)
    {
        case PIDINC:
            pid->output = pid->Kp*(pid->err[0]-pid->err[1])+
                          pid->Ki*pid->err[0]+
                          pid->Kd*(pid->err[0]-2*pid->err[1]+pid->err[2]);
            pid->err[2]=pid->err[1];
            pid->err[1]=pid->err[0];
            
                        break;
        case PIDPOS:
            pid->output = pid->Kp*pid->err[0]+
                          pid->Ki*pid->err[2]+
                          pid->Kd*(pid->err[0]-pid->err[1]);
            pid->err[1]=pid->err[0];
            pid->err[2]+=pid->err[0];
        default:
            break;
    }
    return pid->output;

}

void PID_Reset(PIDType* pid)
{
    pid->err[0]=0;
    pid->err[1]=0;
    pid->err[2]=0;
}
void PID_Init(PIDType* pid,float Kp,float Ki,float Kd,PIDmode mode)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->mode=mode;
    pid->SetVal=pid->CurVal;
    pid->output=0;
    pid->err[0]=0;
    pid->err[1]=0;
    pid->err[2]=0;
}