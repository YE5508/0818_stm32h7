#ifndef PID_H
#define PID_H

/*includes*/
#include "main.h"

typedef enum 
{
    PIDINC = 0,
    PIDPOS
}PIDmode;

typedef struct
{
    PIDmode mode;
    float Kp;
    float Ki;
    float Kd;
    float err[3];
    float output;
    float SetVal;
    float CurVal;
}PIDType;

float PID_Caculate(PIDType* pid);
void PID_Reset(PIDType* pid);
void PID_Init(PIDType* pid,float Kp,float Ki,float Kd,PIDmode mode);
#endif PID_H