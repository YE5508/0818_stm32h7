#ifndef MOTOR_H
#define MOTOR_H

/*includes*/
#include "main.h"
#include <stdbool.h>
#include "pid.h"
#include "protocol.h"

#define USE_DJ 1
#define USE_DJNUM 8

typedef enum
{
    DJ_Disable =0,/*关：tranmit 0 current*/
    DJ_RPM,/*速度 mode*/
    DJ_Position,/*位置 mode*/
    DJ_Zero,/*寻零 mode*/
    DJ_Current,/*电流/扭矩*/
}DJmotor_mode_t;

typedef struct 
{
    volatile int16_t current_raw; //直接设置电流
    volatile float angle_deg; //输出角度，degree
    volatile int16_t speed_rpm;//valSet: 输出轴 rpm;valNow: 转子 rpm(原始反馈)
    volatile float current_A; //反馈电流, A
    volatile int16_t PulseRead; //raw encoder pulse
    volatile int16_t PulseGap; //pulse delta
    volatile int32_t PulseTotal;//accumulated pulse
    volatile int8_t temperature_C;//℃
}DJmotorVal;

typedef struct
{
    uint16_t PulsePerRound;// 8191
    float Gear_ratio; //mechanism ratio
    float Reduction_ratio;//motor reducer ratio
    uint32_t ParamID; //CAN receive ID base
    int16_t CurrentLimit_raw;//output current limit ,raw
}DJmotorParam;

typedef struct
{
    bool RPMLimitFlag;
    bool PosAngleLimitFlag;
    bool PosRPMFlag;
    bool CurrentLimitFlag;
    float MaxAngle_deg;
    float MinAngle_deg;
    int16_t SpeedRPMLimit;
    int32_t PosRPMLimit;
    int16_t ZeroRPMLimit;
    int16_t ZeroCurrentLimit_raw;
    bool IsLooseStuck;
}DJmotorLimit;

typedef struct
{
    bool IsSetZero;
    bool Overtimeflag;
    bool StuckFlag;
    bool ZeroFlag;

}DJmotorStatus;

typedef struct 
{
    int32_t pulselock;
    int32_t zeroCnt;
    int32_t GapCnt;
}DJmotorArgum;

typedef struct 
{
    uint32_t lastRxTime;
    uint16_t stuckCount;
    uint16_t timeoutCount;
}DJmotorError;


typedef struct 
{
    uint8_t ID;
    volatile bool Begin;
    volatile DJmotor_mode_t MODE_Set;
    volatile DJmotor_mode_t MODE_Cur;

    DJmotorParam param;
    DJmotorVal valSet;
    DJmotorVal valNow;
    DJmotorVal valPre;
    DJmotorStatus statusFlag;
    DJmotorLimit limit;
    DJmotorArgum argum;
    DJmotorError error;
    PIDType posPID;
    PIDType velPID;
} DJMotor,*DJMotorPointer;

#if USE_DJ
    extern DJMotor DJmotor[USE_DJNUM];

    void DJmotor_Init(void);
    void DJmotor_Func(void);
    void DJmotor_Receive(CanMsg_t msg);
    //void DJmotor_PID_Reload(DJMotorPointer moter,DJmotorPID pid_reload);


#endif
#endif