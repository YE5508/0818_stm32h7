#include "motor.h"
#include "protocol.h"
#include "fdcan.h"

#define M2006_RATIO 36
#define M3508_RATIO 3591 / 187

#define M2006_NUM 8
#define M3508_NUM 8

#define Zero_Distance 10

DJmotorLimit limit;

DJMotor DJmotor[USE_DJNUM];

static int16_t ABS(int16_t x)
{
    if (x > 0)
        return x;
    else
        return -x;
}

static int16_t GetSign(int16_t x)
{
    if (x > 0)
        return 1;
    else if (x < 0)
        return -1;
    else
        return 0;
}

/*DJ电机初始化*/
void DJmotor_Init(void)
{
    DJmotorParam dj2006_param;
    DJmotorParam dj3508_param;
    DJmotorLimit limit;
    DJmotorStatus statusFlag;
    DJmotorArgum argum;
    DJmotorError error;

    dj2006_param.ParamID = 0x1ffU;
    dj2006_param.Gear_ratio = 1.0f;
    dj2006_param.Reduction_ratio = M2006_RATIO;
    dj2006_param.PulsePerRound = 8191U;
    dj2006_param.CurrentLimit_raw = 4500;

    dj3508_param.ParamID = 0x200U;
    dj3508_param.Gear_ratio = 1.0f;
    dj3508_param.Reduction_ratio = M3508_RATIO;
    dj3508_param.PulsePerRound = 8191U;
    dj3508_param.CurrentLimit_raw = 10000;

    limit.CurrentLimitFlag = true;
    limit.IsLooseStuck = false;

    limit.MaxAngle_deg = 270.0f;
    limit.MinAngle_deg = -270.0f;
    limit.PosAngleLimitFlag = false;
    limit.PosRPMFlag = true;
    limit.PosRPMLimit = 500;

    statusFlag.IsSetZero = true;
    statusFlag.Overtimeflag = false;
    statusFlag.StuckFlag = false;
    statusFlag.ZeroFlag = false;

    argum.pulselock = 0;
    argum.zeroCnt = 0;
    argum.GapCnt = 0;

    error.lastRxTime = 0;
    error.stuckCount = 0;
    error.timeoutCount = 0;

    for (uint32_t i = 0; i < USE_DJNUM; i++)
    {
        DJmotor[i].Begin = false;
        DJmotor[i].MODE_Set = DJ_Disable; /*上电失能，发0电流*/
        DJmotor[i].statusFlag = statusFlag;
        DJmotor[i].limit = limit;
        DJmotor[i].argum = argum;
        DJmotor[i].error = error;
        DJmotor[i].valSet.current_raw = 0;
        DJmotor[i].valSet.speed_rpm = 0;
        DJmotor[i].valNow.PulseTotal = 0;
        DJmotor[i].valPre.PulseRead = 0;
    }
    for (uint32_t i = 0; i < M2006_NUM; i++)
    {
        DJmotor[i].ID = (uint8_t)(i + 1U);
        DJmotor[i].param = dj2006_param;
    }
    for (uint32_t i = 0; i < M3508_NUM; i++)
    {
        DJmotor[i].ID = (uint8_t)(i + M2006_NUM + 1U);
        DJmotor[i].param = dj3508_param;
    }

    for (uint32_t i = 0; i < USE_DJNUM; i++)
    {
        PID_Init(&DJmotor[i].posPID, 0.07f, 0.0005f, 0.0f, PIDPOS);
        PID_Init(&DJmotor[i].velPID, 5.5f, 0.3f, 0.01f, PIDINC);
    }
}

void DJmotor_SetZero(DJMotorPointer motor)
{
    motor->valPre.PulseRead=motor->valNow.PulseRead;
    motor->valNow.PulseTotal=0;
}

void DJmotor_AngleCalculate(DJMotorPointer motor)
{
    motor->valNow.PulseGap = (int16_t)(motor->valNow.PulseRead - motor->valPre.PulseRead);

    if (ABS(motor->valNow.PulseGap) > 4096)
    {
        motor->valNow.PulseGap = (uint16_t)(motor->valNow.PulseGap -
                                            GetSign(motor->valNow.PulseGap) * (int32_t)motor->param.PulsePerRound);
    }

    motor->valNow.PulseTotal+=motor->valNow.PulseGap;
    motor->valNow.angle_deg=(float)motor->valNow.PulseTotal*360.0f/((float)motor->param.PulsePerRound*motor->param.Gear_ratio*motor->param.Reduction_ratio);

    if(motor->Begin)
    {
        motor->argum.pulselock=motor->valNow.PulseTotal;
    }

    if(motor->statusFlag.IsSetZero)
    {
        DJmotor_SetZero(motor);
        motor->statusFlag.IsSetZero = false;
    }
    motor->valPre = motor->valNow;
}

void DJmotor_Receive(CanMsg_t msg)
{
    if((msg.idtype!=FDCAN_STANDARD_ID)||(msg.frametype!=FDCAN_DATA_FRAME)||(msg.id<0x201U)||(msg.id>0x208U))
    {
        return;
    }

    uint8_t card_id = (uint8_t)(msg.id-0x200U);

    if(card_id>USE_DJNUM)
    {
        return;
    }

    DJMotorPointer motor = &DJmotor[card_id-1U];

    motor->valNow.PulseRead=(int16_t)(((uint16_t)msg.data[0]<<8)|msg.data[1]);
    motor->valNow.speed_rpm=(int16_t)(((uint16_t)msg.data[2]<<8)|msg.data[3]);
    motor->valNow.current_raw=(int16_t)(((uint16_t)msg.data[4]<<8)|msg.data[5]);

    if(motor->param.Reduction_ratio == M3508_RATIO)
    {
        motor->valNow.temperature_C=(int8_t)msg.data[6];
        motor->valNow.current_A=(float)motor->valNow.current_raw*0.0012207f;
    }
    else
    {
        motor->valNow.current_A=(float)motor->valNow.current_raw/10000.0f*10.0f;
    }

    motor->valNow.speed_rpm/=(motor->param.Gear_ratio*motor->param.Reduction_ratio);

    motor->error.lastRxTime=0;
    DJmotor_AngleCalculate(motor);
}

static void EncodeS16Data(volatile int16_t *d16,uint8_t* d8)
{
    d8[1]=(int8_t)((*d16>>8)&0x00FF);
    d8[0]=(int8_t)(*d16&0x00FF);
}

static void ChangeDataByte(uint8_t *a, uint8_t *b)
{
    int8_t c;
    c=*a;
    *a=*b;
    *b=c;
}

FDCAN_HandleTypeDef *DJmotor_GetCanHandle(void)
{
    return &hfdcan2;
}

void DJmotor_CurrentTransmit(DJMotorPointer motor)
{
    static uint8_t tx_data[8]={0};
    FDCAN_TxHeaderTypeDef tx_header = {0};
    uint8_t tag=0;

    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.BitRateSwitch=FDCAN_BRS_OFF;
    tx_header.DataLength=FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
    tx_header.FDFormat=FDCAN_CLASSIC_CAN;
    tx_header.MessageMarker=0;
    tx_header.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
    tx_header.TxFrameType=FDCAN_DATA_FRAME;
    
    if(motor->ID<=4U)
    {
        tx_header.Identifier=0x200U;
        tag=(uint8_t)((motor->ID-1U)*2U);
    }
    else
    {
        tx_header.Identifier=0x1FFU;
        tag=(uint8_t)((motor->ID-5U)*2U);
    }

    EncodeS16Data(&motor->valSet.current_raw,&tx_data[tag]);
    ChangeDataByte(&tx_data[tag],&tx_data[tag+1U]);

    if(motor->ID>=1U&&motor->ID<=8U)
    {
        HAL_FDCAN_AddMessageToTxFifoQ(DJmotor_GetCanHandle(),&tx_header,tx_data);
    }
}


static void DJmotor_SwitchMode(DJMotorPointer motor)
{
	if (motor->MODE_Set != motor->MODE_Cur)
	{
		motor->MODE_Cur = motor->MODE_Set;
		motor->valSet.current_raw = 0;
		motor->valSet.speed_rpm = 0;
		motor->valSet.angle_deg = motor->valNow.angle_deg;
		/* 清误差历史与位置环累加的目标速度(velPID.SetVal),避免残留值冲击新模式 */
		PID_Reset(&motor->posPID);
		PID_Reset(&motor->velPID);
		motor->statusFlag.ZeroFlag = false;
		motor->statusFlag.Overtimeflag = false;
		motor->statusFlag.StuckFlag = false;
        if(motor->MODE_Cur!=DJ_Disable)
        {
            motor->Begin =true;
        }
	}
}

static int16_t ClampPeak(int16_t raw_data,int16_t limit)
{
    if(ABS(raw_data)>limit)
    return limit*GetSign(raw_data);
    else
    return raw_data;
}

/*速度模式*/
void DJmotor_SpeedMode(DJMotorPointer motor)
{
	motor->velPID.SetVal = (float)motor->valSet.speed_rpm * motor->param.Gear_ratio *
						   motor->param.Reduction_ratio;
	motor->velPID.CurVal = (float)motor->valNow.speed_rpm * motor->param.Gear_ratio *
						   motor->param.Reduction_ratio;

	if (motor->limit.RPMLimitFlag)
	{
		motor->velPID.SetVal = ClampPeak(motor->velPID.SetVal, motor->limit.SpeedRPMLimit);
	}

	motor->valSet.current_raw += PID_Caculate(&motor->velPID);
	motor->valSet.current_raw = (int16_t)ClampPeak(motor->valSet.current_raw, motor->param.CurrentLimit_raw);
}

static int32_t Clamp(int32_t x,int32_t min,int32_t max)
{
    if(x>max)
    return max;
    else if(x<min)
    return min;
    else 
    return x;
}

void DJmotor_PositionMode(DJMotorPointer motor)
{
	motor->valSet.PulseTotal = (int32_t)(motor->valSet.angle_deg * motor->param.Gear_ratio *
										 motor->param.Reduction_ratio *
										 (float)motor->param.PulsePerRound / 360.0f);
	motor->posPID.SetVal = (float)motor->valSet.PulseTotal;
	if (motor->limit.PosAngleLimitFlag)
	{
		const int32_t max_pulse = (int32_t)(motor->limit.MaxAngle_deg *
											(float)motor->param.PulsePerRound *
											motor->param.Gear_ratio * motor->param.Reduction_ratio / 360.0f);
		const int32_t min_pulse = (int32_t)(motor->limit.MinAngle_deg *
											(float)motor->param.PulsePerRound *
											motor->param.Gear_ratio * motor->param.Reduction_ratio / 360.0f);

		motor->posPID.SetVal = Clamp(motor->valSet.PulseTotal, min_pulse, max_pulse);
	}

	motor->posPID.CurVal = (float)motor->valNow.PulseTotal;

	motor->velPID.SetVal = PID_Caculate(&motor->posPID);
	motor->velPID.CurVal = (float)motor->valNow.speed_rpm * motor->param.Gear_ratio * motor->param.Reduction_ratio;
	if (motor->limit.PosRPMFlag)
	{
		motor->velPID.SetVal = ClampPeak(motor->velPID.SetVal, motor->limit.PosRPMLimit);
	}

	motor->valSet.current_raw += PID_Caculate(&motor->velPID);
	motor->valSet.current_raw = (int16_t)ClampPeak(motor->valSet.current_raw, motor->param.CurrentLimit_raw);
}

void DJmotor_ZeroMode(DJMotorPointer motor)
{
	motor->velPID.SetVal = (float)motor->limit.ZeroRPMLimit;
	motor->velPID.CurVal = (float)motor->valNow.speed_rpm;
	motor->valSet.current_raw += PID_Caculate(&motor->velPID);
	motor->valSet.current_raw = (int16_t)ClampPeak(motor->valSet.current_raw, motor->limit.ZeroCurrentLimit_raw);

	if (ABS(motor->valNow.PulseGap) < Zero_Distance)
	{
		if (motor->argum.zeroCnt++ > 100U)
		{
			motor->argum.zeroCnt = 0;
			motor->statusFlag.ZeroFlag = true;
			motor->Begin = false;
			/* 寻零结束不走 SwitchMode,这里手动清 PID 历史,重新使能时从零起步 */
			PID_Reset(&motor->posPID);
			PID_Reset(&motor->velPID);
			DJmotor_SetZero(motor);
		}
	}
}

static void DJmotor_Monitor(DJMotorPointer motor)
{
	if (motor->valNow.PulseGap < 5 && motor->valNow.current_raw > 3000)
	{
		if (motor->error.stuckCount++ > 500U)
		{
			motor->error.stuckCount = 0;
			motor->statusFlag.StuckFlag = true;
			if (motor->limit.IsLooseStuck)
			{
				motor->MODE_Set = DJ_Disable;
			}
		}
	}
	else
	{
		motor->error.stuckCount = 0;
	}

	if (motor->error.lastRxTime++ > 50U)
	{
		if (motor->error.timeoutCount++ > 20U)
		{
			motor->error.timeoutCount = 0;
			motor->MODE_Set = DJ_Disable;
			motor->statusFlag.Overtimeflag = true;
		}
	}
}


void DJmotor_Func(void)
{
	for (uint32_t i = 0; i < USE_DJNUM; i++)
	{
		if (DJmotor[i].Begin)
		{
			// DJmotor_Monitor(&DJmotor[i]);
			DJmotor_SwitchMode(&DJmotor[i]);

			switch (DJmotor[i].MODE_Cur)
			{
			case DJ_Disable:
				DJmotor[i].valSet.current_raw = 0;
				DJmotor_CurrentTransmit(&DJmotor[i]);
				continue;
				break;
			case DJ_RPM:
				DJmotor_SpeedMode(&DJmotor[i]);
				break;
			case DJ_Position:
				DJmotor_PositionMode(&DJmotor[i]);
				break;
			case DJ_Zero:
				DJmotor_ZeroMode(&DJmotor[i]);
				break;
			case DJ_Current:
				/* 直通电流:任务层每周期写 valSet.current_raw,这里补限幅 */
				ClampPeak(DJmotor[i].valSet.current_raw, DJmotor[i].param.CurrentLimit_raw);
				break;
			default:
				break;
			}
		}
		else
		{
			/* Begin=false(未初始化/寻零完成):强制 0 电流,防止残留累加电流持续输出 */
			DJmotor[i].valSet.current_raw = 0;
		}
		DJmotor_CurrentTransmit(&DJmotor[i]);
	}
}







