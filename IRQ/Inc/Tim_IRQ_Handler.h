#ifndef TIM_IRQ_HANDLER_H
#define TIM_IRQ_HANDLER_H

#include "main.h"
#include "tim.h"
#include "motor.h"


void Tim2_Start(void);
void TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif