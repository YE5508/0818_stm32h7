#include "Tim_IRQ_Handler.h"

void Tim2_Start(void)
{
    HAL_TIM_Base_Start_IT(&htim2);
}

void TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{   
    if(htim->Instance==TIM2)
    {
        DJmotor_Func();        
    }

}