#include "Tim_IRQ_Handler.h"

void Tim2_Start(void)
{
    HAL_TIM_Base_Start_IT(&htim2);
}

void TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{   
    if(htim->Instance==TIM2)
    {
        #if USE_DJ
        DJmotor_Func();
        #endif
        #if USE_ZMDR
        ZdriveFunc();
        #endif        
    }

}