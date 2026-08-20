#include "app_tasks.h"
#include "protocol.h"
#include "fdCan_IRQ_Handler.h"

/* 队列句柄 */
osMessageQueueId_t can_rx_queue;

static const osThreadAttr_t Motor_Feedback_Task_attr = {.name = "Motor_Feedback_Task",  .priority = osPriorityNormal};//定义线程属性结构体，用于控制创建的线程的参数(命名，优先级)
//static const osThreadAttr_t Motor_Monitor_Task_attr = {.name = "Motor_Monitor_Task_attr",  .priority = osPriorityNormal};
void MotorFeedbackTask(void *argument)
{
    CanMsg_t msg;
    for(;;)
    {   
        if (osMessageQueueGet(can_rx_queue, &msg, NULL, osWaitForever) != osOK)
        continue;
        
        DJmotor_Receive(msg);
        osDelay(1);
    }
}


/*void MotorMonitorTask(void *argument)
{
    for(;;)
    {
        DJmotor_Monitor_All();
        osDelay(10);
    }
}*/
/*创建任务和队列*/
void app_tasks_create(void)
{
    can_rx_queue = osMessageQueueNew(8, sizeof(CanMsg_t), NULL);
    osThreadNew(MotorFeedbackTask, NULL, &Motor_Feedback_Task_attr);
    //osThreadNew(MotorMonitorTask,NULL,&Motor_Monitor_Task_attr);
}