#include "app_tasks.h"
#include "protocol.h"
#include "fdCan_IRQ_Handler.h"

/* 队列句柄 */
osMessageQueueId_t zdrive_can_rx_queue;

static const osThreadAttr_t Motor_Feedback_Task_attr = {.name = "Motor_Feedback_Task",  .priority = osPriorityNormal};//定义线程属性结构体，用于控制创建的线程的参数(命名，优先级)
//static const osThreadAttr_t Motor_Monitor_Task_attr = {.name = "Motor_Monitor_Task_attr",  .priority = osPriorityNormal};
void MotorFeedbackTask(void *argument)
{
    ZdriveReceiveMsg msg;
    const uint8_t* data={0};
    for(;;)
    {   fdCAN_Send_Data(0,0x001,1,data);
        osDelay(100);
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
    zdrive_can_rx_queue = osMessageQueueNew(8, sizeof(ZdriveReceiveMsg), NULL);
    osThreadNew(MotorFeedbackTask, NULL, &Motor_Feedback_Task_attr);
    //osThreadNew(MotorMonitorTask,NULL,&Motor_Monitor_Task_attr);
}