#ifndef APP_TASKS_H
#define APP_TASKS_H

#include "cmsis_os2.h"
#include "protocol.h"
#include "DJmotor.h"

extern osMessageQueueId_t zdrive_can_rx_queue;
void app_tasks_create(void);


#endif