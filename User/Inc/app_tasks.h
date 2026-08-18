#ifndef APP_TASKS_H
#define APP_TASKS_H

#include "cmsis_os2.h"
#include "protocol.h"
#include "motor.h"

extern osMessageQueueId_t can_rx_queue;
void app_tasks_create(void);


#endif