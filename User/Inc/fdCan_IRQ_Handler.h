#ifndef FDCAN_IRQ_HANDLER_H
#define FDCAN_IRQ_HANDLER_H

#include "main.h"
#include "fdcan.h"
#include "app_tasks.h"


void fdCan_Start(void);
void fdCAN_Send_Data(uint32_t ide, uint32_t id, uint8_t dlc, uint8_t *data);

#endif