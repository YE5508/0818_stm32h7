#ifndef PROTOCOL_H
#define PROTOCOL_H

#include "main.h"

typedef struct {
    uint8_t  idtype;
    uint8_t  frametype; 
    uint32_t id;          
    uint8_t  dlc;
    uint8_t  data[8];
} CanMsg_t;

typedef struct {
    FDCAN_RxHeaderTypeDef Rxheader;
    uint8_t *Rx_Data;
    uint8_t bus;
}ZdriveReceiveMsg;


#endif