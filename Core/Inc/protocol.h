#ifndef PROTOCOL_H
#define PROTOCOL_H

typedef struct {
    uint8_t  idtype;
    uint8_t  frametype; 
    uint32_t id;          
    uint8_t  dlc;
    uint8_t  data[8];
} CanMsg_t;



#endif