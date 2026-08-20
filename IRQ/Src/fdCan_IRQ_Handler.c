#include "fdCan_IRQ_Handler.h"

void fdCan_Start(void)
{
    HAL_FDCAN_Start(&hfdcan2);
    HAL_FDCAN_ActivateNotification(&hfdcan2,FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

void fdCAN_Send_Data(uint32_t ide, uint32_t id, uint8_t dlc, uint8_t *data)
{
    FDCAN_TxHeaderTypeDef tx = {0};
    tx.DataLength = dlc;
    tx.BitRateSwitch = FDCAN_BRS_OFF;
    tx.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
    tx.FDFormat=FDCAN_CLASSIC_CAN;
    tx.Identifier=id;
    tx.IdType=ide;
    tx.MessageMarker=0;
    tx.TxEventFifoControl=FDCAN_STORE_TX_EVENTS;
    tx.TxFrameType=FDCAN_DATA_FRAME;//数据帧
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &tx, data);
}



void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,uint32_t RxFifo0ITs)
{
    if (hfdcan->Instance != FDCAN2)
        return;

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0)
        return;

    
    FDCAN_RxHeaderTypeDef rx = {0};
    uint8_t data[8] = {0};
    CanMsg_t Msg = {0};

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx, data) != HAL_OK)
        return;
    memcpy(Msg.data,data,8);
    Msg.dlc=rx.DataLength;//经典CAN模式下，DataLength为0-8
    Msg.idtype=rx.IdType;
    Msg.frametype=rx.RxFrameType;
    Msg.id=rx.Identifier;
    osMessageQueuePut(can_rx_queue, &Msg, 0, 0);

}
