#include "callback.h"
#include "DR16.h"
#include "usart.h"
#include "drv_can.h"
#include "gimbal_config.h"

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance == USART3)
    {
        if(Size == 18)
        {
            DR16_Data_Get(DR16.Rx_Buff, &DR16_Data);            
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, DR16.Rx_Buff, UART_MAX_SIZE);
    }
    //接收上位机Vofa数据调参
    if(huart->Instance == USART6)
    {
        //if(...)
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if(rx_header.StdId == 0x205)  // 3508 反馈
    {
        Gimbal_6020_Rx_Callback(&rx_header, rx_data);
    }
}


