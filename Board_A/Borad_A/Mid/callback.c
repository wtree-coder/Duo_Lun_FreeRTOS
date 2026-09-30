#include "callback.h"
#include "communication.h"
#include "motor.h"
#include "chassis_config.h"
#include "vofa_config.h"


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    if(hcan->Instance == CAN1)
    {
        switch(rx_header.StdId)
        {
            case 0x300:
                Message_Update(rx_data);
                break;
            case 0x201: Motor_Rx_Callback(&chassis.motor_3508[LD], rx_data); break;
            case 0x202: Motor_Rx_Callback(&chassis.motor_3508[RD], rx_data); break;
            case 0x203: Motor_Rx_Callback(&chassis.motor_3508[RU], rx_data); break;
            case 0x204: Motor_Rx_Callback(&chassis.motor_3508[LU], rx_data); break;
            default: break;
        }

    }
    if(hcan->Instance == CAN2)
    {
        switch(rx_header.StdId)
        {
            case 0x205: Motor_Rx_Callback(&chassis.motor_6020[LU], rx_data); break;
            case 0x206: Motor_Rx_Callback(&chassis.motor_6020[LD], rx_data); break;
            case 0x207: Motor_Rx_Callback(&chassis.motor_6020[RD], rx_data); break;
            case 0x208: Motor_Rx_Callback(&chassis.motor_6020[RU], rx_data); break;
            default: break;
        }
    }
}
