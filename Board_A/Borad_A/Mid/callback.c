#include "callback.h"
#include "communication.h"
#include "motor.h"
#include "chassis_config.h"

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if(rx_header.StdId == 0x300)
    {
        Message_Update(rx_data);
    }
    else if(rx_header.StdId >= 0x201 && rx_header.StdId <= 0x204)
    {
        Motor_Rx_Callback(&chassis.motor_3508[rx_header.StdId - 0x201], rx_data);
    }
    else if(rx_header.StdId >= 0x205 && rx_header.StdId <= 0x208)
    {
        Motor_Rx_Callback(&chassis.motor_6020[rx_header.StdId - 0x205], rx_data);
    }
}
