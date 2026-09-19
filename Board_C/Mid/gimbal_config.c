#include "gimbal_config.h"
#include "drv_can.h"
#include "motor.h"

Gimbal_t gimbal;

void Gimbal_Init(void)
{
    CAN_Init(&hcan1);
    CAN_Filter_Mask_Config(&hcan1, CAN_FILTER(0) | CAN_FIFO_0, 0x1FF, 0x7FF);
}

void Gimbal_6020_Rx_Callback(CAN_RxHeaderTypeDef *rx_header, uint8_t rx_data)
{
    switch (rx_header->StdId)
    {
        case 0x1FF:    
            Motor_Rx_Callback(&gimbal.motor_6020, &rx_data);
            break;
    
        default:
            break;
    }

}