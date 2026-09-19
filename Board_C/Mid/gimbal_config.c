#include "gimbal_config.h"
#include "drv_can.h"
#include "motor.h"

Gimbal_t gimbal;

void Gimbal_Init(void)
{
    CAN_Init(&hcan1);
    CAN_Filter_Mask_Config(&hcan1, CAN_FILTER(0) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0x205, 0x7FF);
    Motor_Init(&gimbal.motor_6020_down, MOTOR_6020_RATE, MOTOR_6020_MAX_RPM, MOTOR_6020_OUT_LIMIT);
}

void Gimbal_6020_Rx_Callback(CAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data)
{
    switch (rx_header->StdId)
    {
        case 0x205:    // 6020 反馈 ID（接收）
            Motor_Rx_Callback(&gimbal.motor_6020_down, rx_data);
            break;
    
        default:
            break;
    }

}
