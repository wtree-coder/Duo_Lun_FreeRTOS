#include "callback.h"
#include "communication.h"
#include "motor.h"
#include "chassis_config.h"

// CAN 接收中断回调：直接按 CAN ID 分发（各 ID 互不重叠，无需区分总线）
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    chassis.check_count = 0;

    if (rx_header.StdId == 0x300)                          // C 板速度指令
    {
        A_GetMessage_From_C(rx_data);
    }
    else if (rx_header.StdId >= 0x201 && rx_header.StdId <= 0x204)  // 3508 反馈
    {
        Chassis_Motor_3508_Rx_Callback(&rx_header, rx_data);
    }
    else if (rx_header.StdId >= 0x205 && rx_header.StdId <= 0x208)  // 6020 反馈
    {
        Chassis_Motor_6020_Rx_Callback(&rx_header, rx_data);
    }
}
