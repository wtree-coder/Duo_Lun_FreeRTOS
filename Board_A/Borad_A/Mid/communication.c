#include "communication.h"
#include "chassis_config.h"
#include "drv_can.h"
#include "can.h"

void Communication_Init(void)
{
    //CAN_Filter_Mask_Config(&hcan1, CAN_FILTER(0) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0x300, 0x7FF);
    CAN_Filter_Mask_Config(&hcan1, CAN_FILTER(0) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
}

//已验证遥控器接收无误
void Message_Update(uint8_t *rx_data)
{
    int16_t vx_temp = (int16_t)((rx_data[0] << 8) | rx_data[1]);
    int16_t vy_temp = (int16_t)((rx_data[2] << 8) | rx_data[3]);
    int16_t wz_temp = (int16_t)((rx_data[4] << 8) | rx_data[5]);

    chassis.vx = vx_temp / 1000.0f;
    chassis.vy = vy_temp / 1000.0f;
    chassis.wz = wz_temp / 1000.0f;

    switch(rx_data[6])
    {
        case 1:
            chassis.mode = Chassis_XiaoTuoLuo;
            break;
        case 3:
            chassis.mode = Chassis_Move;
            break;
        case 2:
        default:
            chassis.mode = Chassis_Disable;
            break;
    }
    
    chassis.is_ok = 1;
    chassis.check_count = 0;
    chassis.cmd_rx_count++;
}
