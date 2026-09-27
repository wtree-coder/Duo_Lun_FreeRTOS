#include "communication.h"
#include "chassis_config.h"

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

    chassis.check_count = 0;
    chassis.is_ok = 1;
}
