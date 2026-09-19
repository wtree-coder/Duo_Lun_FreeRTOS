#include "communication.h"
#include "chassis_config.h"

void A_GetMessage_From_C(uint8_t *data)
{
    int16_t vx = (int16_t)((data[0] << 8) | data[1]);
    int16_t vy = (int16_t)((data[2] << 8) | data[3]);
    int16_t wz = (int16_t)((data[4] << 8) | data[5]);

    chassis.vx = vx / 3000.0f * V_MAX;
    chassis.vy = vy / 3000.0f * V_MAX;
    chassis.wz = wz / 3000.0f * WZ_MAX_RADPS;

    chassis.chassis_switch = data[6];

    switch (chassis.chassis_switch)
    {
        case DR16_SWITCH_UP:
            chassis.mode = Chassis_Move;
            break;
        case DR16_SWITCH_MID:
            chassis.mode = Chassis_XiaoTuoLuo;
            break;
        case DR16_SWITCH_DOWN:
        default:
            chassis.mode = Chassis_Disable;
            break;
    }
}
