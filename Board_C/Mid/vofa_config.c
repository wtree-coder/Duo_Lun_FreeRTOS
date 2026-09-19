#include "vofa_config.h"
#include "vofa.h"
#include "DR16.h"
#include "gimbal_config.h"

void Vofa_20ms_Send_Callback(void)
{
    static uint32_t vofa_count = 0;
    if(++vofa_count < 20) return;
    vofa_count = 0;

    //Vofa_Set_Data(3, &gimbal.motor_6020.total_encoder, &gimbal.motor_6020.now_angle, &gimbal.motor_6020.now_rad_s);
    Vofa_Send_Data();
}

