#include "vofa_config.h"
#include "vofa.h"
#include "DR16.h"
#include "gimbal_config.h"
#include "imu.h"
#include "communication.h"

void Vofa_20ms_Send_Callback(void)
{
    float temp[10];
    static uint32_t vofa_count = 0;
    if(++vofa_count < 10) return;
    vofa_count = 0;

    //Vofa_Set_Data(3, &gimbal.motor_6020.total_encoder, &gimbal.motor_6020.now_angle, &gimbal.motor_6020.now_rad_s);

    temp[0] = (float)gimbal.motor_6020.now_rad_s;
    temp[1] = (float)gimbal.motor_6020.target_rad_s;
    Vofa_Set_Data(2, &temp[0], &temp[1]);
    //Vofa_Set_Data(7, &temp[0], &temp[1], &temp[2], &temp[3], &temp[4], &temp[5], &temp[6]);
    Vofa_Send_Data();
}

