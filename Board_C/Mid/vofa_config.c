#include "vofa_config.h"
#include "vofa.h"
#include "DR16.h"
#include "gimbal_config.h"
#include "imu.h"

void Vofa_20ms_Send_Callback(void)
{
    float temp[10];
    static uint32_t vofa_count = 0;
    if(++vofa_count < 10) return;
    vofa_count = 0;

    //Vofa_Set_Data(3, &gimbal.motor_6020.total_encoder, &gimbal.motor_6020.now_angle, &gimbal.motor_6020.now_rad_s);
    temp[0] = gimbal.target_yaw;
    temp[1] = gimbal.total_yaw;
    temp[2] = gimbal.motor_6020.now_angle;
    temp[3] = gimbal.motor_6020.target_rad_s;
    temp[4] = gimbal.motor_6020.now_rad_s;
    temp[5] = (float)gimbal.motor_6020.out;
    temp[6] = DR16_Data.Right_X;
    temp[7] = gimbal.motor_6020.now_torque;      // 电机反馈的实际电流
    temp[8] = (float)gimbal.motor_6020.is_ok;    // 反馈是否在收
    Vofa_Set_Data(9, &temp[0], &temp[1], &temp[2], &temp[3], &temp[4], &temp[5], &temp[6], &temp[7], &temp[8]);
    Vofa_Send_Data();
}

