#include "communication.h"
#include "DR16.h"
#include "drv_can.h"
#include "gimbal_config.h"
#include <math.h>

#define CHASSIS_FOLLOW_KP 5
#define CHASSIS_FOLLOW_KF 0.8f
#define XIAOTUOLUO_WZ 1.0f

uint8_t CAN1_0x300_Tx_Data[8];

float vx = 0.0f, vy = 0.0f, wz = 0.0f;
int16_t vx_temp,vy_temp,wz_temp;

void Message_Update(void)
{
    float c = cosf(gimbal.delta_angle);
    float s = sinf(gimbal.delta_angle);
    vx = ( DR16_XiaoZhun(DR16_Data.Left_Y) * c + DR16_XiaoZhun(DR16_Data.Left_X) * s) * MAX_V;
    vy = ( DR16_XiaoZhun(DR16_Data.Left_Y) * s - DR16_XiaoZhun(DR16_Data.Left_X) * c) * MAX_V;

    if(DR16_Data.Chassis_Switch == DR16_SWITCH_UP)
    {
        wz = XIAOTUOLUO_WZ;
    }
    else
    {
        wz = gimbal.delta_angle * CHASSIS_FOLLOW_KP + gimbal.motor_6020.now_rad_s * CHASSIS_FOLLOW_KF;

        if(fabsf(gimbal.motor_6020.now_rad_s) < 0.05f)
        {
            if(fabsf(gimbal.delta_angle) < 0.1f)
            {
                wz = 0.0f;
            }
            else if(fabsf(wz) < 0.3f)
            {
                wz = copysignf(0.3f, gimbal.delta_angle);
            }
        }
    }
    if(wz > 3.0f) wz = 3.0f;
    if(wz < -3.0f) wz = -3.0f;
}

void C_SendMessage_To_A_1ms_Callback(void)
{
    if(gimbal.mode != GIMBAL_MOVE) return;
    Message_Update();

    //vx = DR16_Data.Right_Y;

    vx_temp = (int16_t)(vx * 1000.0f);
    vy_temp = (int16_t)(vy * 1000.0f);
    wz_temp = (int16_t)(wz * 1000.0f);

    CAN1_0x300_Tx_Data[0] = (uint8_t)(vx_temp >> 8);
    CAN1_0x300_Tx_Data[1] = (uint8_t)(vx_temp & 0xFF);
    CAN1_0x300_Tx_Data[2] = (uint8_t)(vy_temp >> 8);
    CAN1_0x300_Tx_Data[3] = (uint8_t)(vy_temp & 0xFF);
    CAN1_0x300_Tx_Data[4] = (uint8_t)(wz_temp >> 8);
    CAN1_0x300_Tx_Data[5] = (uint8_t)(wz_temp & 0xFF);
    CAN1_0x300_Tx_Data[6] = DR16_Data.is_ok ? (uint8_t)DR16_Data.Chassis_Switch : (uint8_t)DR16_SWITCH_DOWN;

    if(DR16_Data.is_ok)
    {
        CAN_Send_Data(&hcan1, 0x300, CAN1_0x300_Tx_Data, 7);
    }
}
