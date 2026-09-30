#include "communication.h"
#include "DR16.h"
#include "drv_can.h"
#include "gimbal_config.h"
#include <math.h>

#define CHASSIS_FOLLOW_KP 2

uint8_t CAN1_0x300_Tx_Data[8];

float vx = 0.0f, vy = 0.0f, wz = 0.0f;
int16_t vx_temp,vy_temp,wz_temp;

void Message_Update(void)
{
    // 云台相对底盘偏角，由 Gimbal_Data_Update() 每 1ms 算出并归一化到 (-PI, PI]，
    // 底盘跟随模式稳态下趋于 0，此时推单杆不会耦合到另一轴。
    float delta_angle = gimbal.delta_angle;

    // 云台系速度指令 -> 底盘系，标准旋转 R(-delta)
    // 底盘跟随模式稳态下 delta -> 0，推单杆不会耦合到另一轴。
    // 两式第二项符号必须相反，写成 [[c,s],[s,c] 会退化成镜像变换。
    float c = cosf(delta_angle);
    float s = sinf(delta_angle);
    vx = ( DR16_Data.Left_X * c + DR16_Data.Left_Y * s) * MAX_V;
    vy = (-DR16_Data.Left_X * s + DR16_Data.Left_Y * c) * MAX_V;

    wz = delta_angle * CHASSIS_FOLLOW_KP;
    if(wz > 3.0f) wz = 3.0f;
    if(wz < -3.0f) wz = -3.0f;
}

void C_SendMessage_To_A_1ms_Callback(void)
{
    //Message_Update();

    vx = DR16_Data.Left_X;
    vy = DR16_Data.Left_Y;

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

    if(DR16_Data.is_ok && DR16_Data.Chassis_Switch == DR16_SWITCH_MID)
    {
        CAN_Send_Data(&hcan1, 0x300, CAN1_0x300_Tx_Data, 7);
    }
}
