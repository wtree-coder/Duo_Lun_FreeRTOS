#include "communication.h"
#include "DR16.h"
#include "drv_can.h"
#include "gimbal_config.h"
#include <math.h>

#define CHASSIS_FOLLOW_KP 2

uint8_t CAN1_0x300_Tx_Data[8];

float vx, vy, wz;

void Limit_V(float v, float limit_v)
{
    if(v > limit_v) v = limit_v;
}

void Message_Update(void)
{
    float c = cosf(gimbal.total_yaw);
    float s = sinf(gimbal.total_yaw);
    vx = DR16_Data.Left_X * c + DR16_Data.Left_Y * s;  //vx*c + vy*s
    vy = DR16_Data.Left_X * s + DR16_Data.Left_Y * c;

    float delta_angle = gimbal.motor_6020.now_angle - gimbal.motor_6020.angle_offset;
    wz = delta_angle * CHASSIS_FOLLOW_KP;
}

void C_SendMessage_To_A_1ms_Callback(void)
{
    // float 不能直接 >> / &，先量化成 int16
    int16_t vx_temp = (int16_t)(vx * MAX_V); Limit_V(vx_temp, MAX_V);
    int16_t vy_temp = (int16_t)(vy * MAX_V); Limit_V(vy_temp, MAX_V);
    int16_t wz_temp = (int16_t)(wz * MAX_V); Limit_V(wz_temp, MAX_W);

    CAN1_0x300_Tx_Data[0] = (uint8_t)(vx_temp >> 8);
    CAN1_0x300_Tx_Data[1] = (uint8_t)(vx_temp & 0xFF);
    CAN1_0x300_Tx_Data[2] = (uint8_t)(vy_temp >> 8);
    CAN1_0x300_Tx_Data[3] = (uint8_t)(vy_temp & 0xFF);
    CAN1_0x300_Tx_Data[4] = (uint8_t)(wz_temp >> 8);
    CAN1_0x300_Tx_Data[5] = (uint8_t)(wz_temp & 0xFF);
    CAN1_0x300_Tx_Data[6] = DR16_Data.is_ok ? (uint8_t)DR16_Data.Chassis_Switch : (uint8_t)DR16_SWITCH_DOWN;

    CAN_Send_Data(&hcan1, 0x300, CAN1_0x300_Tx_Data, 7);
}
