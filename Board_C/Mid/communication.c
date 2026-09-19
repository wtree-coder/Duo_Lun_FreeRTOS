#include "communication.h"
#include "DR16.h"
#include "drv_can.h"

uint8_t CAN1_0x300_Tx_Data[8];

void C_SendMessage_To_A_1ms_Callback(void)
{
    int16_t vx, vy, wz;
    if (DR16_Data.is_ok)
    {
        vx = (int16_t)(DR16_Data.Left_Y  * MAX_V);
        vy = (int16_t)(DR16_Data.Left_X  * MAX_V);
        wz = (int16_t)(DR16_Data.Right_X * MAX_V);
    }
    else
    {
        vx = vy = wz = 0;
    }

    CAN1_0x300_Tx_Data[0] = (uint8_t)(vx >> 8);
    CAN1_0x300_Tx_Data[1] = (uint8_t)(vx & 0xFF);
    CAN1_0x300_Tx_Data[2] = (uint8_t)(vy >> 8);
    CAN1_0x300_Tx_Data[3] = (uint8_t)(vy & 0xFF);
    CAN1_0x300_Tx_Data[4] = (uint8_t)(wz >> 8);
    CAN1_0x300_Tx_Data[5] = (uint8_t)(wz & 0xFF);
    CAN1_0x300_Tx_Data[6] = DR16_Data.is_ok ? (uint8_t)DR16_Data.Chassis_Switch : (uint8_t)DR16_SWITCH_DOWN;

    CAN_Send_Data(&hcan1, 0x300, CAN1_0x300_Tx_Data, 7);
}
