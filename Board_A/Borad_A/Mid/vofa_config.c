#include "vofa_config.h"
#include "vofa.h"
#include "chassis_config.h"
#include "can.h"

void Vofa_20ms_Send_Callback(void)
{
    static uint32_t vofa_count = 0;

    if(++vofa_count < 20) return;
    vofa_count = 0;

    float temp[8];
    for(int i = 0;i < 4;i++)
    {
        temp[i] = (float)chassis.motor_6020[i].rx_encoder;
    }
    temp[4] = (float)chassis.vx;
    temp[5] = (float)chassis.is_ok;
    temp[6] = (float)chassis.cmd_rx_count;
    //Vofa_Set_Data(8, &temp[0], &temp[1], &temp[2], &temp[3], &temp[4], &temp[5], &temp[6], &temp[7]);
    Vofa_Set_Data(7, &temp[0], &temp[1], &temp[2], &temp[3], &temp[4], &temp[5], &temp[6]);
    Vofa_Send_Data();
}

