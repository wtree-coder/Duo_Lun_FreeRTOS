#include "vofa_config.h"
#include "vofa.h"
#include "chassis_config.h"

void Vofa_20ms_Send_Callback(void)
{
    static uint32_t vofa_count = 0;

    if(++vofa_count < 20) return;
    vofa_count = 0;

    float temp[4];
    for(int i = 0; i < 4; i++)
    {
        temp[i] = (float)chassis.motor_6020[i].total_encoder;
    }

    Vofa_Set_Data(5, &chassis.vy, &temp[0], &temp[1], &temp[2], &temp[3]);
    Vofa_Send_Data();
}
