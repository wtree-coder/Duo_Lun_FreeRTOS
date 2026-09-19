#include "vofa_config.h"
#include "vofa.h"
#include "chassis_config.h"

void Vofa_20ms_Send_Callback(void)
{
    static uint32_t vofa_count = 0;
    if(++vofa_count < 20) return;
    vofa_count = 0;

    Vofa_Set_Data(4, &chassis.motor_3508[0].total_encoder, &chassis.motor_3508[1].total_encoder,
                     &chassis.motor_3508[2].total_encoder, &chassis.motor_3508[3].total_encoder);

    //Vofa_Set_Data(4, &chassis.motor_6020[0].total_encoder, &chassis.motor_6020[1].total_encode
    //                &chassis.motor_6020[2].total_encode, &chassis.motor_6020[3].total_encode);
    Vofa_Send_Data();
}

