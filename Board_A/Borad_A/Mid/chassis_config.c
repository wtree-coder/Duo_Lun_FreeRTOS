#include "chassis_config.h"
#include "motor.h"

Chassis_t chassis = {0};

void Chassis_Motor_3508_Rx_Callback(CAN_RxHeaderTypeDef *rxbuf, uint8_t *rx_data)
{
    switch (rxbuf->StdId)
    {
        case 0x201: Motor_Rx_Callback(&chassis.motor_3508[LU], rx_data); break;
        case 0x202: Motor_Rx_Callback(&chassis.motor_3508[LD], rx_data); break;
        case 0x203: Motor_Rx_Callback(&chassis.motor_3508[RU], rx_data); break;
        case 0x204: Motor_Rx_Callback(&chassis.motor_3508[RD], rx_data); break;
    }
}

void Chassis_Motor_6020_Rx_Callback(CAN_RxHeaderTypeDef *rxbuf, uint8_t *rx_data)
{
    switch (rxbuf->StdId)
    {
        case 0x205: Motor_Rx_Callback(&chassis.motor_6020[LU], rx_data); break;
        case 0x206: Motor_Rx_Callback(&chassis.motor_6020[LD], rx_data); break;
        case 0x207: Motor_Rx_Callback(&chassis.motor_6020[RU], rx_data); break;
        case 0x208: Motor_Rx_Callback(&chassis.motor_6020[RD], rx_data); break;
    }
}

void Chassis_Init(void)
{
    for(int i = 0;i < 4;i++)
    {
        Motor_Init(&chassis.motor_3508[i], MOTOR_3508_RATE, MOTOR_3508_MAX_RPM, MOTOR_3508_OUT_LIMIT);
        Motor_Init(&chassis.motor_6020[i], MOTOR_6020_RATE, MOTOR_6020_MAX_RPM, MOTOR_6020_OUT_LIMIT);
    }


    for (uint8_t i = 0; i < 4; i++)
    {
        PID_Init(&chassis.pid_motor_3508_omega[i], 0.0f, 0.0f, 0.0f, 0.001f,
                 MOTOR_3508_OUT_LIMIT, MOTOR_3508_OUT_LIMIT, 0.0f, PID_MODE_POSITION);

        PID_Init(&chassis.pid_motor_6020_angle[i], 0.0f, 0.0f, 0.0f, 0.001f,
                 MOTOR_6020_MAX_RPM * RPM_TO_RADPS, MOTOR_6020_MAX_RPM * RPM_TO_RADPS, 0.0f, PID_MODE_POSITION);

        PID_Init(&chassis.pid_motor_6020_omega[i], 0.0f, 0.0f, 0.0f, 0.001f,
                 MOTOR_6020_OUT_LIMIT, MOTOR_6020_OUT_LIMIT, 0.0f, PID_MODE_POSITION);
    }
}

void Chassis_Check(void)
{
    if(chassis.check_count < 500)
    {
        chassis.check_count++;
    }
    else
    {
        chassis.is_ok = 0;
    }
}

void Chassis_1ms_PID_Calc(void)
{

}
