#include "chassis_config.h"
#include "drv_can.h"
#include <math.h>

Chassis_t chassis;

static float To_PI(float angle)
{
    while(angle > PI) angle -= 2.0f * PI;
    while(angle < -PI) angle += 2.0f * PI;
    return angle;
}

static float To_Half_PI(float angle)
{
    angle = To_PI(angle);

    if(angle > PI / 2.0f)       angle -= PI;
    else if(angle < -PI / 2.0f) angle += PI;

    return angle;
}

void Chassis_Init(void)
{
    CAN_Filter_Mask_Config(&hcan1, CAN_FILTER(1) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
    CAN_Filter_Mask_Config(&hcan2, CAN_FILTER(14) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0, 0);

    for(int i = 0; i < 4; i++)
    {
        Motor_Init(&chassis.motor_3508[i], MOTOR_3508_RATE, MOTOR_3508_MAX_RPM, MOTOR_3508_OUT_LIMIT);
        Motor_Init(&chassis.motor_6020[i], MOTOR_6020_RATE, MOTOR_6020_MAX_RPM, MOTOR_6020_OUT_LIMIT);

        PID_Init(&chassis.pid_motor_3508[i], 1200.0f, 0.0f, 0.0f, 0.001f,
                 CHASSIS_3508_OUT_MAX, CHASSIS_3508_OUT_MAX, 0.0f, PID_MODE_POSITION);
        PID_Init(&chassis.pid_motor_6020_angle[i], 20.0f, 0.0f, 0.0f, 0.001f,
                 MOTOR_6020_MAX_RPM * RPM_TO_RADPS, 0.0f, 0.0f, PID_MODE_POSITION);
        PID_Init(&chassis.pid_motor_6020_omega[i], 1000.0f, 0.0f, 0.0f, 0.001f,
                 MOTOR_6020_OUT_LIMIT, 0.0f, 0.0f, PID_MODE_POSITION);
    }

    // 6020 舵向零点：摆正后读的绝对编码器值(0~8191)，掉电不丢，标一次长期有效
    chassis.motor_6020[LU].angle_offset = 329.0f / 8192.0f * 2.0f * PI;
    chassis.motor_6020[LD].angle_offset = 6406.0f / 8192.0f * 2.0f * PI;
    chassis.motor_6020[RD].angle_offset = 1563.0f / 8192.0f * 2.0f * PI;
    chassis.motor_6020[RU].angle_offset = 1079.0f / 8192.0f * 2.0f * PI;

    chassis.mode = Chassis_Disable;
}

void Chassis_Check(void)
{
    for(int i = 0; i < 4; i++)
    {
        Motor_Check(&chassis.motor_3508[i]);
        Motor_Check(&chassis.motor_6020[i]);
    }

    if(chassis.check_count < 500)
    {
        chassis.check_count++;
    }
    else
    {
        chassis.is_ok = 0;
        chassis.mode = Chassis_Disable;
    }
}

void Chassis_Mode_Disable(void)
{
    for(int i = 0; i < 4; i++)
    {
        chassis.motor_3508[i].target_rad_s = 0.0f;
        chassis.motor_3508[i].out = 0.0f;

        chassis.motor_6020[i].target_angle = 0.0f;
        chassis.motor_6020[i].target_rad_s = 0.0f;
        chassis.motor_6020[i].out = 0.0f;

        PID_Reset(&chassis.pid_motor_3508[i]);  
        PID_Reset(&chassis.pid_motor_6020_angle[i]);
        PID_Reset(&chassis.pid_motor_6020_omega[i]);
    }
}

void Chassis_Move_Calc(void)
{
    for(int i = 0; i < 4; i++)
    {
        //舵向电机速度
        float wheel_vx = chassis.vx - chassis.wz * wheel_pos[i].y;
        float wheel_vy = chassis.vy + chassis.wz * wheel_pos[i].x;

        if(wheel_vx * wheel_vx + wheel_vy * wheel_vy < 1e-6f)
        {
            chassis.motor_6020[i].target_angle = chassis.motor_6020[i].now_angle;   //锁住不动
            chassis.motor_3508[i].target_rad_s = 0.0f;
        }
        else
        {
            chassis.motor_6020[i].target_angle = chassis.motor_6020[i].now_angle
                + To_Half_PI(chassis.motor_6020[i].angle_offset - chassis.motor_6020[i].now_angle - atan2f(wheel_vy, wheel_vx));

            chassis.motor_3508[i].target_rad_s = sqrtf(wheel_vx * wheel_vx + wheel_vy * wheel_vy) / WHEEL_R
                * cosf(To_PI(chassis.motor_6020[i].angle_offset - chassis.motor_6020[i].now_angle - atan2f(wheel_vy, wheel_vx)));
        }
    }
}

void Chassis_Data_Update_Callback(void)
{
    switch(chassis.mode)
    {
        case Chassis_Move:
        case Chassis_XiaoTuoLuo:
            Chassis_Move_Calc();
            break;
        case Chassis_Disable:
        default:
            Chassis_Mode_Disable();
            break;
    }
}

void Chassis_PID_Callback(void)
{
    for(int i = 0; i < 4; i++)
    {
        if(chassis.mode == Chassis_Disable)
        {
            chassis.motor_3508[i].out = 0;
            chassis.motor_6020[i].out = 0;
        }
        else
        {
            float motor_3508_out = PIDCompute(&chassis.pid_motor_3508[i],
                                              chassis.motor_3508[i].now_rad_s,
                                              chassis.motor_3508[i].target_rad_s);
            chassis.motor_6020[i].target_rad_s = PIDCompute(&chassis.pid_motor_6020_angle[i],
                                                            chassis.motor_6020[i].now_angle,
                                                            chassis.motor_6020[i].target_angle);
            float motor_6020_out = PIDCompute(&chassis.pid_motor_6020_omega[i],
                                              chassis.motor_6020[i].now_rad_s,
                                              chassis.motor_6020[i].target_rad_s);

            chassis.motor_3508[i].out = chassis.motor_3508[i].is_ok ? (int16_t)motor_3508_out : 0;
            chassis.motor_6020[i].out = chassis.motor_6020[i].is_ok ? (int16_t)motor_6020_out : 0;
        }
    }
}

void Chassis_Send_Callback(void)
{
    // 掉线时也继续发帧、显式发 0，而不是停发依赖电调自身的超时保护(响应慢且不可控)
    if(!chassis.is_ok)
    {
        for(int i = 0; i < 8; i++)
        {
            CAN1_0x200_Tx_Data[i] = 0;
            CAN1_0x1ff_Tx_Data[i] = 0;
        }
    }
    else
    {
        for(int i = 0; i < 4; i++)
        {
            //int16_t out_3508 = chassis.motor_3508[motor3508_order[i]].out;
            //int16_t out_6020 = chassis.motor_6020[motor6020_order[i]].out;

            int16_t out_3508 = chassis.motor_3508[(i + 1) % 4].out;
            //int16_t out_3508 = chassis.vx * 3000;
            int16_t out_6020 = chassis.motor_6020[i].out;

            CAN1_0x200_Tx_Data[2*i]   = (uint8_t)(out_3508 >> 8);
            CAN1_0x200_Tx_Data[2*i+1] = (uint8_t)(out_3508);

            CAN1_0x1ff_Tx_Data[2*i]   = (uint8_t)(out_6020 >> 8);
            CAN1_0x1ff_Tx_Data[2*i+1] = (uint8_t)(out_6020);
        }
    }

    CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);
    CAN_Send_Data(&hcan2, 0x1FE, CAN1_0x1ff_Tx_Data, 8);
}
