#include "chassis_config.h"
#include "drv_can.h"
#include <math.h>

#define CAN_DRIVE_3508   &hcan1
#define CAN_STEER_6020   &hcan2

Chassis_t chassis;

static float angle_wrap(float angle)
{
    while(angle > PI) angle -= 2.0f * PI;
    while(angle < -PI) angle += 2.0f * PI;
    return angle;
}

void Chassis_Init(void)
{
    for(int i = 0; i < 4; i++)
    {
        Motor_Init(&chassis.motor_3508[i], MOTOR_3508_RATE, MOTOR_3508_MAX_RPM, MOTOR_3508_OUT_LIMIT);
        Motor_Init(&chassis.motor_6020[i], MOTOR_6020_RATE, MOTOR_6020_MAX_RPM, MOTOR_6020_OUT_LIMIT);

        PID_Init(&chassis.pid_motor_3508[i], 20.0f, 1000.0f, 0.0f, 0.001f,
                 MOTOR_3508_OUT_LIMIT, MOTOR_3508_OUT_LIMIT, 0.0f, PID_MODE_POSITION);
        PID_Init(&chassis.pid_motor_6020[i], 200.0f, 0.0f, 5.0f, 0.001f,
                 MOTOR_6020_OUT_LIMIT, MOTOR_6020_OUT_LIMIT, 0.0f, PID_MODE_POSITION);
    }

    chassis.mode = Chassis_Disable;
}

void Chassis_Check(void)
{
    chassis.is_ok = 1;

    for(int i = 0; i < 4; i++)
    {
        Motor_Check(&chassis.motor_3508[i]);
        Motor_Check(&chassis.motor_6020[i]);

        if((!chassis.motor_3508[i].is_ok) || (!chassis.motor_6020[i].is_ok))
        {
            chassis.is_ok = 0;
        }
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

static void Chassis_Mode_Disable(void)
{
    for(int i = 0; i < 4; i++)
    {
        chassis.motor_3508[i].target_rad_s = 0.0f;
        chassis.motor_6020[i].target_angle = 0.0f;
        PID_Reset(&chassis.pid_motor_3508[i]);
        PID_Reset(&chassis.pid_motor_6020[i]);
    }
}

static void Chassis_Swerve_Calc(float vx, float vy, float wz)
{
    const float half = LENGTH / 2.0f;
    const float wheel_x[4] = { half,  half, -half, -half};
    const float wheel_y[4] = { half, -half, -half,  half};

    for(int i = 0; i < 4; i++)
    {
        float wheel_vx = vx - wz * wheel_y[i];
        float wheel_vy = vy + wz * wheel_x[i];

        float module_angle = chassis.motor_6020[i].now_angle - chassis.motor_6020[i].angle_offset;
        float steer_target;
        float drive_target;

        if(fabsf(wheel_vx) < 1e-4f && fabsf(wheel_vy) < 1e-4f)
        {
            steer_target = module_angle;
            drive_target = 0.0f;
        }
        else
        {
            steer_target = atan2f(wheel_vy, wheel_vx);
            drive_target = sqrtf(wheel_vx * wheel_vx + wheel_vy * wheel_vy) / 0.1016f;
        }

        float delta = angle_wrap(steer_target - module_angle);

        if(delta > PI / 2.0f)
        {
            delta -= PI;
            drive_target = -drive_target;
        }
        else if(delta < -PI / 2.0f)
        {
            delta += PI;
            drive_target = -drive_target;
        }

        chassis.motor_6020[i].target_angle = chassis.motor_6020[i].now_angle + delta;
        chassis.motor_3508[i].target_rad_s = drive_target;
    }
}

void Chassis_Mode_Choose(void)
{
    Chassis_Check();

    switch(chassis.mode)
    {
        case Chassis_Move:
            Chassis_Swerve_Calc(chassis.vx, chassis.vy, chassis.wz);
            break;
        case Chassis_XiaoTuoLuo:
            Chassis_Swerve_Calc(0.0f, 0.0f, chassis.wz);
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
            float drive_rpm = chassis.motor_3508[i].target_rad_s * 60.0f / (2.0f * PI)
                            * chassis.motor_3508[i].rate;
            float drive_out = PIDCompute(&chassis.pid_motor_3508[i],
                                         chassis.motor_3508[i].rx_rpm, drive_rpm);
            float steer_out = PIDCompute(&chassis.pid_motor_6020[i],
                                         chassis.motor_6020[i].now_angle,
                                         chassis.motor_6020[i].target_angle);

            chassis.motor_3508[i].out = (int16_t)drive_out;
            chassis.motor_6020[i].out = (int16_t)steer_out;
        }
    }
}

void Chassis_Task_1ms_Callback(void)
{
    Chassis_Mode_Choose();
    Chassis_PID_Callback();
    Chassis_Send_Callback();
}

void Chassis_Send_Callback(void)
{
    for(int i = 0; i < 4; i++)
    {
        CAN1_0x200_Tx_Data[2*i]   = (uint8_t)(chassis.motor_3508[i].out >> 8);
        CAN1_0x200_Tx_Data[2*i+1] = (uint8_t)(chassis.motor_3508[i].out);
        CAN1_0x1ff_Tx_Data[2*i]   = (uint8_t)(chassis.motor_6020[i].out >> 8);
        CAN1_0x1ff_Tx_Data[2*i+1] = (uint8_t)(chassis.motor_6020[i].out);
    }

    CAN_Send_Data(CAN_DRIVE_3508, 0x200, CAN1_0x200_Tx_Data, 8);
    CAN_Send_Data(CAN_STEER_6020, 0x1FF, CAN1_0x1ff_Tx_Data, 8);
}
