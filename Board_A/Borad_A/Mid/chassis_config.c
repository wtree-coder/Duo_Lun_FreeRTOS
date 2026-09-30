#include "chassis_config.h"
#include "drv_can.h"
#include <math.h>

Chassis_t chassis;

static float angle_wrap(float angle)
{
    while(angle > PI) angle -= 2.0f * PI;
    while(angle < -PI) angle += 2.0f * PI;
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

        PID_Init(&chassis.pid_motor_3508[i], 0.0f, 0.0f, 0.0f, 0.001f,
                 MOTOR_3508_OUT_LIMIT, MOTOR_3508_OUT_LIMIT, 0.0f, PID_MODE_POSITION);
        PID_Init(&chassis.pid_motor_6020_angle[i], 20.0f, 0.0f, 0.0f, 0.001f,
                 MOTOR_6020_MAX_RPM * RPM_TO_RADPS, 0.0f, 0.0f, PID_MODE_POSITION);
        PID_Init(&chassis.pid_motor_6020_omega[i], 1000.0f, 0.0f, 0.0f, 0.001f,
                 MOTOR_6020_OUT_LIMIT, 0.0f, 0.0f, PID_MODE_POSITION);
    }

    // 6020 舵向零点：摆正后读的绝对编码器值(0~8191)，掉电不丢，标一次长期有效
    chassis.motor_6020[LU].angle_offset = 254.0f  / 8192.0f * 2.0f * PI;
    chassis.motor_6020[LD].angle_offset = 6406.0f / 8192.0f * 2.0f * PI;
    chassis.motor_6020[RD].angle_offset = 1700.0f / 8192.0f * 2.0f * PI;
    chassis.motor_6020[RU].angle_offset = 1191.0f / 8192.0f * 2.0f * PI;

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
        chassis.motor_6020[i].target_angle = 0.0f;
        chassis.motor_6020[i].target_rad_s = 0.0f;
        PID_Reset(&chassis.pid_motor_3508[i]);
        PID_Reset(&chassis.pid_motor_6020_angle[i]);
        PID_Reset(&chassis.pid_motor_6020_omega[i]);
    }
}

void Chassis_Stable(void)
{
    // 四轮朝外(径向)。车体系里模块转向与角度符号相反，故整体取反
    const float stable_angle[4] =
    {
        [LU] = 7.0f * PI / 4.0f,   //315°
        [LD] = 5.0f * PI / 4.0f,   //225°
        [RD] = 3.0f * PI / 4.0f,   //135°
        [RU] = 1.0f * PI / 4.0f,   // 45°
    };

    for(int i = 0; i < 4; i++)
    {
        float module_angle = chassis.motor_6020[i].now_angle - chassis.motor_6020[i].angle_offset;
        float delta = angle_wrap(stable_angle[i] - module_angle);
        chassis.motor_6020[i].target_angle = chassis.motor_6020[i].now_angle + delta;
        chassis.motor_3508[i].target_rad_s = 0.0f;
    }
}

void Chassis_XiaoTuoLuo_Pose(void)
{
    // 小陀螺姿态：四轮切向(垂直于到中心的连线)
    // 该姿态下四个 3508 同号驱动 -> 合力为 0、合力矩同向叠加 -> 纯自转
    // 数值 = Chassis_Stable 那组 -90°(同样因为符号相反，所以是减不是加)
    const float xiaotuoluo_angle[4] =
    {
        [LU] = 5.0f * PI / 4.0f,   //225°
        [LD] = 3.0f * PI / 4.0f,   //135°
        [RD] = 1.0f * PI / 4.0f,   // 45°
        [RU] = 7.0f * PI / 4.0f,   //315°
    };

    for(int i = 0; i < 4; i++)
    {
        float module_angle = chassis.motor_6020[i].now_angle - chassis.motor_6020[i].angle_offset;
        float delta = angle_wrap(xiaotuoluo_angle[i] - module_angle);
        chassis.motor_6020[i].target_angle = chassis.motor_6020[i].now_angle + delta;
        chassis.motor_3508[i].target_rad_s = 0.0f;
    }
}
void Move_Calc(void)
{
    if(fabsf(chassis.vx) < 1e-4f &&
       fabsf(chassis.vy) < 1e-4f &&
       fabsf(chassis.wz) < 1e-4f)
    {
        Chassis_Stable();
        return;
    }

    const float half = LENGTH / 2.0f;
    const float wheel_x[4] =
    {
        [LU] =  half, [LD] = -half, [RD] = -half, [RU] =  half,
    };
    const float wheel_y[4] =
    {
        [LU] =  half, [LD] =  half, [RD] = -half, [RU] = -half,
    };

    for(int i = 0; i < 4; i++)
    {
        float wheel_vx = chassis.vx - chassis.wz * wheel_y[i];
        float wheel_vy = chassis.vy + chassis.wz * wheel_x[i];

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

void Xiao_Tuo_Luo_Calc(void)
{
    const float half = LENGTH / 2.0f;
    const float wheel_x[4] =
    {
        [LU] =  half, [LD] = -half, [RD] = -half, [RU] =  half,
    };
    const float wheel_y[4] =
    {
        [LU] =  half, [LD] =  half, [RD] = -half, [RU] = -half,
    };

    for(int i = 0; i < 4; i++)
    {
        float wheel_vx = chassis.vx - chassis.wz * wheel_y[i];
        float wheel_vy = chassis.vy + chassis.wz * wheel_x[i];

        float module_angle = chassis.motor_6020[i].now_angle - chassis.motor_6020[i].angle_offset;
        float steer_target = atan2f(wheel_vy, wheel_vx);
        float drive_target = sqrtf(wheel_vx * wheel_vx + wheel_vy * wheel_vy) / 0.1016f;

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

void Chassis_Data_Update_Callback(void)
{
    switch(chassis.mode)
    {
        case Chassis_Move:
            //Chassis_Stable();   // 自锁成 X 型
            Chassis_XiaoTuoLuo_Pose();
            break;
        case Chassis_XiaoTuoLuo:
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
            float drive_out = PIDCompute(&chassis.pid_motor_3508[i],
                                         chassis.motor_3508[i].now_rad_s,
                                         chassis.motor_3508[i].target_rad_s);
            chassis.motor_6020[i].target_rad_s = PIDCompute(&chassis.pid_motor_6020_angle[i],
                                                            chassis.motor_6020[i].now_angle,
                                                            chassis.motor_6020[i].target_angle);
            float steer_out = PIDCompute(&chassis.pid_motor_6020_omega[i],
                                         chassis.motor_6020[i].now_rad_s,
                                         chassis.motor_6020[i].target_rad_s);

            chassis.motor_3508[i].out = chassis.motor_3508[i].is_ok ? (int16_t)drive_out : 0;
            chassis.motor_6020[i].out = chassis.motor_6020[i].is_ok ? (int16_t)steer_out : 0;
        }
    }
}

void Chassis_Send_Callback(void)
{
    if(!chassis.is_ok) return;
    // 0x200 帧槽位按 3508 电调 ID1~4：ID1=LD, ID2=RD, ID3=RU, ID4=LU
    //static const Chassis_Pos motor3508_order[4] = { LD, RD, RU, LU };
    // 6020 帧槽位按电调 ID1~4：ID1=LU, ID2=LD, ID3=RD, ID4=RU
    //static const Chassis_Pos motor6020_order[4] = { LU, LD, RD, RU };

    for(int i = 0; i < 4; i++)
    {
        //int16_t out_3508 = chassis.motor_3508[motor3508_order[i]].out;
        //int16_t out_6020 = chassis.motor_6020[motor6020_order[i]].out;

        int16_t out_3508 = chassis.vx * 8000;
        int16_t out_6020 = chassis.motor_6020[i].out;

        CAN1_0x200_Tx_Data[2*i]   = (uint8_t)(out_3508 >> 8);
        CAN1_0x200_Tx_Data[2*i+1] = (uint8_t)(out_3508);

        CAN1_0x1ff_Tx_Data[2*i]   = (uint8_t)(out_6020 >> 8);
        CAN1_0x1ff_Tx_Data[2*i+1] = (uint8_t)(out_6020);
    }

    CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);
    CAN_Send_Data(&hcan2, 0x1FE, CAN1_0x1ff_Tx_Data, 8);
}
