#include "gimbal_config.h"
#include "drv_can.h"
#include "motor.h"
#include "pid.h"
#include "imu.h"
#include "DR16.h"
#include <math.h>

Gimbal_t gimbal;

float To_2PI(float angle)
{
    while(angle < -PI) angle += 2*PI;
    while(angle > PI) angle -= 2*PI;
    return angle;
}

void Gimbal_Total_Yaw_Update(void)
{
    static int total_round = 0;
    static float last_yaw = 0.0f;
    if(imu_euler_data.yaw - last_yaw > PI) total_round--;
    else if(imu_euler_data.yaw - last_yaw < -PI) total_round++;
    last_yaw = imu_euler_data.yaw;

    gimbal.total_yaw = total_round*2*PI + imu_euler_data.yaw;
}

void Gimbal_Init(void)
{
    CAN_Init(&hcan1);
    CAN_Filter_Mask_Config(&hcan1, CAN_FILTER(0) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0, 0);

    Motor_Init(&gimbal.motor_6020, MOTOR_6020_RATE, MOTOR_6020_MAX_RPM, MOTOR_6020_OUT_LIMIT);

    gimbal.mode           = GIMBAL_DISABLE;    

    //云台正对值
    gimbal.motor_6020.angle_offset = 4870.0f / 8192.0f * 2.0f * PI;

    //待精调
    PID_Init(&gimbal.pid_motor_6020_angle, 90.0f, 0.0f, 0.0f, 0.001f, MOTOR_6020_MAX_RPM * RPM_TO_RADPS, 0.2f, 0.0f, PID_MODE_POSITION);
    //PID_SetIntegralSeparationThreshold(&gimbal.pid_motor_6020_angle, 0.0f);
    PID_Init(&gimbal.pid_motor_6020_omega, 1200.0f, 0.0f, 0.0f, 0.001f, MOTOR_6020_OUT_LIMIT, 0.0f, 0.0f, PID_MODE_POSITION);
}

void Gimbal_6020_Rx_Callback(CAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data)
{
    if (rx_header->StdId == 0x205)    // 6020 反馈 ID（接收）
    {
        Motor_Rx_Callback(&gimbal.motor_6020, rx_data);
    }
}

void Gimbal_Check(void)
{
    Motor_Check(&gimbal.motor_6020);
}

void Gimbal_Mode_Choose(void)
{
    //未开遥控器
    if (!DR16_Data.is_ok)
    {
        gimbal.mode = GIMBAL_DISABLE;
        return;
    }

    //开启遥控器
    switch (DR16_Data.Gimbal_Switch)
    {
        case DR16_SWITCH_UP:   gimbal.mode = GIMBAL_MOVE;    break;
        case DR16_SWITCH_MID:  gimbal.mode = GIMBAL_MOVE;    break;
        case DR16_SWITCH_DOWN: gimbal.mode = GIMBAL_DISABLE; break;
        default: break;
    }
}


void Gimbal_Data_Update(void)
{
    Gimbal_Total_Yaw_Update();

    float delta_gimbal_angle = gimbal.motor_6020.now_angle - gimbal.motor_6020.angle_offset;
    //归一到[-PI, PI]
    gimbal.delta_angle = atan2f(sinf(delta_gimbal_angle), cosf(delta_gimbal_angle));

    //失能则不进PID_Calc(), 相关数据于此清零
    if (gimbal.mode != GIMBAL_MOVE && gimbal.mode != GIMBAL_SHOOT) gimbal.motor_6020.out = 0;

    switch (gimbal.mode)
    {
        case GIMBAL_MOVE:
        case GIMBAL_SHOOT:
            //遥控器控制
            gimbal.target_yaw -= DR16_XiaoZhun(DR16_Data.Right_X) * 0.012f;
            break;

        case GIMBAL_DISABLE:
        default:
            gimbal.target_yaw = gimbal.total_yaw;
            break;
    }
}

void Gimbal_PID_Calc(void)
{
    if((gimbal.mode != GIMBAL_MOVE) && (gimbal.mode != GIMBAL_SHOOT)) return;
    gimbal.motor_6020.target_rad_s = PIDCompute(&gimbal.pid_motor_6020_angle,
                                                gimbal.total_yaw,
                                                gimbal.target_yaw);
    //gimbal.motor_6020.target_rad_s = -DR16_XiaoZhun(DR16_Data.Right_X) * MOTOR_6020_MAX_RPM * RPM_TO_RADPS;
    gimbal.motor_6020.out = PIDCompute(&gimbal.pid_motor_6020_omega,
                                        gimbal.motor_6020.now_rad_s,
                                        gimbal.motor_6020.target_rad_s) + gimbal.motor_6020.target_rad_s * MOTOR_6020_OMEGA_KF;
                                        
}

void Gimbal_CAN_Send_Callback(void)
{
    int16_t out = gimbal.motor_6020.is_ok ? gimbal.motor_6020.out : 0;

    uint8_t txbuf[8] = {0};

    txbuf[0] = out >> 8;
    txbuf[1] = out;
    CAN_Send_Data(&hcan1, 0x1FE, txbuf, 8);
}
