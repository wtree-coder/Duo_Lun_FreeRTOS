#include "gimbal_config.h"
#include "drv_can.h"
#include "motor.h"
#include "pid.h"
#include "imu.h"
#include "DR16.h"
#include <math.h>

Gimbal_t gimbal;

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
    gimbal.motor_6020.angle_offset = -1.0f;    // 必须放在 Motor_Init 之后，Motor_Init 会 memset 整个结构体

    PID_Init(&gimbal.pid_motor_6020_angle, 20.0f, 0.0f, 0.0f, 0.001f, MOTOR_6020_MAX_RPM * RPM_TO_RADPS, 0.0f, 0.0f, PID_MODE_POSITION);
    PID_Init(&gimbal.pid_motor_6020_omega, 1000.0f, 0.0f, 0.0f, 0.001f, MOTOR_6020_OUT_LIMIT, 0.0f, 0.0f, PID_MODE_POSITION);
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
    if (!DR16_Data.is_ok)
    {
        gimbal.mode = GIMBAL_DISABLE;
        return;
    }

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

    // 云台相对底盘偏角：多圈 now_angle 减去零点标定值，再归一化到 (-PI, PI]。
    // now_angle 上电时 total_round 恒为 0，同一物理角度会相差 2π 的整数倍
    // （-2.8 与 +3.48 是同一位置），不归一化会让 wz 打满。
    float d = gimbal.motor_6020.now_angle - gimbal.motor_6020.angle_offset;
    gimbal.delta_angle = atan2f(sinf(d), cosf(d));

    // 只有 GIMBAL_MOVE 会跑 PID 产生输出（见 Gimbal_PID_Calc 的 mode 判断），
    // 其余模式一律显式清零，否则 out 会停在上一帧指令上。
    if (gimbal.mode != GIMBAL_MOVE)
    {
        gimbal.motor_6020.out = 0;
        gimbal.motor_6020.target_rad_s = 0.0f;
    }

    switch (gimbal.mode)
    {
        case GIMBAL_MOVE:
        case GIMBAL_SHOOT:
            gimbal.target_yaw -= DR16_XiaoZhun(DR16_Data.Right_X) * 0.008f; //二者不一样的值的来源
            break;

        default:
            gimbal.target_yaw = gimbal.total_yaw; // 进入之前target一直都是和imu相等的，进入Move模式不会突变
            break;
    }
}

void Gimbal_PID_Calc(void)
{
    if((gimbal.mode != GIMBAL_MOVE)) return;
    gimbal.motor_6020.target_rad_s = PIDCompute(&gimbal.pid_motor_6020_angle,
                                                gimbal.total_yaw,
                                                gimbal.target_yaw);
    //gimbal.motor_6020.target_rad_s = -DR16_XiaoZhun(DR16_Data.Right_X) * MOTOR_6020_MAX_RPM * RPM_TO_RADPS;
    gimbal.motor_6020.out = PIDCompute(&gimbal.pid_motor_6020_omega,
                                        gimbal.motor_6020.now_rad_s,
                                        gimbal.motor_6020.target_rad_s);
}

void Gimbal_CAN_Send_Callback(void)
{
    // 掉了反馈要显式发 0，而不是停发：停发只能等电调自己的失联超时，
    // 那段窗口里电机执行的还是上一帧指令。非 MOVE 模式下 out 已被清零，
    // 所以这里照常发出去就是安全的。
    int16_t out = gimbal.motor_6020.is_ok ? gimbal.motor_6020.out : 0;

    uint8_t txbuf[8] = {0};

    txbuf[0] = out >> 8;
    txbuf[1] = out;
    CAN_Send_Data(&hcan1, 0x1FE, txbuf, 8);
}
