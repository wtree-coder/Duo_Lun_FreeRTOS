#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx_hal.h"

#ifndef PI
#define PI 3.1415926535f
#endif

#define RPM_TO_RADPS (2.0f * PI / 60.0f)

#define MOTOR_3508_RATE 19.203f
#define MOTOR_6020_RATE 1

#define MOTOR_3508_MAX_RPM     3000.0f
#define MOTOR_3508_OUT_LIMIT   16384

#define MOTOR_6020_MAX_RPM     320.0f * 0.5f
#define MOTOR_6020_OUT_LIMIT   16384

// 掉线判定阈值：连续这么多次调用 Motor_Check 都没收到反馈，就清空结构体并置 is_ok=0。
// 调用点在 Task_Data_Get，1kHz，所以 20 ≈ 20ms 无反馈即判定掉线。
#define MOTOR_OFFLINE_COUNT    20


typedef struct
{
    //原始电调反馈值
    uint16_t rx_encoder;
    int16_t  rx_rpm;
    int16_t  rx_torque;
    uint8_t  temp;

    //处理后的数据
    float    now_angle;         //弧度制、多圈、未归一化
    float    now_rad_s;
    float    now_torque;

    float    angle_offset;

    int32_t  total_round;      //总圈数
    int32_t  total_encoder;

    float    rate;            // 减速比
    float    max_rpm;         // 最大转速 RPM
    float    output_limit;

    float target_angle;
    float target_rad_s;

    int16_t  out;


    uint16_t check_count;
    uint8_t  is_ok;
}Motor_t;

void Motor_Init(Motor_t *motor, float rate, float max_rpm, float output_limit);
void Motor_Check(Motor_t *motor);
void Motor_Rx_Callback(Motor_t *motor, uint8_t *Rx_Data);

#endif
