#include "motor.h"
#include <string.h>

void Motor_Init(Motor_t *motor, float rate, float max_rpm, float output_limit)
{
    if (motor == NULL) return;

    memset(motor, 0, sizeof(Motor_t));

    motor->rate         = rate;
    motor->max_rpm      = max_rpm;
    motor->output_limit = output_limit;
}

void Motor_Check(Motor_t *motor)
{
    if(motor->check_count < MOTOR_OFFLINE_COUNT)
    {
        motor->check_count++;
    }
    else
    {
        float rate          = motor->rate;
        float max_rpm       = motor->max_rpm;
        float output_limit  = motor->output_limit;
        float angle_offset  = motor->angle_offset;   // 零点标定值必须保留，否则掉线重连后零点丢失
        uint16_t rx_encoder = motor->rx_encoder;
        int32_t speed_encoder = motor->speed_encoder;

        memset(motor, 0, sizeof(Motor_t));

        motor->rate          = rate;
        motor->max_rpm       = max_rpm;
        motor->output_limit  = output_limit;
        motor->angle_offset  = angle_offset;
        motor->rx_encoder    = rx_encoder;
        motor->speed_encoder = speed_encoder;
    }
}

void Motor_Rx_Callback(Motor_t *motor, uint8_t *Rx_Data)
{
    int16_t pre_encoder;
    uint8_t first_frame;
    if (motor == NULL || Rx_Data == NULL) return;

    first_frame = !motor->is_ok;
    motor->check_count = 0;
    motor->is_ok = 1;
    pre_encoder = motor->rx_encoder;

    motor->rx_encoder   = (uint16_t)(Rx_Data[0] << 8  | Rx_Data[1]);            // [0:1] 编码器 0~8191
    motor->rx_rpm       = (int16_t)( Rx_Data[2] << 8  | Rx_Data[3]);            // [2:3] 转速 RPM (转子端)
    motor->rx_torque    = (int16_t)( Rx_Data[4] << 8  | Rx_Data[5]);            // [4:5] 转矩电流
    motor->temp         = Rx_Data[6];                                           // [6]   温度

    int16_t delta_encoder = motor->rx_encoder - pre_encoder;

    if (delta_encoder < -4096)
    {
        motor->total_round++;          // 正转跨越零点
    }
    else if (delta_encoder > 4096)
    {
        motor->total_round--;          // 反转跨越零点
    }

    motor->total_encoder = motor->total_round * 8192 + motor->rx_encoder;

    if(first_frame)
    {
        motor->speed_encoder = motor->total_encoder;
    }

    motor->now_angle = (float)motor->total_encoder / 8192.0f * 2.0f * PI / motor->rate;
    if(motor->rx_rpm != 0)
    {
        motor->now_rad_s = (float)motor->rx_rpm * RPM_TO_RADPS / motor->rate;
    }

    if(++motor->speed_count >= MOTOR_SPEED_WINDOW)
    {
        if(motor->rx_rpm == 0)
        {
            int32_t delta_low_speed_encoder = motor->total_encoder - motor->speed_encoder;

            if(delta_low_speed_encoder <= 1 && delta_low_speed_encoder >= -1)
            {
                motor->now_rad_s = 0.0f;
            }
            else
            {
                motor->now_rad_s = (float)delta_low_speed_encoder * 2.0f * PI / 8192.0f / motor->rate
                                   * (1000.0f / MOTOR_SPEED_WINDOW);
            }
        }
        motor->speed_encoder = motor->total_encoder;
        motor->speed_count = 0;
    }

    motor->now_torque = (float)motor->rx_torque;
}

