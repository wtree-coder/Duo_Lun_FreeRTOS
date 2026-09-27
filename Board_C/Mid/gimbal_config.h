#ifndef __GIMBAL_CONFIG_H
#define __GIMBAL_CONFIG_H

#include "stm32f4xx_hal.h"
#include "motor.h"
#include "pid.h"

typedef enum
{
    GIMBAL_DISABLE = 0,
    GIMBAL_MOVE,
    GIMBAL_SHOOT,
}Gimbal_Mode_t;

/* 云台朝正前方时实测的 yaw 6020 编码器角 (rad)，标定后填入 */
#define GIMBAL_YAW_FORWARD_OFFSET  0.0f

typedef struct
{
    float total_yaw;
    float target_yaw;        // IMU 绝对 yaw 目标 (rad)
    float delta_angle;       // 云台相对底盘角 (rad) = yaw编码器角 - 正前零偏

    Motor_t motor_6020;
    Gimbal_Mode_t mode;

    PIDControllerTypedef pid_motor_6020_angle;
    PIDControllerTypedef pid_motor_6020_omega;
}Gimbal_t;

void Gimbal_Init(void);
void Gimbal_6020_Rx_Callback(CAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data);
void Gimbal_Check(void);

void Gimbal_Mode_Choose(void);
void Gimbal_Data_Update(void);
void Gimbal_PID_Calc(void);

void Gimbal_Task_1ms_Callback(void);

extern Gimbal_t gimbal;


#endif
