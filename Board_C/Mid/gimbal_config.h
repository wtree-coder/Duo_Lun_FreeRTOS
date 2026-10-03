#ifndef __GIMBAL_CONFIG_H
#define __GIMBAL_CONFIG_H

#include "stm32f4xx_hal.h"
#include "motor.h"
#include "pid.h"

#define MOTOR_6020_OMEGA_KF 130
typedef enum
{
    GIMBAL_DISABLE = 0,
    GIMBAL_MOVE,
    GIMBAL_SHOOT,
}Gimbal_Mode_t;

typedef struct
{
    float total_yaw;
    float target_yaw;        // IMU 绝对 yaw 目标 (rad)
    float delta_angle;       // 云台相对底盘角 (rad)，由 Gimbal_Data_Update 每 1ms 更新，已归一化到 (-PI, PI]

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
void Gimbal_CAN_Send_Callback(void);


extern Gimbal_t gimbal;


#endif
