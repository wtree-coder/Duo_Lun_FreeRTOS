#ifndef __CHASSIS_CONFIG_H
#define __CHASSIS_CONFIG_H

#include "stm32f4xx_hal.h"
#include "motor.h"
#include "pid.h"

typedef enum
{
    LU = 0,
    LD,
    RU,
    RD,
}Motor_3508_Position;

typedef enum
{
    Chassis_Move = 0,
    Chassis_XiaoTuoLuo = 1,
    Chassis_Disable = 2,
}Chassis_Mode;

typedef struct
{
    float vx;   // 前后速度 (m/s)
    float vy;   // 左右速度 (m/s)
    float wz;   // 旋转速度 (rad/s)

    Motor_t motor_3508[4];
    Motor_t motor_6020[4];

    PIDControllerTypedef pid_motor_3508_omega[4];
    PIDControllerTypedef pid_motor_6020_angle[4];
    PIDControllerTypedef pid_motor_6020_omega[4];

    uint8_t chassis_switch;
    Chassis_Mode mode;

    uint32_t check_count;
    uint8_t is_ok;
}Chassis_t;

void Chassis_Motor_3508_Rx_Callback(CAN_RxHeaderTypeDef *rxbuf, uint8_t *rx_data);
void Chassis_Motor_6020_Rx_Callback(CAN_RxHeaderTypeDef *rxbuf, uint8_t *rx_data);
void Chassis_Init(void);
void Chassis_Check(void);
void Chassis_1ms_PID_Calc(void);

extern Chassis_t chassis;


#endif
