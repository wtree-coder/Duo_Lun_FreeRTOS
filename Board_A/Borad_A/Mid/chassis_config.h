#ifndef __CHASSIS_CONFIG_H
#define __CHASSIS_CONFIG_H

#include "stm32f4xx_hal.h"
#include "motor.h"
#include "pid.h"

#define LENGTH 0.5   //正方形边长0.5m

typedef enum
{
    LU = 0,
    LD,
    RU,
    RD,
}Chassis_Pos;

typedef enum
{
    Chassis_Disable = 0,
    Chassis_XiaoTuoLuo,
    Chassis_Move,
}Chassis_Mode;

typedef struct
{
    Chassis_Mode mode;

    Motor_t motor_3508[4];
    Motor_t motor_6020[4];
    PIDControllerTypedef pid_motor_3508[4];
    PIDControllerTypedef pid_motor_6020[4];

    float vx;
    float vy;
    float wz;

    uint32_t check_count;
    uint8_t is_ok;
} Chassis_t;

extern Chassis_t chassis;

void Chassis_Init(void);
void Chassis_Check(void);
void Chassis_Task_1ms_Callback(void);
void Chassis_Mode_Choose(void);
void Chassis_PID_Callback(void);
void Chassis_Send_Callback(void);

#endif
