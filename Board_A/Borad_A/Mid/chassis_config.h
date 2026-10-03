#ifndef __CHASSIS_CONFIG_H
#define __CHASSIS_CONFIG_H

#include "stm32f4xx_hal.h"
#include "motor.h"
#include "pid.h"

#define LENGTH      0.5f       //正方形边长:0.5m
//#define WHEEL_R     0.1016f    //单位:m
#define WHEEL_R 0.05f
#define CHASSIS_3508_OUT_MAX  8000

typedef enum
{
    Chassis_Disable = 0,
    Chassis_XiaoTuoLuo,
    Chassis_Move,
}Chassis_Mode;

typedef enum
{
                      //舵向6020、电机轮向3508电机
    LU = 0,           //0x205、0x204
    LD,               //0x206、0x201
    RD,               //0x207、0x202
    RU,               //0x208、0x203
}Chassis_Pos;

typedef struct
{
    float x;
    float y;
}Chassis_WheelPos_t;

static const Chassis_WheelPos_t wheel_pos[4] =
{
    [LU] = {  LENGTH / 2.0f,  LENGTH / 2.0f },   //左前
    [LD] = { -LENGTH / 2.0f,  LENGTH / 2.0f },   //左后
    [RD] = { -LENGTH / 2.0f, -LENGTH / 2.0f },   //右后
    [RU] = {  LENGTH / 2.0f, -LENGTH / 2.0f },   //右前
};



typedef struct
{
    Chassis_Mode mode;

    Motor_t motor_3508[4];
    Motor_t motor_6020[4];
    PIDControllerTypedef pid_motor_3508[4];
    PIDControllerTypedef pid_motor_6020_angle[4];
    PIDControllerTypedef pid_motor_6020_omega[4];

    float vx;
    float vy;
    float wz;

    uint32_t check_count;
    uint8_t is_ok;
} Chassis_t;

extern Chassis_t chassis;

void Chassis_Init(void);
void Chassis_Check(void);

void Chassis_Mode_Disable(void);
void Chassis_Move_Calc(void);
void Chassis_Data_Update_Callback(void);
void Chassis_PID_Callback(void);
void Chassis_Send_Callback(void);

#endif
