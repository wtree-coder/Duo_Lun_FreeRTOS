#ifndef __CHASSIS_CONFIG_H
#define __CHASSIS_CONFIG_H

#include "stm32f4xx_hal.h"
#include "motor.h"

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
    Motor_t motor_3508[4];    
    Motor_t motor_6020[4];
}Chassis_t;



#endif
