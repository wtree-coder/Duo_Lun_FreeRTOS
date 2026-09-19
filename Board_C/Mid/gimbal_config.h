#ifndef __GIMBAL_CONFIG_H
#define __GIMBAL_CONFIG_H

#include "stm32f4xx_hal.h"
#include "motor.h"

typedef struct
{
    Motor_t motor_6020;

}Gimbal_t;

void Gimbal_Init(void);
void Gimbal_6020_Rx_Callback(CAN_RxHeaderTypeDef *rx_header, uint8_t rx_data);

extern Gimbal_t gimbal;


#endif
