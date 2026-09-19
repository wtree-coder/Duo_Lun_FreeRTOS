#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#include "stm32f4xx_hal.h"

#define V_MAX           2.0f    // 满量程线速度 3 m/s
#define WZ_MAX_RADPS    3.0f    // 满量程角速度 3 rad/s

typedef enum
{
    DR16_SWITCH_UP = 1,
    DR16_SWITCH_DOWN = 2,
    DR16_SWITCH_MID = 3,
} Enum_DR16_Switch_Status;

void A_GetMessage_From_C(uint8_t *data);

#endif
