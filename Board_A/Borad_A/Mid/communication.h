#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#include "stm32f4xx_hal.h"

#define V_MAX            2.0f
#define WZ_MAX_RADPS     3.0f
#define REL_ANGLE_SCALE  1000.0f

void Message_Update(uint8_t *rx_data);

#endif
