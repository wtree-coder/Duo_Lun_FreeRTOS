#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#include "stm32f4xx_hal.h"

#define V_MAX            2.0f     //m/s
#define WZ_MAX           3.0f    // rad/s

void Communication_Init(void);
void Message_Update(uint8_t *rx_data);

#endif
