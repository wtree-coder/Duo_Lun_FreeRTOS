#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#include "stm32f4xx_hal.h"

#define MAX_V           3000.0f      // 摇杆满量程(-1~1)对应的速度量化值(3m/s)

void C_SendMessage_To_A_1ms_Callback(void);

#endif
