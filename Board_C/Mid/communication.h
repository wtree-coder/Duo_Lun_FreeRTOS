#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#include "stm32f4xx_hal.h"

#define MAX_V           3.0f         //3m/s
#define MAX_W           15.0f        //15.0 rad/s
void C_SendMessage_To_A_1ms_Callback(void);

#endif
