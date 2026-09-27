#include "Task_Communicate.h"
#include "drv_can.h"
#include "can.h"
#include "communication.h"

void Task_Communicate(void *argument)
{
    CAN_Init(&hcan1);
    //C板只发送数据
    //CAN_Filter_Mask_Config()
    uint32_t tick = osKernelGetTickCount();
    for(;;)
    {
        C_SendMessage_To_A_1ms_Callback();
        
        tick++;
        osDelayUntil(tick);
    }
}

