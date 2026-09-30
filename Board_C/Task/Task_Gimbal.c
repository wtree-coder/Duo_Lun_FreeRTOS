#include "Task_Gimbal.h"
#include "gimbal_config.h"

void Task_Gimbal(void *argument)
{
    Gimbal_Init();
    uint32_t tick = osKernelGetTickCount();
    for(;;)
    {
        Gimbal_Mode_Choose();
        Gimbal_Data_Update();
        Gimbal_PID_Calc();
        Gimbal_CAN_Send_Callback();
        tick++;
        osDelayUntil(tick);
    }
}
