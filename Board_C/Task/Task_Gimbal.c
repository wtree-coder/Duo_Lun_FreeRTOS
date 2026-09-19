#include "Task_Gimbal.h"
#include "gimbal_config.h"

void Task_Gimbal(void *argument)
{
    Gimbal_Init();
    uint32_t tick = osKernelGetTickCount();
    for(;;)
    {
        tick++;
        osDelayUntil(tick);
    }
}



