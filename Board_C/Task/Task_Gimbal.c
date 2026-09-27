#include "Task_Gimbal.h"
#include "gimbal_config.h"

void Task_Gimbal(void *argument)
{
    Gimbal_Init();
    uint32_t tick = osKernelGetTickCount();
    for(;;)
    {
        Gimbal_Task_1ms_Callback();
        tick++;
        osDelayUntil(tick);
    }
}
