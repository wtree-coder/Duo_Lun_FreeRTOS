#include "Task_Chassis.h"
#include "chassis_config.h"

void Task_Chassis(void *argument)
{
    Chassis_Init();
    for(;;)
    {
        osDelay(1);
    }
}


