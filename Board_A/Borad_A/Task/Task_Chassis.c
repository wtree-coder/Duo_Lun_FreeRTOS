#include "Task_Chassis.h"
#include "chassis_config.h"

void Task_Chassis(void *argument)
{
    Chassis_Init();
    osDelay(500);
    for(;;)
    {
        Chassis_Data_Update_Callback();
        Chassis_PID_Callback();
        Chassis_Send_Callback();
        osDelay(1);
    }
}
