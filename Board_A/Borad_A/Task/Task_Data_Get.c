#include "Task_Data_Get.h"
#include "usart.h"
#include "vofa.h"
#include "drv_can.h"
#include "vofa_config.h"
#include "communication.h"

void Task_Data_Get(void *argument)
{
    Vofa_Init(&huart6);

    CAN_Init(&hcan1);
    CAN_Init(&hcan2);
    Communication_Init();

    for(;;)
    {
        Vofa_20ms_Send_Callback();
        osDelay(1);
    }
}
