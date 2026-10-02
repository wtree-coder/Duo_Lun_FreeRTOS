#include "Task_Data_Get.h"
#include "usart.h"
#include "vofa.h"
#include "drv_can.h"
#include "vofa_config.h"
#include "communication.h"
#include "chassis_config.h"

void Task_Data_Get(void *argument)
{
    Vofa_Init(&huart6);

    CAN_Init(&hcan1);
    CAN_Init(&hcan2);
    //Communication_Init();
    //can1无掩码
    CAN_Filter_Mask_Config(&hcan1, CAN_FILTER(0) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
    
    for(;;)
    {
        Chassis_Check();
        
        Vofa_20ms_Send_Callback();
        osDelay(1);
    }
}
