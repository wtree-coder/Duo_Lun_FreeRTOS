#include "Task_Data_Get.h"
#include "vofa.h"
#include "vofa_config.h"
#include "usart.h"
#include "drv_can.h"
#include "communication.h"
#include "chassis_config.h"

void Task_Data_Get(void *argument)
{   
    CAN_Init(&hcan1);
    CAN_Init(&hcan2);
    
    Vofa_Init(&huart3);

    //0x7FF : 11个1
    CAN_Filter_Mask_Config(&hcan1, CAN_FILTER(0) | CAN_FIFO_0, 0x300, 0x7FF);       // 只放行 0x300
    CAN_Filter_Mask_Config(&hcan1, CAN_FILTER(1) | CAN_FIFO_0, 0x200, 0x7F0);       // 放行 0x200~0x20F
    CAN_Filter_Mask_Config(&hcan2, CAN_FILTER(14) | CAN_FIFO_0, 0x1ff, 0x7F0);      // 放行 0x200~0x20F
    
    uint32_t tick = osKernelGetTickCount();
    for(;;)
    {
        Chassis_Check();
        Vofa_20ms_Send_Callback();     
        
        tick++;
        osDelayUntil(tick);
    }
}
