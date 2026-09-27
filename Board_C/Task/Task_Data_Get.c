#include "Task_Data_Get.h"
#include "vofa.h"
#include "vofa_config.h"
#include "usart.h"
#include "DR16.h"
#include "gimbal_config.h"

void Task_Data_Get(void *argument)
{    
    //两个都是串口DMA接收
    //大疆C板丝印UART1对应UART6
    //大疆C板丝印UART2对应UART1
    Vofa_Init(&huart1);
    DR16_Init(&huart3);    

    uint32_t tick = osKernelGetTickCount();
    for(;;)
    {
        DR16_Check();
        Gimbal_Check();
        
        Vofa_20ms_Send_Callback();     
        
        tick++;
        osDelayUntil(tick);

    }
}
