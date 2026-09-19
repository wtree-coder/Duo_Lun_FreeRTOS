#include "vofa.h"
#include <string.h>
#include "usart.h"

Vofa_t vofa;

void Vofa_Init(UART_HandleTypeDef *huart)
{
    vofa.huart = huart;   // 记住绑定的串口

    //还没有使用上位机实时更改数据
    HAL_UARTEx_ReceiveToIdle_DMA(huart, vofa.Rx_Buff, VOFA_RX_BUFF_SIZE);
}

void Vofa_Set_Data(int Number, ...)
{
    va_list data_ptr;
    va_start(data_ptr, Number);
    
    if (Number > 12) Number = 12; // 保护数组不越界
    
    for (int i = 0; i < Number; i++)
    {
        // va_arg: 用 void* 取出再强转为 const float*
        // 可变参数传的是指针，用 void* 是因为不检查具体类型，传 float*、int* 等任意指针都能通用接住
        vofa.Tx_Data_Ptrs[i] = (const float *)va_arg(data_ptr, void *);
    }
    vofa.Tx_Data_Num = Number;
    
    va_end(data_ptr);
}

void Vofa_Send_Data(void)
{
    if (vofa.Tx_Data_Num == 0) return;

    for (int i = 0; i < vofa.Tx_Data_Num; i++)
    {
        // Tx_Buff 是数组首地址(指针), + i*4 即偏移 i 个 float 的位置(每个 float 占 4 字节)
        memcpy(vofa.Tx_Buff + i * sizeof(float), vofa.Tx_Data_Ptrs[i], sizeof(float));
    }

    uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};
    memcpy(vofa.Tx_Buff + vofa.Tx_Data_Num * sizeof(float), tail, 4);

    uint16_t send_len = vofa.Tx_Data_Num * sizeof(float) + 4;
    HAL_UART_Transmit_DMA(vofa.huart, vofa.Tx_Buff, send_len);
}

