#include "drv_can.h"

// CAN通信发送缓冲区
uint8_t CAN1_0x1ff_Tx_Data[8];
uint8_t CAN1_0x200_Tx_Data[8];

void CAN_Init(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_Start(hcan);

    //开启接收中断
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
}

/*
1. CAN 滤波器配置
2. 高五位选择滤波器序号：FILTER(x)
3. 第低三位选择FIFOx
4. ID是目标ID，掩码Mask_ID就是当前位是否需要检查进入
*/
void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan, uint8_t Config_Data, uint32_t ID, uint32_t Mask_ID)
{
    CAN_FilterTypeDef can_filter_init_structure;

    if (Config_Data & 0x01) return;
    if ((Config_Data & 0x02) >> 1) return;


    // 标准帧的 ID 只有 11 位
    // 标准帧 ID 规定必须存放在高 16 位 (FilterIdHigh)， 即 [15:5] 处。

    // 11 个 1 | 然后左移 5 位，正好对齐到寄存器的 [15:5] 位置。
    can_filter_init_structure.FilterIdHigh = (ID & 0x7FF) << 5;
    
    // 标准帧配置下，低 16 位不需要存放 ID，全部填 0
    can_filter_init_structure.FilterIdLow = 0x0000;
    
    // 掩码配置完全相同，对齐到 [15:5]
    can_filter_init_structure.FilterMaskIdHigh = (Mask_ID & 0x7FF) << 5;
    
    // 掩码低 16 位同样全部填 0
    can_filter_init_structure.FilterMaskIdLow = 0x0000;

    // 滤波器编号选择
    // Config_Data 的高 5 位存放了我们要分配的滤波器编号，右移 3 位拿出来。
    // 0x1F : 11111)，掩码限制最大值为 31
    can_filter_init_structure.FilterBank = (Config_Data >> 3) & 0x1F;

    can_filter_init_structure.FilterMode = CAN_FILTERMODE_IDMASK;
    
    // 滤波器位宽
    can_filter_init_structure.FilterScale = CAN_FILTERSCALE_32BIT;

    // 激活/使能这个滤波器组
    can_filter_init_structure.FilterActivation = ENABLE;
    can_filter_init_structure.SlaveStartFilterBank = 14;
    // 取 Config_Data 的第 2 位来决定，0 对应 FIFO0，1 对应 FIFO1。
    can_filter_init_structure.FilterFIFOAssignment = (Config_Data >> 2) & 0x01;


    HAL_CAN_ConfigFilter(hcan, &can_filter_init_structure);
}

// CAN 发送
uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t used_mailbox;

    if (hcan == NULL) return HAL_ERROR;

    tx_header.StdId = ID;
    tx_header.ExtId = 0;
    tx_header.IDE   = CAN_ID_STD;
    tx_header.RTR   = CAN_RTR_DATA;
    tx_header.DLC   = Length;

    return (HAL_CAN_AddTxMessage(hcan, &tx_header, Data, &used_mailbox));
}
