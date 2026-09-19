/**
  * @file        imu_task.c/h
  * @brief      IMU数据处理任务
  * @history
  * Version    Date            Author                  Modification
  * V1.0.0     2025-11-12      FengYangjingbei         1. 完成初始版本
  *
  * @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  */

#include "Task_IMU.h"

extern IMU_RawData_TypeDef imu_raw_data;
extern IMU_EulerAngles_TypeDef imu_euler_data;

void imu_task(void const *pvParameters)
{
  TickType_t xLastWakeTime_Imu;                   // 存储上一次唤醒时间的变量
  const TickType_t xDelay_Imu = pdMS_TO_TICKS(1); // 1ms 延时（转换为 tick形式）

  /*初始化IMU硬件相关（内部状态、SPI配置、定时器、PID等）*/
  IMU_Init();

  vTaskDelay(50); // 延时50ms，等待任务稳定

  while (1)
  {
    /*获取时间，用于vTaskDelayUntil*/
    xLastWakeTime_Imu = xTaskGetTickCount();

    /*获取一次IMU数据（内部执行温度补偿、姿态解算等）*/
    IMU_Update();

    /*绝对延时一毫秒*/
    vTaskDelayUntil(&xLastWakeTime_Imu, xDelay_Imu);
  }
}
