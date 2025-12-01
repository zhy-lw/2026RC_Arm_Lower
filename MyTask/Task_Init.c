#include "Task_Init.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can.h"
#include "Run.h"
#include "stdbool.h"

extern Joint_t Joint[5];
extern SemaphoreHandle_t Can1_semaphore;
extern SemaphoreHandle_t Can2_semaphore;

extern TaskHandle_t Motor_Drive_Handle;
extern TaskHandle_t MotorSendTask_Handle;
extern TaskHandle_t MotorRecTask_Handle;

TaskHandle_t MotorInit_Handle;
TaskHandle_t CAN_Rec_Handle;
void MotorInitTask(void *pvParameters);
void CAN_Rec(void *pvParameters);

void Task_Init(void)
{
    CanFilter_Init(&hcan1);
    CanFilter_Init(&hcan2);
    HAL_CAN_Start(&hcan1); 
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO1_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan1,CAN_IT_TX_MAILBOX_EMPTY);
    HAL_CAN_ActivateNotification(&hcan2,CAN_IT_TX_MAILBOX_EMPTY);
    
    vPortEnterCritical();
    
	xTaskCreate(CAN_Rec, "CAN_Rec", 128, NULL, 4, &CAN_Rec_Handle);//记录偏移值
    xTaskCreate(MotorInitTask, "MotorInitTask", 128, NULL, 4, &MotorInit_Handle);//记录偏移值
    xTaskCreate(MotorSendTask, "MotorSendTask", 128, NULL, 4, &MotorSendTask_Handle);//将数据发送到PC
    xTaskCreate(MotorRecTask, "MotorRecTask", 128, NULL, 4, &MotorRecTask_Handle);//将数据发送到PC

    vPortExitCritical();

}

void CAN_Rec(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    for(;;)
    {
        if(xSemaphoreTake(Can1_semaphore, pdMS_TO_TICKS(0)) == pdTRUE)
        {
            uint8_t buf[8];
            uint32_t ID = CAN_Receive_DataFrame(&hcan1, buf);
            for (uint8_t i = 0; i < 4; i++)
            {
                RobStrideRecv_Handle(&Joint[i].Rs_motor, &hcan1, ID, buf);
            }
        }

        if(xSemaphoreTake(Can2_semaphore, pdMS_TO_TICKS(0)) == pdTRUE)
        {
            uint8_t buf[8];
            uint16_t ID = CAN_Receive_DataFrame(&hcan2, buf);
            Motor2006Recv(&Joint[4].RM_motor, &hcan2, ID, buf);
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(5));
    }
}

void MotorInitTask(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    uint8_t count = 0;

    for(;;)
    {
        if(xSemaphoreTake(Can1_semaphore, portMAX_DELAY) == pdTRUE)
        {
            for(uint8_t i = 0; i < 4; i++)
            {
                Joint[i].pos_offset = Joint[i].Rs_motor.state.rad;
            }
            count++;
        }

        if(count > 3)
        {
            xTaskCreate(Motor_Drive, "Motor_Drive", 512, NULL, 4, &Motor_Drive_Handle);
            vTaskDelete(NULL); // 删除自己
        }
    }

}
