#include "Task_Init.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can.h"
#include "Run.h"
#include "math.h"

extern Joint_t Joint[5];
float Motor_Init[4] = {0};

extern TaskHandle_t Motor_Drive_Handle;
extern TaskHandle_t MotorSendTask_Handle;
extern TaskHandle_t MotorRecTask_Handle;
TaskHandle_t Motor_Reset_Handle;

void MotorInit(void);
void Motor_reset(void *param);

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
    
    MotorInit();
    
		xTaskCreate(Motor_Drive, "Motor_Drive", 512, NULL, 4, &Motor_Drive_Handle);//驱动
		xTaskCreate(Motor_reset, "Motor_reset", 256, NULL, 4, &Motor_Reset_Handle);//驱动
    xTaskCreate(MotorSendTask, "MotorSendTask", 128, NULL, 4, &MotorSendTask_Handle);//将数据发送到PC
//    xTaskCreate(MotorRecTask, "MotorRecTask", 128, NULL, 4, &MotorRecTask_Handle);//PC接收数据

}

void RampToTarget(float *val, float target, float step)//斜坡
{
    float diff = target - *val;

    if (fabsf(diff) < step)
    {
        *val = target;
    }
    else
    {
        *val += (diff > 0 ? step : -step);
    }
}

void Motor_reset(void *param)
{
    TickType_t Last_wake_time = xTaskGetTickCount();
		
		vTaskDelay(1000);
		
		Motor_Init[0] = Joint[0].Rs_motor.state.rad;
		Motor_Init[1] = Joint[1].Rs_motor.state.rad;
		Motor_Init[2] = Joint[2].Rs_motor.state.rad;
		Motor_Init[3] = Joint[3].Rs_motor.state.rad;
		
		Joint[0].exp_rad = Motor_Init[0] - Joint[0].pos_offset;
		Joint[1].exp_rad = Motor_Init[1] - Joint[1].pos_offset;
		Joint[2].exp_rad = Motor_Init[2] - Joint[2].pos_offset;
		Joint[3].exp_rad = Motor_Init[3] - Joint[3].pos_offset;
    for (;;)
    {
        RampToTarget(&Joint[0].exp_rad, 0, 0.0002f);
				RampToTarget(&Joint[1].exp_rad, 0, 0.0002f);
				RampToTarget(&Joint[2].exp_rad, -1.53, 0.0002f);
				RampToTarget(&Joint[3].exp_rad, 0, 0.005f);
				RampToTarget(&Joint[4].exp_rad, -30000, 500);
			
				vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(5));
    }
    
}

void PID_Init_Pos(Joint_t *Joint, float kp, float ki, float kd, float limit, float pid_out)
{
	Joint->pos_pid.Kp = kp;
	Joint->pos_pid.Ki = ki;
	Joint->pos_pid.Kd = kd;
	Joint->pos_pid.limit = limit;
	Joint->pos_pid.output_limit = pid_out;
}

void PID_Init_Vel(Joint_t *Joint, float kp, float ki, float kd, float limit, float pid_out)
{
	Joint->vel_pid.Kp = kp;
	Joint->vel_pid.Ki = ki;
	Joint->vel_pid.Kd = kd;
	Joint->vel_pid.limit = limit;
	Joint->vel_pid.output_limit = pid_out;
}

void RS_Offest_inv(Joint_t *Joint, int8_t inv_motor, float pos_offset)
{
	Joint->inv_motor = inv_motor;
	Joint->pos_offset = pos_offset;
}

void MotorInit(void)
{
  Joint[4].RM_motor.hcan = &hcan2;
	Joint[4].RM_motor.ID = 0x0201;

	Joint[4].RM_motor.pos_pid.Kp = 0.15f;
	Joint[4].RM_motor.pos_pid.Ki = 0.0f;
	Joint[4].RM_motor.pos_pid.Kd = 0.0f;
	Joint[4].RM_motor.pos_pid.limit = 10000.0f;
	Joint[4].RM_motor.pos_pid.output_limit = 10000.0f;

	Joint[4].RM_motor.vel_pid.Kp = 15.0f;
	Joint[4].RM_motor.vel_pid.Ki = 0.1f;
	Joint[4].RM_motor.vel_pid.Kd = 0.0f;
	Joint[4].RM_motor.vel_pid.limit = 10000.0f;
	Joint[4].RM_motor.vel_pid.output_limit = 10000.0f;
	RS_Offest_inv(&Joint[4], 1, 30000.0f);//方向和偏移值
	
	PID_Init_Pos(&Joint[0], 1.0f, 0.0f, 0.0f, 100.0f, 2.0f);//位置pid//云台
	PID_Init_Vel(&Joint[0], 6.3f, 0.65f, 0.0f, 20.0f, 20.0f);//速度pid
	RS_Offest_inv(&Joint[0], 1, 4.31598663f);//方向和偏移值

	PID_Init_Pos(&Joint[1], 1.46f, 0.0f, 0.0f, 100.0f, 1.0f);//大臂
	PID_Init_Vel(&Joint[1], 6.1f, 0.57f, 0.0f, 20.0f, 20.0f);
	RS_Offest_inv(&Joint[1], 1, 1.78484118f);

	PID_Init_Pos(&Joint[2], 0.37f, 0.0f, 0.0f, 100.0f, 1.0f);//小臂
	PID_Init_Vel(&Joint[2], 4.0f, 0.5f, 0.0f, 20.0f, 20.0f);
	RS_Offest_inv(&Joint[2], 1, 5.32691097f);

	PID_Init_Pos(&Joint[3], 0.3f, 0.0f, 0.0f, 100.0f, 1.0f);
	PID_Init_Vel(&Joint[3], 2.0f, 0.3f, 0.0f, 20.0f, 20.0f);
	RS_Offest_inv(&Joint[3], 1, -1.86806214f);
	
	vTaskDelay(100);
	RobStrideInit(&Joint[0].Rs_motor, &hcan1, 0x01, RobStride_03);//云台
	RobStrideInit(&Joint[1].Rs_motor, &hcan1, 0x02, RobStride_03);//大臂
	RobStrideInit(&Joint[2].Rs_motor, &hcan1, 0x03, RobStride_03);//小臂
	RobStrideInit(&Joint[3].Rs_motor, &hcan1, 0x04, RobStride_01);//01
	RobStrideSetMode(&Joint[0].Rs_motor, RobStride_Torque);
	vTaskDelay(1);
	RobStrideSetMode(&Joint[1].Rs_motor, RobStride_Torque);
	vTaskDelay(1);
	RobStrideSetMode(&Joint[2].Rs_motor, RobStride_Torque);
	vTaskDelay(1);
	RobStrideSetMode(&Joint[3].Rs_motor, RobStride_Torque);
	vTaskDelay(200);
	RobStrideEnable(&Joint[0].Rs_motor);
	vTaskDelay(1);
	RobStrideEnable(&Joint[1].Rs_motor);
	vTaskDelay(1);
	RobStrideEnable(&Joint[2].Rs_motor);
	vTaskDelay(1);
	RobStrideEnable(&Joint[3].Rs_motor);

	vTaskDelay(2000);
}
