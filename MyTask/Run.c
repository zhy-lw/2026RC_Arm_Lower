#include "Run.h"
#include "usb_trans.h"
#include "usbd_cdc_if.h"

Joint_t Joint[5];
int16_t can_buf[4] = {0};

extern SemaphoreHandle_t Can1_semaphore;
extern SemaphoreHandle_t Can2_semaphore;

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

TaskHandle_t Motor_Drive_Handle;
void Motor_Drive(void *param)
{
	TickType_t Last_wake_time = xTaskGetTickCount();
	
	Joint[4].RM_motor.hcan = &hcan2;
	Joint[4].RM_motor.ID = 0x0201;

	Joint[4].RM_motor.pos_pid.Kp = 1.0f;
	Joint[4].RM_motor.pos_pid.Ki = 0.0f;
	Joint[4].RM_motor.pos_pid.Kd = 0.0f;
	Joint[4].RM_motor.pos_pid.limit = 10000.0f;
	Joint[4].RM_motor.pos_pid.output_limit = 10000.0f;

	Joint[4].vel_pid.Kp = 0.5f;
	Joint[4].vel_pid.Ki = 0.07f;
	Joint[4].vel_pid.Kd = 0.0f;
	Joint[4].vel_pid.limit = 10000.0f;
	Joint[4].vel_pid.output_limit = 10000.0f;

	PID_Init_Pos(&Joint[0], 1.0f, 0.0f, 0.0f, 100.0f, 1.0f);//位置pid
	PID_Init_Vel(&Joint[0], 6.0f, 0.5f, 0.0f, 20.0f, 20.0f);//速度pid
	RS_Offest_inv(&Joint[0], 1, 0.0f);//方向和偏移值

	PID_Init_Pos(&Joint[1], 1.0f, 0.0f, 0.0f, 100.0f, 1.0f);
	PID_Init_Vel(&Joint[1], 6.0f, 0.5f, 0.0f, 20.0f, 20.0f);
	RS_Offest_inv(&Joint[1], 1, 0.0f);

	PID_Init_Pos(&Joint[2], 1.0f, 0.0f, 0.0f, 100.0f, 1.0f);
	PID_Init_Vel(&Joint[2], 6.0f, 0.5f, 0.0f, 20.0f, 20.0f);
	RS_Offest_inv(&Joint[2], 1, 0.0f);

	PID_Init_Pos(&Joint[3], 1.0f, 0.0f, 0.0f, 100.0f, 1.0f);
	PID_Init_Vel(&Joint[3], 6.0f, 0.5f, 0.0f, 20.0f, 20.0f);
	RS_Offest_inv(&Joint[3], 1, 0.0f);

	RobStrideInit(&Joint[0].Rs_motor, &hcan1, 0x201, RobStride_03);
	RobStrideInit(&Joint[1].Rs_motor, &hcan1, 0x202, RobStride_03);
	RobStrideInit(&Joint[2].Rs_motor, &hcan1, 0x203, RobStride_03);
	RobStrideInit(&Joint[3].Rs_motor, &hcan1, 0x204, RobStride_03);
	RobStrideSetMode(&Joint[0].Rs_motor, RobStride_Torque);
	RobStrideSetMode(&Joint[1].Rs_motor, RobStride_Torque);
	RobStrideSetMode(&Joint[2].Rs_motor, RobStride_Torque);
	RobStrideSetMode(&Joint[3].Rs_motor, RobStride_Torque);
	vTaskDelay(100);
	RobStrideEnable(&Joint[0].Rs_motor);
	RobStrideEnable(&Joint[1].Rs_motor);
	RobStrideEnable(&Joint[2].Rs_motor);
	RobStrideEnable(&Joint[3].Rs_motor);

	vTaskDelay(2000);

	for(;;)
	{
		for(uint8_t i = 0; i < 4; i++)
		{
			PID_Control(Joint[i].Rs_motor.state.rad, Joint[i].exp_rad + Joint[i].pos_offset, &Joint[i].pos_pid);
			PID_Control(Joint[i].Rs_motor.state.omega * Joint[i].inv_motor, Joint[i].pos_pid.pid_out, &Joint[i].vel_pid);
			RobStrideTorqueControl(&Joint[i].Rs_motor, Joint[i].vel_pid.pid_out * Joint[i].inv_motor);
		}
		PID_Control(Joint[4].RM_motor.motor.Angle_DEG, Joint[4].exp_rad, &Joint[4].RM_motor.pos_pid);
		PID_Control(Joint[4].RM_motor.motor.Speed * Joint[4].inv_motor, Joint[4].RM_motor.pos_pid.pid_out, &Joint[4].RM_motor.vel_pid);
		can_buf[0] = Joint[4].RM_motor.vel_pid.pid_out * Joint[4].inv_motor;
		MotorSend(&hcan2 ,0x200, can_buf);

		vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(5));
	}
	
}

Arm_t arm_t;
TaskHandle_t MotorSendTask_Handle;
void MotorSendTask(void *param)// 将电机的数据发送到PC上
{
	TickType_t Last_wake_time = xTaskGetTickCount();

	arm_t.pack_type = 0;
	
	for(;;)
	{
		for (uint8_t i = 0; i < 4; i++)
		{
			arm_t.joints[i].rad = Joint[i].Rs_motor.state.rad -  Joint[i].pos_offset;
			arm_t.joints[i].omega = Joint[i].Rs_motor.state.omega;
			arm_t.joints[i].torque = Joint[i].Rs_motor.state.torque;
		}
		
		arm_t.joints[4].rad = Joint[4].RM_motor.motor.Angle_DEG;
		arm_t.joints[4].omega = Joint[4].RM_motor.motor.Speed;
		memset(&arm_t.joints[5], 0, sizeof(arm_t.joints[5]));
		
		CDC_Transmit_FS((uint8_t*)&arm_t, sizeof(arm_t));
		vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(10));
	}
}

SemaphoreHandle_t cdc_recv_semphr;
Arm_t arm_Rec_t;

void CDC_Recv_Cb(uint8_t *src, uint16_t size)
{
	if(((Arm_t *)src )->pack_type == 0x00)
	{
		memcpy(&arm_Rec_t, src, sizeof(arm_Rec_t));
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(cdc_recv_semphr, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
	
}

TaskHandle_t MotorRecTask_Handle;
void MotorRecTask(void *param)// 从PC接收电机的期望值
{
	USB_CDC_Init(CDC_Recv_Cb, NULL, NULL);
	TickType_t last_wake_time = xTaskGetTickCount();

	cdc_recv_semphr = xSemaphoreCreateBinary();
    xSemaphoreTake(cdc_recv_semphr, 0);

	for (;;)
	{
		if(xSemaphoreTake(cdc_recv_semphr, pdMS_TO_TICKS(200)) == pdTRUE)
		{
			for (uint8_t i = 0; i < 4; i++)
			{
				Joint[i].exp_rad = arm_Rec_t.joints[i].rad;
				Joint[i].exp_omega = arm_Rec_t.joints[i].omega;
				Joint[i].exp_torque = arm_Rec_t.joints[i].torque;
			}
			Joint[4].exp_rad = arm_Rec_t.joints[4].rad;
			Joint[4].exp_omega = arm_Rec_t.joints[4].omega;
			Joint[4].exp_torque = arm_Rec_t.joints[4].torque;
		}else
		{
			memset(&Joint->exp_rad,    0, sizeof(Joint->exp_rad));
			memset(&Joint->exp_omega,  0, sizeof(Joint->exp_omega));
			memset(&Joint->exp_torque, 0, sizeof(Joint->exp_torque));
		}
	}
} 

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan->Instance == CAN1)
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(Can1_semaphore, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan->Instance == CAN2)
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(Can2_semaphore, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}


