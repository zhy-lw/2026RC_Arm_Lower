#include "Run.h"
#include "usb_trans.h"
#include "usbd_cdc_if.h"

extern uint8_t ready;

Joint_t Joint[5];
int16_t can_buf[4] = {0};

uint8_t enable_Joint[5] = {1,1,1,1,1};//1,1,1,1,1

TaskHandle_t Motor_Drive_Handle;
void Motor_Drive(void *param)
{
	TickType_t Last_wake_time = xTaskGetTickCount();
	
	for(;;)
	{
		for(uint8_t i = 0; i < 3; i++)
		{
			PID_Control(Joint[i].Rs_motor.state.rad, Joint[i].exp_rad + Joint[i].pos_offset, &Joint[i].pos_pid);
			PID_Control(Joint[i].Rs_motor.state.omega, Joint[i].pos_pid.pid_out + Joint[i].exp_omega, &Joint[i].vel_pid);
			RobStrideTorqueControl(&Joint[i].Rs_motor, Joint[i].vel_pid.pid_out * enable_Joint[i]);
		}
		vTaskDelay(1);
		PID_Control(Joint[3].Rs_motor.state.rad, 	Joint[3].exp_rad + Joint[3].pos_offset, &Joint[3].pos_pid);
		PID_Control(Joint[3].Rs_motor.state.omega, Joint[3].pos_pid.pid_out + Joint[3].exp_omega, &Joint[3].vel_pid);
		RobStrideTorqueControl(&Joint[3].Rs_motor, Joint[3].vel_pid.pid_out * enable_Joint[3]);//
		
		PID_Control(Joint[4].RM_motor.actual_pos, Joint[4].exp_rad - Joint[4].pos_offset, &Joint[4].RM_motor.pos_pid);
		PID_Control(Joint[4].RM_motor.motor.Speed, Joint[4].RM_motor.pos_pid.pid_out, &Joint[4].RM_motor.vel_pid);
		can_buf[0] = Joint[4].RM_motor.vel_pid.pid_out * enable_Joint[4];
		MotorSend(&hcan2 ,0x200, can_buf);
		
		vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(2));
	}
}

Arm_t arm_t;
TaskHandle_t MotorSendTask_Handle;

SemaphoreHandle_t cdc_recv_semphr;
Arm_t arm_Rec_t;
uint16_t cur_recv_size;

void CDC_Recv_Cb(uint8_t *src, uint16_t size)
{
	cur_recv_size=size;
	if(((Arm_t *)src )->pack_type == 0x01)
	{
		memcpy(&arm_Rec_t, src, sizeof(arm_Rec_t));
		xSemaphoreGive(cdc_recv_semphr);
	}
}

void MotorSendTask(void *param)// 将电机的数据发送到PC上
{
	TickType_t Last_wake_time = xTaskGetTickCount();
	USB_CDC_Init(CDC_Recv_Cb, NULL, NULL);
	arm_t.pack_type = 1;
	
	for(;;)
	{
		for (uint8_t i = 0; i < 4; i++)
		{
			arm_t.joints[i].rad = (Joint[i].Rs_motor.state.rad -  Joint[i].pos_offset) * Joint[i].inv_motor;
			arm_t.joints[i].omega = Joint[i].Rs_motor.state.omega;
			arm_t.joints[i].torque = Joint[i].Rs_motor.state.torque;
		}
		
		arm_t.joints[4].rad =(((Joint[4].RM_motor.actual_pos + Joint[4].pos_offset)/ 8192.0f / 36.0f) * 2.0f * 3.1415926f) * Joint[4].inv_motor;
		arm_t.joints[4].omega = Joint[4].RM_motor.motor.Speed / 36.0f * 3.1415926f /30.0f;
		
		CDC_Transmit_FS((uint8_t*)&arm_t, sizeof(arm_t));
		vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(20));
	}
}

uint8_t count = 0; 
TaskHandle_t MotorRecTask_Handle;
void MotorRecTask(void *param)// 从PC接收电机的期望值
{
	TickType_t last_wake_time = xTaskGetTickCount();

	cdc_recv_semphr = xSemaphoreCreateBinary();
  xSemaphoreTake(cdc_recv_semphr, 0);

	for (;;)
	{
		if(xSemaphoreTake(cdc_recv_semphr, pdMS_TO_TICKS(200)) == pdTRUE)
		{
			count ++;
			Joint[0].exp_rad = arm_Rec_t.joints[0].rad;
			Joint[0].exp_omega = arm_Rec_t.joints[0].omega;
			Joint[0].exp_torque = arm_Rec_t.joints[0].torque;

			Joint[1].exp_rad = arm_Rec_t.joints[1].rad;
			Joint[1].exp_omega = arm_Rec_t.joints[1].omega;
			Joint[1].exp_torque = arm_Rec_t.joints[1].torque;

			Joint[2].exp_rad = arm_Rec_t.joints[2].rad * Joint[2].inv_motor;
			Joint[2].exp_omega = arm_Rec_t.joints[2].omega * Joint[2].inv_motor;
			Joint[2].exp_torque = arm_Rec_t.joints[2].torque * Joint[2].inv_motor;

			Joint[3].exp_rad = arm_Rec_t.joints[3].rad * Joint[3].inv_motor;
			Joint[3].exp_omega = arm_Rec_t.joints[3].omega * Joint[3].inv_motor;
			Joint[3].exp_torque = arm_Rec_t.joints[3].torque * Joint[3].inv_motor;

			Joint[4].exp_rad =( arm_Rec_t.joints[4].rad / 6.28319f * 36.0f * 8192.0f);
			Joint[4].exp_omega = arm_Rec_t.joints[4].omega;
			Joint[4].exp_torque = arm_Rec_t.joints[4].torque;

		}
	}
} 

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan->Instance == CAN1)
	{
		uint8_t buf[8];
		uint32_t ID = CAN_Receive_DataFrame(&hcan1, buf);
		
		RobStrideRecv_Handle(&Joint[0].Rs_motor, &hcan1, ID, buf);
		RobStrideRecv_Handle(&Joint[1].Rs_motor, &hcan1, ID, buf);
		RobStrideRecv_Handle(&Joint[2].Rs_motor, &hcan1, ID, buf);
		RobStrideRecv_Handle(&Joint[3].Rs_motor, &hcan1, ID, buf);
	}
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan->Instance == CAN2)
	{
		uint8_t buf[8];
		uint16_t ID = CAN_Receive_DataFrame(&hcan2, buf);
		Motor2006Recv(&Joint[4].RM_motor, &hcan2, ID, buf);
	}
}


