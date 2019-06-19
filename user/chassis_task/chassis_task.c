#include "chassis_task.h"

#include "stm32f4xx_hal.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "projdefs.h"

#include "usart1.h"
#include "rc.h"
#include "optical_flow.h"
#include "judg_info.h"

#include "shoot.h"
#include "can_receive.h"
#include "pid.h"
#include "user_lib.h"
#include "math.h"
#include "time.h"
#include "stdlib.h"

//�������ݽṹ��
chassis_move_t chassis_move;
//���̷��͵�can ָ��
static int16_t Shoot_Can_Set_Current = 0;
//��������ռ�ʣ����
uint32_t chassis_high_water;
void chassis_task(void *pvParameters)
{
//	static float pid_in,pid_kp,pid_ki,pid_kd = 0;
	//����һ��ʱ��
  vTaskDelay(pdMS_TO_TICKS(2000));
	//���̳�ʼ��
	chassis_init(&chassis_move);
	//�����ʼ��
	shoot_init();
	while(1)
	{
		//�������ݸ���
		chassis_feedback_update(&chassis_move);
		//���̿���PID����
		chassis_control_loop(&chassis_move);
		//����������ѭ��
		Shoot_Can_Set_Current = shoot_control_loop(chassis_move.heat);
		//CAN_CMD_CHASSIS(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,	Shoot_Can_Set_Current, (int16_t)(chassis_move.dis_x + 120));
		//CAN_CMD_CHASSIS(0, 0,	Shoot_Can_Set_Current, (int16_t)(chassis_move.dis_x + 120));
		vTaskDelay(CHASSIS_CONTROL_TIME_MS);
		chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
	}
}

//���̳�ʼ��
void chassis_init(chassis_move_t *chassis_move_init)
{
	if (chassis_move_init == NULL)
	{
		return;
	}
	//�����ٶȻ�PIDֵ
	const static float motor_speed_pid[3] = {700, 0, 0};
	//����λ�û�PIDֵ
	const static float motor_pos_pid[3] = {1, 0, 0};
	
	//��ȡ���̵������ָ��
	for (uint8_t i = 0; i < 4; i++)
	{  
		chassis_move_init->motor_chassis[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i);
	}
	
	//����ϵͳ����ָ���ȡ
	chassis_move_init->judg_data = get_Judg_Info_Measure_Point();
	
	//tof����ָ���ȡ
	chassis_move_init->tof_measure = get_tof_Info_Measure_Point();
	
	chassis_move_init->chassis_gyro_point = get_GYRO_Measure_Point();
	
	//��ʼ�������ٶȻ�PID 
	for (uint8_t i = 0; i < 2; i++)
	{
		PID_Init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
	}
	
	//��ʼ������λ�û�PID 
	for (uint8_t i = 0; i < 2; i++)
	{
		PID_Init(&chassis_move_init->motor_pos_pid[i], PID_POSITION, motor_pos_pid, 30, 0);//25
	}
	
	//��ʼ��λ�û�����ֵ
	chassis_move_init->motor_chassis[0].pos_set = 0;
	chassis_move_init->cail_dis = 0;
	
	//����һ������
  chassis_feedback_update(chassis_move_init);
}

//�������ݸ���
void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
	//��̼� = ����ǶȻ��� - ����ʱ�����Ƕ�
	chassis_move_update->dis_x = (chassis_move_update->motor_chassis[0].chassis_motor_measure->angle * 0.0506145483f) - chassis_move_update->cail_dis;
	
	//����tof����
	chassis_move_update->tof_l = chassis_move_update->tof_measure->dis_l;
	chassis_move_update->tof_r = chassis_move_update->tof_measure->dis_r;
	
	//���²���ϵͳ����
	chassis_move_update->bullet_speed = chassis_move_update->judg_data->bullet_speed; //����
	chassis_move_update->bullet_freq = chassis_move_update->judg_data->bullet_freq;		//��Ƶ
	chassis_move_update->power = chassis_move_update->judg_data->power; 							//����
	chassis_move_update->power_buffer = chassis_move_update->judg_data->power_buffer; //ʣ������
	chassis_move_update->heat = chassis_move_update->judg_data->shooter_heat; 				//����
	chassis_move_update->last_Hp = chassis_move_update->Hp;       										//�ϴ�Ѫ��
	chassis_move_update->Hp = chassis_move_update->judg_data->Hp; 										//Ѫ��
}

//���̿���PID����
void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
	
	//����PID
	for (uint8_t i = 0; i < 2; i++)
	{
		PID_Calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
	}

	//��ֵ����ֵ
	for (uint8_t i = 0; i < 2; i++)
	{
		chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out * chassis_move_control_loop->power_buffer / 200.0f) ;
	}
}