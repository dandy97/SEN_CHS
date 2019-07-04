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

//��������״̬
static chassis_mode_e chassis_mode = INIT_MODE;
//�����Զ�ģʽ״̬
static auto_mode_e auto_mode = auto_ready;
//�������ݽṹ��
chassis_move_t chassis_move;
//���̷��͵�can ָ��
static int16_t Shoot_Can_Set_Current = 0;
static uint32_t can_send_time = 0;
//��������ռ�ʣ����
uint32_t chassis_high_water;
void chassis_task(void *pvParameters)
{
//	static float pid_in,pid_kp,pid_ki,pid_kd = 0;
	//����һ��ʱ��
  vTaskDelay(pdMS_TO_TICKS(4000));
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
		Shoot_Can_Set_Current = shoot_control_loop(chassis_move.heat, chassis_move.mains_power_shooter_output);
		CAN_CMD_CHASSIS(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,	Shoot_Can_Set_Current, (int16_t)(chassis_move.dis_x + 120));
		if((can_send_time++) % 200 == 0)
		{
			CAN_FRIC_INFO(chassis_move.mains_power_shooter_output);
		}
		Ni_Ming(0xf1,chassis_move.dis_x,0,0,0);
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
	const static float motor_speed_pid[3] = {350, 0, 0};
	//����λ�û�PIDֵ
	const static float motor_pos_pid[3] = {3, 0, 0};
	
	//��ȡ���̵������ָ��
	for (uint8_t i = 0; i < 4; i++)
	{  
		chassis_move_init->motor_chassis[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i);
	}
	
	//����ϵͳ����ָ���ȡ
	chassis_move_init->judg_data = get_Judg_Info_Measure_Point();
	
	//tof����ָ���ȡ
	chassis_move_init->tof_measure = get_tof_Info_Measure_Point();
	
	//���������ݻ�ȡ
	chassis_move_init->chassis_gyro_point = get_GYRO_Measure_Point();
	chassis_move_init->last_angle_z = chassis_move_init->chassis_gyro_point->yaw;
	chassis_move_init->angle_z = chassis_move_init->chassis_gyro_point->yaw;
	
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

static uint32_t updata_time,six_time,auto_six_time = 0;
static uint8_t pass_positive,pass_negative,pass_curve_1,pass_curve_2 = 0;
//�������ݸ���
void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
	
	//���µ���״̬
	switch(chassis_move_update->motor_chassis[0].chassis_motor_measure->chassis_mode)
	{
		case 1:
		{
			chassis_mode = RC_MODE;//ң��ģʽ
			auto_mode = auto_ready;//�Զ�ģʽ��ʼ��
			pass_positive = pass_negative = pass_curve_1 = pass_curve_2 = 0;
			break;
		}			
		case 3:
		{
			if(chassis_move_update->motor_chassis[0].chassis_motor_measure->chassis_stop == 3)
			{
				chassis_mode = STOP_MODE;//�Ӿ�����ģʽ
			}
			else
			{
				chassis_mode = AUTO_MODE;//�Զ�ģʽ
			}
				//�Զ�ģʽ״̬�л�
				if(auto_mode == auto_ready)//��ʼ��״̬
				{
					if(chassis_move_update->tof_l < 80)//�����������С��80cm
					{
						auto_mode = auto_find;//Ѳ��ģʽ
						chassis_move_update->cail_dis = chassis_move_update->motor_chassis[0].chassis_motor_measure->angle * 0.0506145483f + 100;//У׼��̼�
					}
				}
				else if(auto_mode == auto_find)//Ѳ��ģʽ
				{
					six_time++;
					auto_six_time = six_time;
					if(chassis_move_update->last_Hp - chassis_move_update->Hp > 0)//������
					{
						auto_mode = auto_six;//��Aģʽ
					}
					else if(chassis_move_update->target == 1)//����Ŀ��
					{
						auto_mode = auto_attack;//����ģʽ
					}
				}			
				else if(auto_mode == auto_six)//��Aģʽ
				{
					auto_six_time++;
					if(auto_six_time - six_time > 5000)
					{
						auto_mode = auto_find;//Ѳ��ģʽ
					}
				}
				else if(auto_mode == auto_attack)//����Ŀ��
				{
					if(chassis_move_update->target == 0)//����Ŀ��
					{
						auto_mode = auto_find;//Ѳ��ģʽ
					}
				}
			break;
		}
		case 2:
		{
			chassis_mode = STOP_MODE;//ֹͣģʽ
			auto_mode = auto_ready;//�Զ�ģʽ��ʼ��
			pass_positive = pass_negative = pass_curve_1 = pass_curve_2 = 0;
			break;
		}
		default:
		{
			break;
		}
	}
	
	//test
	//auto_mode = auto_six;
	//
	
	//���µ��̵������
	chassis_move_update->motor_chassis[0].speed = chassis_move_update->motor_chassis[0].chassis_motor_measure->filter_rate / 19.0f; //����ٶȷ���
	
	//���¹�������
	chassis_move_update->target = chassis_move_update->motor_chassis[0].chassis_motor_measure->target;//�����־λ
	
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
	chassis_move_update->last_hurd_type = chassis_move_update->hurd_type;							//��һ���˺�
	chassis_move_update->hurd_type = get_Hurt_Type();																	//�˺�
	chassis_move_update->mains_power_shooter_output = chassis_move_update->judg_data->mains_power_shooter_output;//Ħ���ֿڵ�Դ
	
	//���������ǽǶ�
	if(xTaskGetTickCount() - updata_time > 200)
	{		
		chassis_move_update->last_angle_z = chassis_move_update->angle_z;//ȡ200msǰ�����ǽǶ�
		updata_time = xTaskGetTickCount();
	}
	chassis_move_update->angle_z = chassis_move_update->chassis_gyro_point->yaw;//��ǰ�����ǽǶ�
	chassis_move_update->gyro_rate = chassis_move_update->angle_z - chassis_move_update->last_angle_z;//�����ǽ��ٶȣ�5hz��
	
	/* ----------------------------------------���Ĵ��롢������̼�-----------------------------------------------*/
	if(auto_mode == auto_find)//Ѳ��ģʽ
	{
		if(chassis_move_update->gyro_rate > 6)//���ٶ�Ϊ��
		{
			pass_positive = 1;
		}
		if((pass_positive == 1) && (chassis_move_update->gyro_rate < -6))//���ٶ�Ϊ��
		{
			pass_negative = 1;
		}
		if(pass_positive && pass_negative)//��->������ʾ����һ���䣩
		{
			if((chassis_move_update->dis_x > -200) && (chassis_move_update->dis_x < 130))//��һ����
			{
				pass_curve_1 = 1;
				pass_positive = 0;
				pass_negative = 0;
			}			
			if((chassis_move_update->dis_x > 200) && (chassis_move_update->dis_x < 500))//�ڶ�����
			{
				pass_curve_2 = 1;
				pass_positive = 0;
				pass_negative = 0;
			}	
			if(pass_curve_1 && pass_curve_2)//���������䣬У׼������
			{
				chassis_move_update->cail_dis = chassis_move_update->motor_chassis[0].chassis_motor_measure->angle * 0.0506145483f;
				pass_curve_1 = 0;
				pass_curve_2 = 0;
			}
		}
	}
	//��̼� = ����ǶȻ��� - ����ʱ����Ƕ�
	chassis_move_update->dis_x = (chassis_move_update->motor_chassis[0].chassis_motor_measure->angle * 0.0506145483f) - chassis_move_update->cail_dis;
	/* ----------------------------------------���Ĵ��롢������̼�-----------------------------------------------*/
	
	
	//��������
	if(auto_mode == auto_six)//��Aģʽ
	{
		if(auto_six_time % 500 == 0 || fabs(chassis_move_update->motor_chassis[0].speed_set) < 3)
		{
			srand(xTaskGetTickCount());
			chassis_move_update->last_six_pos = chassis_move_update->six_pos;//��һ�������
			chassis_move_update->six_pos = rand()%370 - 20;//�����
			//�����������Ĳ�ֵ����
			if((chassis_move_update->six_pos - chassis_move_update->last_six_pos < 20) && (chassis_move_update->six_pos - chassis_move_update->last_six_pos > 0))
			{
				chassis_move_update->six_pos += 100;
			}
			else if((chassis_move_update->six_pos - chassis_move_update->last_six_pos > -20) && (chassis_move_update->six_pos - chassis_move_update->last_six_pos < 0))
			{
				chassis_move_update->six_pos -= 100;
			}
		}
		//������޷�
		if(chassis_move_update->six_pos > 350) chassis_move_update->six_pos = 200;
		if(chassis_move_update->six_pos < -20) chassis_move_update->six_pos = 100;
		//Ni_Ming(0xf1,rand()%400,chassis_move_update->six_pos,0,0);
	}
}

static uint8_t turn_run = 0;
//���̿���PID����
void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
	switch(chassis_mode)
	{
		case AUTO_MODE:
		{
			if(auto_mode == auto_ready)//��ʼ��״̬
			{
				chassis_move_control_loop->motor_chassis[0].speed_set = 15;
			}
			else if(auto_mode == auto_find)//Ѳ��ģʽ
			{
				if(turn_run == 0)
				{
					if(chassis_move_control_loop->dis_x > 350)//����޷�
					{
						turn_run = 1;
					}
					chassis_move_control_loop->motor_chassis[0].pos_set = 600;
				}
				else if(turn_run == 1)
				{
					if(chassis_move_control_loop->dis_x < -50)//����޷�
					{
						turn_run = 0;
					}
					chassis_move_control_loop->motor_chassis[0].pos_set = -200;
				}
				PID_Calc(&chassis_move_control_loop->motor_pos_pid[0], chassis_move_control_loop->dis_x, chassis_move_control_loop->motor_chassis[0].pos_set);//λ�û�
				chassis_move_control_loop->motor_chassis[0].speed_set = -(int16_t)chassis_move_control_loop->motor_pos_pid[0].out;//�ٶȻ�����
			}
			else if(auto_mode == auto_attack)//����ģʽ
			{
				chassis_move_control_loop->motor_chassis[0].speed_set = 0;//�ٶȻ�����
			}
			else if(auto_mode == auto_six)//��Aģʽ
			{
				chassis_move_control_loop->motor_chassis[0].pos_set = chassis_move_control_loop->six_pos;//�����
				PID_Calc(&chassis_move_control_loop->motor_pos_pid[0], chassis_move_control_loop->dis_x, chassis_move_control_loop->motor_chassis[0].pos_set);//λ�û�
				chassis_move_control_loop->motor_chassis[0].speed_set = -(int16_t)chassis_move_control_loop->motor_pos_pid[0].out;//�ٶȻ�����
			}
			
			break;
		}			
		case RC_MODE:
		{
			chassis_move_control_loop->motor_chassis[0].speed_set = chassis_move_control_loop->motor_chassis[0].chassis_motor_measure->speed_set * 12.0f; //����ٶ����� (0~4)*12
			break;
		}
		case STOP_MODE:
		{
			chassis_move_control_loop->motor_chassis[0].speed_set = 0;
			break;
		}
		default:
		{
			break;
		}
	}
	if(auto_mode == auto_six)//�Ӵ�����
	{
		chassis_move_control_loop->motor_pos_pid[0].max_out = 40;
	}
	else
	{
		chassis_move_control_loop->motor_pos_pid[0].max_out = 30;
	}
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

//���ص�������״̬
uint8_t get_chassis_state(void)
{
	return chassis_mode;
}
