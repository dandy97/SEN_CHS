#include "shoot.h"
#include "stm32f4xx_hal.h"

#include "rc.h"
#include "can_receive.h"
#include "pid.h"
#include "usart1.h"
#include "math.h"
#include "judg_info.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

extern uint8_t autoshoot_open;
InfantryJudge_Struct shoot_judg_data;
static Shoot_Motor_t trigger_motor;          //�������

static const RC_ctrl_t *shoot_rc; //ң����ָ��
static PidTypeDef trigger_motor_pid;         //���PID
static shoot_mode_e shoot_mode = SHOOT_STOP; //���״̬��
static shoot_mode_e last_shoot_mode = SHOOT_STOP; //���״̬��

void shoot_init(void)
{
	autoshoot_open = 0;
	
	static const float Trigger_speed_pid[3] = {1200, 2, 0};//P I D
	//���ָ��
	trigger_motor.shoot_motor_measure = get_Trigger_Motor_Measure_Point();
	//��ʼ��PID                                                   PID_MAX_OUT     PID_MAX_IOUT
	PID_Init(&trigger_motor_pid, PID_POSITION, Trigger_speed_pid, 10000,          5000);
	//��������
	Shoot_Feedback_Update();

}

//���״̬�����ã�ң�����ϲ�һ�ο��������ϲ��رգ��²�1�η���1�ţ�һֱ�����£���������䣬����3min׼��ʱ�������ӵ�
void Shoot_Set_Mode(void)
{
	switch(trigger_motor.shoot_motor_measure->shoot_mode)//�ұ�ң�ز���
	{
		case 1://ң��ģʽ
		{
				switch(trigger_motor.shoot_motor_measure->bullet_launch)//���ң�ز���
				{
					case 1:
					{
						last_shoot_mode = shoot_mode;//��һ�����״̬
						shoot_mode = SHOOT_STOP;//������
						break;
					}
					case 3:
					{
						last_shoot_mode = shoot_mode;//��һ�����״̬
						shoot_mode = SHOOT_READY;//׼������
						trigger_motor.shoot_ready_time = xTaskGetTickCount();
						break;
					}
					case 2:
					{
						if((xTaskGetTickCount() - trigger_motor.shoot_ready_time > 1000) && (last_shoot_mode == SHOOT_READY))//׼������״̬��1�����һ�����״̬��׼��״̬
						{
							last_shoot_mode = shoot_mode;//��һ�����״̬
							shoot_mode = SHOOT_BULLET;//���䵯��
						}
						break;
					}
					default:
					{
						break;
					}
				}
			break;
		}			
		case 3://�Զ�ģʽ
		{
			if(trigger_motor.shoot_motor_measure->target == 1)
			{
				shoot_mode = SHOOT_BULLET;//���䵯��
			}
			else
			{
				shoot_mode = SHOOT_READY;//׼������
			}
		}
		case 2:
		{
			
		}
		default:
		{
			break;
		}
	}
}

//������ݸ���
void Shoot_Feedback_Update(void)
{
	trigger_motor.speed = trigger_motor.shoot_motor_measure->filter_rate;

	//��������µ�ʱ���ʱ
	if (switch_is_down(shoot_rc->rc.s[1]) || autoshoot_open)
	{
		if (trigger_motor.rc_s_time < RC_S_LONG_TIME)
		{
			trigger_motor.rc_s_time++;
		}
	}
	else
	{
		trigger_motor.rc_s_time = 0;
	}
}

//���ѭ��
int16_t shoot_control_loop(uint16_t shoot_heat, uint8_t mains_power_shooter_output)
{
	int16_t shoot_CAN_Set_Current; //���ص�canֵ

	Shoot_Set_Mode();        //����״̬��
	Shoot_Feedback_Update(); //��������
	
	if(mains_power_shooter_output == 3)
	{
		shoot_mode = SHOOT_STOP;
	}
	
	trigger_motor.cmd_time = xTaskGetTickCount();
	
	//û�������ʱ�򣬲�������ʱ�����ϵͳʱ��
	//��ȡ����ת���Ƕȵ���
	if((shoot_mode != SHOOT_BULLET) && (shoot_mode != AUTO_SHOOT))
	{
		trigger_motor.run_time = trigger_motor.cmd_time;
		trigger_motor.angle_before_1s = trigger_motor.shoot_motor_measure->angle;
		trigger_motor.block_time = 0;
	}
	
	//��ȡ����ת����ÿһ��ת�ĽǶ�
	if((trigger_motor.cmd_time - trigger_motor.run_time)%1000 == 0)
	{
		trigger_motor.angle_before_1s = trigger_motor.shoot_motor_measure->angle;
	}

	static uint32_t delay_shoot = 0;
	//����ӵ��߼������ж����״̬�����жϿ�����
	if((shoot_mode == SHOOT_BULLET) || (shoot_mode == AUTO_SHOOT))
	{
		delay_shoot++;
		if(delay_shoot > 50)
		{
			//��1s�ڲ���ת���ĽǶȳ���ÿ������֮��ĽǶȵõ�������ӵ���
			trigger_motor.BulletShootCnt = (int32_t)fabs((trigger_motor.shoot_motor_measure->angle - trigger_motor.angle_before_1s)/36.0f);
			//�����ٶ�����
			if(trigger_motor.BulletShootCnt < 999)
			{
				trigger_motor.speed_set = 8;  
			}
			else 
			{
				trigger_motor.speed_set = 0;
			}	
			
			//������ת
			//����ӵ���1s�ڷ�����Ϊ0
			if(trigger_motor.BulletShootCnt == 0) 
			{
				//��������ʱ��+1
				trigger_motor.block_time++;
				
				//�����������ʱ�����200ms
				if(trigger_motor.block_time > 200) 
				{ 
					//���̷�ת50ms
					trigger_motor.speed_set = -20;
				}
				
				//�����������ʱ�����250ms
				if(trigger_motor.block_time > 250) 
				{
					//�����������ʱ����0��������ת
					trigger_motor.block_time = 0;
					//���»�ȡ����ת����ÿһ��ת�ĽǶ�
					trigger_motor.angle_before_1s = trigger_motor.shoot_motor_measure->angle;
				}
			}
			//û�п���
			else 
			{
				//��������ʱ��Ϊ0
				trigger_motor.block_time = 0;
			}
		}
	}
	else
	{
		trigger_motor.speed_set = 0;
		delay_shoot = 0;
	}
	
	if(shoot_heat > 430)
	{
		trigger_motor.speed_set = 0;
	}
	
	if(trigger_motor.shoot_motor_measure->bullet_launch == 0x01)
	{
		trigger_motor.speed_set = 0;
	}

//	if(1)
//	{
//		trigger_motor.speed_set = 0;
//	}
	//printf("%d\r\n",trigger_motor.BulletShootCnt);
	//���㲦���ֵ��PID
	PID_Calc(&trigger_motor_pid, trigger_motor.speed, trigger_motor.speed_set);
	
	trigger_motor.given_current = (int16_t)(trigger_motor_pid.out);
	shoot_CAN_Set_Current = trigger_motor.given_current;
	
	//Ni_Ming(0xf1,trigger_motor.speed,0,0,0);
	
	return shoot_CAN_Set_Current;
}
