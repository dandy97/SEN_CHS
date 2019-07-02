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
static Shoot_Motor_t trigger_motor;          //射击数据

static const RC_ctrl_t *shoot_rc; //遥控器指针
static PidTypeDef trigger_motor_pid;         //电机PID
static shoot_mode_e shoot_mode = SHOOT_STOP; //射击状态机
static shoot_mode_e last_shoot_mode = SHOOT_STOP; //射击状态机

void shoot_init(void)
{
	autoshoot_open = 0;
	
	static const float Trigger_speed_pid[3] = {1200, 2, 0};//P I D
	//电机指针
	trigger_motor.shoot_motor_measure = get_Trigger_Motor_Measure_Point();
	//初始化PID                                                   PID_MAX_OUT     PID_MAX_IOUT
	PID_Init(&trigger_motor_pid, PID_POSITION, Trigger_speed_pid, 10000,          5000);
	//更新数据
	Shoot_Feedback_Update();

}

//射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
void Shoot_Set_Mode(void)
{
	switch(trigger_motor.shoot_motor_measure->shoot_mode)//右边遥控拨杆
	{
		case 1://遥控模式
		{
				switch(trigger_motor.shoot_motor_measure->bullet_launch)//左边遥控拨杆
				{
					case 1:
					{
						last_shoot_mode = shoot_mode;//上一次射击状态
						shoot_mode = SHOOT_STOP;//不发弹
						break;
					}
					case 3:
					{
						last_shoot_mode = shoot_mode;//上一次射击状态
						shoot_mode = SHOOT_READY;//准备发弹
						trigger_motor.shoot_ready_time = xTaskGetTickCount();
						break;
					}
					case 2:
					{
						if((xTaskGetTickCount() - trigger_motor.shoot_ready_time > 1000) && (last_shoot_mode == SHOOT_READY))//准备发弹状态后1秒和上一次射击状态是准备状态
						{
							last_shoot_mode = shoot_mode;//上一次射击状态
							shoot_mode = SHOOT_BULLET;//发射弹丸
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
		case 3://自动模式
		{
			if(trigger_motor.shoot_motor_measure->target == 1)
			{
				shoot_mode = SHOOT_BULLET;//发射弹丸
			}
			else
			{
				shoot_mode = SHOOT_READY;//准备发弹
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

//射击数据更新
void Shoot_Feedback_Update(void)
{
	trigger_motor.speed = trigger_motor.shoot_motor_measure->filter_rate;

	//射击开关下档时间计时
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

//射击循环
int16_t shoot_control_loop(uint16_t shoot_heat, uint8_t mains_power_shooter_output)
{
	int16_t shoot_CAN_Set_Current; //返回的can值

	Shoot_Set_Mode();        //设置状态机
	Shoot_Feedback_Update(); //更新数据
	
	if(mains_power_shooter_output == 3)
	{
		shoot_mode = SHOOT_STOP;
	}
	
	trigger_motor.cmd_time = xTaskGetTickCount();
	
	//没有射击的时候，拨盘启动时间等于系统时间
	//获取拨盘转动角度等于
	if((shoot_mode != SHOOT_BULLET) && (shoot_mode != AUTO_SHOOT))
	{
		trigger_motor.run_time = trigger_motor.cmd_time;
		trigger_motor.angle_before_1s = trigger_motor.shoot_motor_measure->angle;
		trigger_motor.block_time = 0;
	}
	
	//获取拨盘转动后每一秒转的角度
	if((trigger_motor.cmd_time - trigger_motor.run_time)%1000 == 0)
	{
		trigger_motor.angle_before_1s = trigger_motor.shoot_motor_measure->angle;
	}

	static uint32_t delay_shoot = 0;
	//射击子弹逻辑（先判断射击状态，再判断卡弹）
	if((shoot_mode == SHOOT_BULLET) || (shoot_mode == AUTO_SHOOT))
	{
		delay_shoot++;
		if(delay_shoot > 50)
		{
			//将1s内拨盘转过的角度除以每个拨齿之间的角度得到射出的子弹数
			trigger_motor.BulletShootCnt = (int32_t)fabs((trigger_motor.shoot_motor_measure->angle - trigger_motor.angle_before_1s)/36.0f);
			//拨盘速度设置
			if(trigger_motor.BulletShootCnt < 999)
			{
				trigger_motor.speed_set = 8;  
			}
			else 
			{
				trigger_motor.speed_set = 0;
			}	
			
			//卡弹反转
			//如果子弹在1s内发射数为0
			if(trigger_motor.BulletShootCnt == 0) 
			{
				//拨弹启动时间+1
				trigger_motor.block_time++;
				
				//如果拨弹启动时间大于200ms
				if(trigger_motor.block_time > 200) 
				{ 
					//拨盘反转50ms
					trigger_motor.speed_set = -20;
				}
				
				//如果拨弹启动时间大于250ms
				if(trigger_motor.block_time > 250) 
				{
					//如果拨弹启动时间清0，结束反转
					trigger_motor.block_time = 0;
					//重新获取拨盘转动后每一秒转的角度
					trigger_motor.angle_before_1s = trigger_motor.shoot_motor_measure->angle;
				}
			}
			//没有卡弹
			else 
			{
				//拨弹启动时间为0
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
	//计算拨弹轮电机PID
	PID_Calc(&trigger_motor_pid, trigger_motor.speed, trigger_motor.speed_set);
	
	trigger_motor.given_current = (int16_t)(trigger_motor_pid.out);
	shoot_CAN_Set_Current = trigger_motor.given_current;
	
	//Ni_Ming(0xf1,trigger_motor.speed,0,0,0);
	
	return shoot_CAN_Set_Current;
}
