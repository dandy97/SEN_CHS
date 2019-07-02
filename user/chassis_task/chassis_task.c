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

//底盘任务状态
static chassis_mode_e chassis_mode = INIT_MODE;
//底盘自动模式状态
static auto_mode_e auto_mode = auto_ready;
//底盘数据结构体
chassis_move_t chassis_move;
//拨盘发送的can 指令
static int16_t Shoot_Can_Set_Current = 0;
//底盘任务空间剩余量
uint32_t chassis_high_water;
void chassis_task(void *pvParameters)
{
//	static float pid_in,pid_kp,pid_ki,pid_kd = 0;
	//空闲一段时间
  vTaskDelay(pdMS_TO_TICKS(4000));
	//底盘初始化
	chassis_init(&chassis_move);
	//射击初始化
	shoot_init();
	while(1)
	{
		//底盘数据更新
		chassis_feedback_update(&chassis_move);
		//底盘控制PID计算
		chassis_control_loop(&chassis_move);
		//射击任务控制循环
		Shoot_Can_Set_Current = shoot_control_loop(chassis_move.heat, chassis_move.mains_power_shooter_output);
		CAN_CMD_CHASSIS(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,	Shoot_Can_Set_Current, (int16_t)(chassis_move.dis_x + 120));
		Ni_Ming(0xf1,chassis_move.tof_l,chassis_move.tof_r,0,0);
		vTaskDelay(CHASSIS_CONTROL_TIME_MS);
		chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
	}
}

//底盘初始化
void chassis_init(chassis_move_t *chassis_move_init)
{
	if (chassis_move_init == NULL)
	{
		return;
	}
	//底盘速度环PID值
	const static float motor_speed_pid[3] = {200, 0, 0};
	//底盘位置环PID值
	const static float motor_pos_pid[3] = {1, 0, 0};
	
	//获取底盘电机数据指针
	for (uint8_t i = 0; i < 4; i++)
	{  
		chassis_move_init->motor_chassis[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i);
	}
	
	//裁判系统数据指针获取
	chassis_move_init->judg_data = get_Judg_Info_Measure_Point();
	
	//tof数据指针获取
	chassis_move_init->tof_measure = get_tof_Info_Measure_Point();
	
	//陀螺仪数据获取
	chassis_move_init->chassis_gyro_point = get_GYRO_Measure_Point();
	
	//初始化底盘速度环PID 
	for (uint8_t i = 0; i < 2; i++)
	{
		PID_Init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
	}
	
	//初始化底盘位置环PID 
	for (uint8_t i = 0; i < 2; i++)
	{
		PID_Init(&chassis_move_init->motor_pos_pid[i], PID_POSITION, motor_pos_pid, 30, 0);//25
	}
	
	//初始化位置环输入值
	chassis_move_init->motor_chassis[0].pos_set = 0;
	chassis_move_init->cail_dis = 0;
	
	//更新一下数据
  chassis_feedback_update(chassis_move_init);
}

//底盘数据更新
void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
	//里程计 = 电机角度换算 - 过弯时清理角度
	chassis_move_update->dis_x = (chassis_move_update->motor_chassis[0].chassis_motor_measure->angle * 0.0506145483f) - chassis_move_update->cail_dis;
	
	//更新底盘电机数据
	chassis_move_update->motor_chassis[0].speed = chassis_move_update->motor_chassis[0].chassis_motor_measure->filter_rate / 19.0f; //电机速度反馈
	
	//更新工控数据
	chassis_move_update->target = chassis_move_update->motor_chassis[0].chassis_motor_measure->target;//自瞄标志位
	
	//更新tof数据
	chassis_move_update->tof_l = chassis_move_update->tof_measure->dis_l;
	chassis_move_update->tof_r = chassis_move_update->tof_measure->dis_r;
	
	//更新裁判系统数据
	chassis_move_update->bullet_speed = chassis_move_update->judg_data->bullet_speed; //射速
	chassis_move_update->bullet_freq = chassis_move_update->judg_data->bullet_freq;		//射频
	chassis_move_update->power = chassis_move_update->judg_data->power; 							//功率
	chassis_move_update->power_buffer = chassis_move_update->judg_data->power_buffer; //剩余能量
	chassis_move_update->heat = chassis_move_update->judg_data->shooter_heat; 				//热量
	chassis_move_update->last_Hp = chassis_move_update->Hp;       										//上次血量
	chassis_move_update->Hp = chassis_move_update->judg_data->Hp; 										//血量
	chassis_move_update->last_hurd_type = chassis_move_update->hurd_type;							//上一次伤害
	chassis_move_update->hurd_type = get_Hurt_Type();																	//伤害
	chassis_move_update->mains_power_shooter_output = chassis_move_update->judg_data->mains_power_shooter_output;//摩擦轮口电源

	//更新底盘状态
	switch(chassis_move_update->motor_chassis[0].chassis_motor_measure->chassis_mode)
	{
		case 1:
		{
			chassis_mode = RC_MODE;//遥控模式
			auto_mode = auto_ready;//自动模式初始化
			break;
		}			
		case 3:
		{
			chassis_mode = AUTO_MODE;//自动模式
				//自动模式状态切换
				if(auto_mode == auto_ready)//初始化状态
				{
					if(chassis_move_update->tof_l < 50)//距离左边柱子小于50cm
					{
						auto_mode = auto_find;//巡查模式
					}
				}
				else if(auto_mode == auto_find)//巡查模式
				{
					if(chassis_move_update->target == 1)//发现目标
					{
						auto_mode = auto_attack;//攻击模式
					}
				}
				else if(auto_mode == auto_attack)//攻击模式
				{
					if((chassis_move_update->last_Hp - chassis_move_update->Hp > 0)||(chassis_move_update->hurd_type != chassis_move_update->last_hurd_type))
					{
						auto_mode = auto_find;//巡查模式
					}
				}
			break;
		}
		case 2:
		{
			chassis_mode = STOP_MODE;//停止模式
			auto_mode = auto_ready;//自动模式初始化
			break;
		}
		default:
		{
			break;
		}
	}
}

//底盘控制PID计算
void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
	switch(chassis_mode)
	{
		case AUTO_MODE:
		{
			if(auto_mode == auto_ready)//初始化状态
			{
				chassis_move_control_loop->motor_chassis[0].speed_set = -10;
			}
			else if(auto_mode == auto_find)//巡查模式
			{
				PID_Calc(&chassis_move_control_loop->motor_pos_pid[0], chassis_move_control_loop->dis_x, chassis_move_control_loop->motor_chassis[0].pos_set);
				chassis_move_control_loop->motor_chassis[0].speed_set = -(int16_t)chassis_move_control_loop->motor_pos_pid[0].out;//速度环输入
			}
			else if(auto_mode == auto_attack)//攻击模式
			{
				chassis_move_control_loop->motor_chassis[0].speed_set = 0;//速度环输入
			}
			
			break;
		}			
		case RC_MODE:
		{
			chassis_move_control_loop->motor_chassis[0].speed_set = chassis_move_control_loop->motor_chassis[0].chassis_motor_measure->speed_set * 12.0f; //电机速度输入 (0~4)*12
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
	//计算PID
	for (uint8_t i = 0; i < 2; i++)
	{
		PID_Calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
	}

	//赋值电流值
	for (uint8_t i = 0; i < 2; i++)
	{
		chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out * chassis_move_control_loop->power_buffer / 250.0f) ;
	}
}

//返回底盘任务状态
uint8_t get_chassis_state(void)
{
	return chassis_mode;
}
