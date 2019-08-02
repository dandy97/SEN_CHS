#ifndef SHOOT_H
#define SHOOT_H

#include "main.h"
#include "stm32f4xx_hal.h"
#include "can_receive.h"
#include "judg_info.h"

typedef unsigned char bool_t;//0和1

//电机rmp 变化成 旋转速度的比例
#define Motor_RMP_TO_SPEED 0.000462962962962963

//遥控器射击开关打下档一段时间后 连续发射子弹 用于清单
#define RC_S_LONG_TIME 2000

//射击发射开关通道数据
#define Shoot_RC_Channel    1

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s) (s == RC_SW_MID)
#define switch_is_up(s) (s == RC_SW_UP)

typedef struct
{
	const motor_measure_t *shoot_motor_measure;
	float speed;
	float speed_set;
	float angle;
	float set_angle;
	float angle_before_1s;
	float angle_before_1ts;
	int16_t given_current;
	int16_t last_given_current;
	int8_t ecd_count;

	uint16_t rc_s_time;

	uint32_t shoot_ready_time;
	
	bool_t move_flag;
	uint32_t cmd_time;
	uint32_t run_time;
	uint32_t block_time;
	bool_t key;
	uint16_t key_time;
	bool_t shoot_done;
	uint8_t shoot_done_time;
	int16_t BulletShootCnt;
	int16_t last_butter_count;
} Shoot_Motor_t;

typedef enum
{
    SHOOT_STOP = 0,
    SHOOT_READY,
    SHOOT_BULLET,
    SHOOT_DONE,
		AUTO_SHOOT,
} shoot_mode_e;

extern InfantryJudge_Struct shoot_judg_data;
//射击初始化
extern void shoot_init(void);
//射击状态机设置
void Shoot_Set_Mode(void);
//射击数据更新
void Shoot_Feedback_Update(void);
//射击循环
int16_t shoot_control_loop(uint16_t shoot_heat, uint8_t mains_power_shooter_output); 

#endif
