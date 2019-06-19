#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "stm32f4xx_hal.h" 

#include "can_receive.h"
#include "rc.h"
#include "user_lib.h"
#include "pid.h"

//yaw,pitch控制通道以及状态开关通道
#define YawChannel 2
#define PitchChannel 3
//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_deadband 10
//yaw，pitch角度与遥控器输入比例
#define Yaw_RC_SEN -0.000005f
#define Pitch_RC_SEN -0.000006f //0.005

//pitch 角度环 
#define PITCH_GYRO_PID_KP 0.0f
#define PITCH_GYRO_PID_KI 0.0f
#define PITCH_GYRO_PID_KD 0.0f

#define PITCH_GYRO_PID_MAX_OUT 5000.0f
#define PITCH_GYRO_PID_MAX_IOUT 0.0f

//pitch 角速度环 
#define PITCH_ACC_PID_KP 0.0f
#define PITCH_ACC_PID_KI 0.0f
#define PITCH_ACC_PID_KD 0.0f

#define PITCH_ACC_PID_MAX_OUT 30000.0f
#define PITCH_ACC_PID_MAX_IOUT 0.0f

typedef struct
{
	float kp;
	float ki;
	float kd;

	float set;
	float get;
	float err;

	float max_out;
	float max_iout;

	float Pout;
	float Iout;
	float Dout;

	float out;
} Gimbal_PID_t;

typedef struct
{
    const motor_measure_t *gimbal_motor_measure;
//    Gimbal_PID_t gimbal_motor_absolute_angle_pid;
//    Gimbal_PID_t gimbal_motor_relative_angle_pid;
    PidTypeDef gimbal_motor_gyro_pid;
	  PidTypeDef gimbal_motor_acc_pid;
//    gimbal_motor_mode_e gimbal_motor_mode;
//    gimbal_motor_mode_e last_gimbal_motor_mode;
    uint16_t offset_ecd;
		uint16_t last_ecd;
		float rount_cnt;
    float max_relative_angle; //rad
    float min_relative_angle; //rad

    float relative_angle;     //rad
    float relative_angle_set; //rad
    float gyro_angle;     //rad
    float gyro_angle_set; //rad
    float gyro_acc;         //rad/s
    float gyro_acc_set;
    float motor_speed;
    float raw_cmd_current;
    float current_set;
    int16_t given_current;

} Gimbal_Motor_t;

typedef struct
{
    float max_yaw;
    float min_yaw;
    float max_pitch;
    float min_pitch;
    uint16_t max_yaw_ecd;
    uint16_t min_yaw_ecd;
    uint16_t max_pitch_ecd;
    uint16_t min_pitch_ecd;
    uint8_t step;
} Gimbal_Cali_t;

typedef struct
{
    const RC_ctrl_t *gimbal_rc_ctrl;
    const gyro_info_t *gimbal_gyro_point;
    Gimbal_Motor_t gimbal_yaw_motor;
    Gimbal_Motor_t gimbal_pitch_motor;
    Gimbal_Cali_t gimbal_cali;
} Gimbal_Control_t;

//云台任务
void GIMBAL_task(void *pvParameters); 
//初始化云台
void GIMBAL_Init(Gimbal_Control_t *gimbal_init);
//云台数据更新
void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_update);
//计算云台电机相对中值的相对角度
float motor_ecd_to_angle_change(uint16_t ecd, uint16_t last_ecd, float round_cnt);
//云台控制量设置
void GIMBAL_set_contorl(Gimbal_Control_t *chassis_move_control);
//云台控制pid计算
void GIMBAL_control_loop(Gimbal_Control_t *gimbal_control_loop);
//云台电机PID初始化
void GIMBAL_PID_Init(PidTypeDef *pid, float maxout, float max_iout, float kp, float ki, float kd);
#endif
