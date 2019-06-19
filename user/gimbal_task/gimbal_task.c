#include "gimbal_task.h"
#include "stm32f4xx_hal.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "usart1.h"
#include "can_receive.h"
#include "chassis_task.h"
#include "pid.h"

/**
  * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定是发送1024过来，
  * @author         RM
  * @param[in]      输入的遥控器值
  * @param[in]      输出的死区处理后遥控器值
  * @param[in]      死区值
  * @retval         返回空
  */
#define rc_deadline_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

//云台控制所有相关数据
static Gimbal_Control_t gimbal_control ;


uint32_t gimbal_high_water;

void GIMBAL_task(void *pvParameters)
{
	//云台初始化
	GIMBAL_Init(&gimbal_control);
	while(1)
	{
		//云台数据更新
		GIMBAL_Feedback_Update(&gimbal_control);
		//云台控制量设置
		GIMBAL_set_contorl(&gimbal_control);
		//云台控制PID计算
		GIMBAL_control_loop(&gimbal_control);               
		//Ni_Ming(0xf1,gimbal_control.gimbal_pitch_motor.gyro_angle,gimbal_control.gimbal_pitch_motor.gyro_angle_set,gimbal_control.gimbal_gyro_point->v_x,gimbal_control.gimbal_gyro_point->v_z);
		CAN_CMD_GIMBAL(1000, 0, 0, 0);
		vTaskDelay(pdMS_TO_TICKS(1));
		gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
	}
}

void GIMBAL_Init(Gimbal_Control_t *gimbal_init)
{
	//电机数据指针获取
	gimbal_init->gimbal_yaw_motor.gimbal_motor_measure = get_Yaw_Gimbal_Motor_Measure_Point();
	gimbal_init->gimbal_pitch_motor.gimbal_motor_measure = get_Pitch_Gimbal_Motor_Measure_Point();
	//陀螺仪数据指针获取
	gimbal_init->gimbal_gyro_point = get_GYRO_Measure_Point();  
  //遥控器数据指针获取
	gimbal_init->gimbal_rc_ctrl = get_remote_control_point();
	//云台pit电机PID初始化
	GIMBAL_PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_gyro_pid, 5000, 0, 20, 0, 0);//kp_out ki_out kp ki kd
	GIMBAL_PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_acc_pid, 30000, 0, 60, 0, 0);
	//云台yaw电机PID初始化
	GIMBAL_PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_gyro_pid, 5000, 5000, 30, 0, 0);
	GIMBAL_PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_acc_pid, 30000, 5000, 15, 0, 0);
}

void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_update)
{
    if (gimbal_feedback_update == NULL)
    {
        return;
    }
    //云台数据更新
    gimbal_feedback_update->gimbal_pitch_motor.gyro_angle = gimbal_feedback_update->gimbal_gyro_point->pit;//pit陀螺仪角度
    gimbal_feedback_update->gimbal_pitch_motor.gyro_acc = gimbal_feedback_update->gimbal_gyro_point->v_x;//pit加速度

    gimbal_feedback_update->gimbal_yaw_motor.gyro_angle = gimbal_feedback_update->gimbal_gyro_point->yaw;//yaw陀螺仪角度
    gimbal_feedback_update->gimbal_yaw_motor.gyro_acc = gimbal_feedback_update->gimbal_gyro_point->v_z;//yaw加速度
		//printf("%f\r\n",gimbal_feedback_update->gimbal_pitch_motor.gimbal_motor_measure->angle);
		//printf("%d %d\r\n",gimbal_feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,gimbal_feedback_update->gimbal_pitch_motor.gimbal_motor_measure->last_ecd);
}

void GIMBAL_set_contorl(Gimbal_Control_t *gimbal_set_control)
{
	static int16_t yaw_channel = 0, pitch_channel = 0;

	//将遥控器的数据处理死区 int16_t yaw_channel,pitch_channel
	rc_deadline_limit(gimbal_set_control->gimbal_rc_ctrl->rc.ch[YawChannel], yaw_channel, RC_deadband);
	rc_deadline_limit(gimbal_set_control->gimbal_rc_ctrl->rc.ch[PitchChannel], pitch_channel, RC_deadband);

	gimbal_set_control->gimbal_yaw_motor.gyro_angle_set = yaw_channel * Yaw_RC_SEN;
	gimbal_set_control->gimbal_pitch_motor.gyro_angle_set = pitch_channel * Pitch_RC_SEN;
}

void GIMBAL_control_loop(Gimbal_Control_t *gimbal_control_loop)
{
//	static float pkp,pki,pkd,skp,ski,skd = 0;
//	gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_gyro_pid.Kp = pkp;
//	gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_gyro_pid.Ki = pki;
//	gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_gyro_pid.Kd = pkd;
//	gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_acc_pid.Kp = skp;
//	gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_acc_pid.Ki = ski;
//	gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_acc_pid.Kd = skd;
	//pit电机角度PID计算
	gimbal_control_loop->gimbal_pitch_motor.gyro_acc_set = -PID_Calc(&gimbal_control_loop->gimbal_pitch_motor.gimbal_motor_gyro_pid, \
																																		gimbal_control_loop->gimbal_pitch_motor.gyro_angle, gimbal_control_loop->gimbal_pitch_motor.gyro_angle_set);
	//pit电机角速度PID计算
	gimbal_control_loop->gimbal_pitch_motor.given_current = PID_Calc(&gimbal_control_loop->gimbal_pitch_motor.gimbal_motor_acc_pid, \
	                                                                  gimbal_control_loop->gimbal_pitch_motor.gyro_acc, gimbal_control_loop->gimbal_pitch_motor.gyro_acc_set);
	//yaw电机角度PID计算
	gimbal_control_loop->gimbal_yaw_motor.gyro_acc_set = PID_Calc(&gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_gyro_pid, \
																																	gimbal_control_loop->gimbal_yaw_motor.gyro_angle, gimbal_control_loop->gimbal_yaw_motor.gyro_angle_set);
	//yaw电机角速度PID计算
	gimbal_control_loop->gimbal_yaw_motor.given_current = PID_Calc(&gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_acc_pid, \
																																	gimbal_control_loop->gimbal_yaw_motor.gyro_acc, gimbal_control_loop->gimbal_yaw_motor.gyro_acc_set);
	//Ni_Ming(0xf1,gimbal_control_loop->gimbal_yaw_motor.gyro_angle,gimbal_control_loop->gimbal_yaw_motor.given_current,0,0);
}

//云台电机PID初始化
void GIMBAL_PID_Init(PidTypeDef *pid, float maxout, float max_iout, float kp, float ki, float kd)
{
	if (pid == NULL)
	{
			return;
	}
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;

	pid->set = 0.0f;

	pid->max_iout = max_iout;
	pid->max_out = maxout;
}

