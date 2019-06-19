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
  * @brief          ң�����������жϣ���Ϊң�����Ĳ�������λ��ʱ�򣬲�һ���Ƿ���1024������
  * @author         RM
  * @param[in]      �����ң����ֵ
  * @param[in]      ��������������ң����ֵ
  * @param[in]      ����ֵ
  * @retval         ���ؿ�
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

//��̨���������������
static Gimbal_Control_t gimbal_control ;


uint32_t gimbal_high_water;

void GIMBAL_task(void *pvParameters)
{
	//��̨��ʼ��
	GIMBAL_Init(&gimbal_control);
	while(1)
	{
		//��̨���ݸ���
		GIMBAL_Feedback_Update(&gimbal_control);
		//��̨����������
		GIMBAL_set_contorl(&gimbal_control);
		//��̨����PID����
		GIMBAL_control_loop(&gimbal_control);               
		//Ni_Ming(0xf1,gimbal_control.gimbal_pitch_motor.gyro_angle,gimbal_control.gimbal_pitch_motor.gyro_angle_set,gimbal_control.gimbal_gyro_point->v_x,gimbal_control.gimbal_gyro_point->v_z);
		CAN_CMD_GIMBAL(1000, 0, 0, 0);
		vTaskDelay(pdMS_TO_TICKS(1));
		gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
	}
}

void GIMBAL_Init(Gimbal_Control_t *gimbal_init)
{
	//�������ָ���ȡ
	gimbal_init->gimbal_yaw_motor.gimbal_motor_measure = get_Yaw_Gimbal_Motor_Measure_Point();
	gimbal_init->gimbal_pitch_motor.gimbal_motor_measure = get_Pitch_Gimbal_Motor_Measure_Point();
	//����������ָ���ȡ
	gimbal_init->gimbal_gyro_point = get_GYRO_Measure_Point();  
  //ң��������ָ���ȡ
	gimbal_init->gimbal_rc_ctrl = get_remote_control_point();
	//��̨pit���PID��ʼ��
	GIMBAL_PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_gyro_pid, 5000, 0, 20, 0, 0);//kp_out ki_out kp ki kd
	GIMBAL_PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_acc_pid, 30000, 0, 60, 0, 0);
	//��̨yaw���PID��ʼ��
	GIMBAL_PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_gyro_pid, 5000, 5000, 30, 0, 0);
	GIMBAL_PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_acc_pid, 30000, 5000, 15, 0, 0);
}

void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_update)
{
    if (gimbal_feedback_update == NULL)
    {
        return;
    }
    //��̨���ݸ���
    gimbal_feedback_update->gimbal_pitch_motor.gyro_angle = gimbal_feedback_update->gimbal_gyro_point->pit;//pit�����ǽǶ�
    gimbal_feedback_update->gimbal_pitch_motor.gyro_acc = gimbal_feedback_update->gimbal_gyro_point->v_x;//pit���ٶ�

    gimbal_feedback_update->gimbal_yaw_motor.gyro_angle = gimbal_feedback_update->gimbal_gyro_point->yaw;//yaw�����ǽǶ�
    gimbal_feedback_update->gimbal_yaw_motor.gyro_acc = gimbal_feedback_update->gimbal_gyro_point->v_z;//yaw���ٶ�
		//printf("%f\r\n",gimbal_feedback_update->gimbal_pitch_motor.gimbal_motor_measure->angle);
		//printf("%d %d\r\n",gimbal_feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,gimbal_feedback_update->gimbal_pitch_motor.gimbal_motor_measure->last_ecd);
}

void GIMBAL_set_contorl(Gimbal_Control_t *gimbal_set_control)
{
	static int16_t yaw_channel = 0, pitch_channel = 0;

	//��ң���������ݴ������� int16_t yaw_channel,pitch_channel
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
	//pit����Ƕ�PID����
	gimbal_control_loop->gimbal_pitch_motor.gyro_acc_set = -PID_Calc(&gimbal_control_loop->gimbal_pitch_motor.gimbal_motor_gyro_pid, \
																																		gimbal_control_loop->gimbal_pitch_motor.gyro_angle, gimbal_control_loop->gimbal_pitch_motor.gyro_angle_set);
	//pit������ٶ�PID����
	gimbal_control_loop->gimbal_pitch_motor.given_current = PID_Calc(&gimbal_control_loop->gimbal_pitch_motor.gimbal_motor_acc_pid, \
	                                                                  gimbal_control_loop->gimbal_pitch_motor.gyro_acc, gimbal_control_loop->gimbal_pitch_motor.gyro_acc_set);
	//yaw����Ƕ�PID����
	gimbal_control_loop->gimbal_yaw_motor.gyro_acc_set = PID_Calc(&gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_gyro_pid, \
																																	gimbal_control_loop->gimbal_yaw_motor.gyro_angle, gimbal_control_loop->gimbal_yaw_motor.gyro_angle_set);
	//yaw������ٶ�PID����
	gimbal_control_loop->gimbal_yaw_motor.given_current = PID_Calc(&gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_acc_pid, \
																																	gimbal_control_loop->gimbal_yaw_motor.gyro_acc, gimbal_control_loop->gimbal_yaw_motor.gyro_acc_set);
	//Ni_Ming(0xf1,gimbal_control_loop->gimbal_yaw_motor.gyro_angle,gimbal_control_loop->gimbal_yaw_motor.given_current,0,0);
}

//��̨���PID��ʼ��
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

