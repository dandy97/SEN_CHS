#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "rc.h"
#include "optical_flow.h"
#include "judg_info.h"

#include "can_receive.h"
#include "pid.h"
#include "user_lib.h"

//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 357

//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//����������Ƽ�� 0.002s
#define CHASSIS_CONTROL_TIME 0.002

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

//ǰ���ң����ͨ������
#define CHASSIS_X_CHANNEL 1
//���ҵ�ң����ͨ������
#define CHASSIS_Y_CHANNEL 0
//ң������������
#define CHASSIS_RC_DEADLINE 10
//ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define CHASSIS_VX_RC_SEN 0.001f
//ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���
#define CHASSIS_VY_RC_SEN 0.001f

//����3508���can���͵���ֵ
#define MAX_MOTOR_CAN_CURRENT 5000.0f

//�����˶��������ǰ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_X 3.0f
//�����˶��������ƽ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_Y 2.9f
//���̵������ٶ�
#define MAX_WHEEL_SPEED 30.0f

//m3508ת���ɵ����ٶ�(m/s)�ı������������� ����Ϊ���ܻ������Ҫ��������
#define M3508_MOTOR_RPM_TO_VECTOR 0.00015597718771929826
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//���̵���ٶȻ�PID
#define M3505_MOTOR_SPEED_PID_KP 15000.0f
#define M3505_MOTOR_SPEED_PID_KI 10.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//1.0 in = 2.54 cm	
//240DPI	ÿ240��һӢ��
#define DPI_POINT_TO_CENTIMETER	(2.54f / 240.0f)

typedef enum
{
	INIT_MODE = 0,
	RC_MODE,
	AUTO_MODE,
	STOP_MODE
} chassis_mode_e;

typedef enum
{
	auto_ready = 0,
	auto_find,
	auto_attack,
	auto_six,
} auto_mode_e;

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  float accel;
  float speed;
  float speed_set;
	float angle;
	float angle_start;
	float pos;
	float pos_set;
  int16_t give_current;
} Chassis_Motor_t;

typedef struct
{
	const gyro_info_t *chassis_gyro_point;
	const tof_data_t *tof_measure;
  const RC_ctrl_t *chassis_RC;               //����ʹ�õ�ң����ָ��
	const optic_flow_data_t *optic_flow_data;
	const InfantryJudge_Struct * judg_data;
//  const Gimbal_Motor_t *chassis_yaw_motor;   //����ʹ�õ�yaw��̨�������ԽǶ���������̵�ŷ����
//  const Gimbal_Motor_t *chassis_pitch_motor; //����ʹ�õ�pitch��̨�������ԽǶ���������̵�ŷ����
//  const fp32 *chassis_INS_angle;             //��ȡ�����ǽ������ŷ����ָ��
//  chassis_mode_e chassis_mode;               //���̿���״̬��
//  chassis_mode_e last_chassis_mode;          //�����ϴο���״̬��
  Chassis_Motor_t motor_chassis[2];          //���̵������
  PidTypeDef motor_speed_pid[2];             //���̵���ٶ�pid
	PidTypeDef motor_pos_pid[2];               //���̵��λ��pid
//  PidTypeDef chassis_angle_pid;              //���̸���Ƕ�pid

  first_order_filter_type_t chassis_cmd_slow_set_vx;
  first_order_filter_type_t chassis_cmd_slow_set_vy;

	uint16_t pass_curve_1;						//����һ�����
	uint16_t return_curve_1;					//���ص�һ�����
	uint16_t pass_curve_2;						//���ڶ������
	uint16_t return_curve_2;					//���صڶ������
	
	uint8_t  mains_power_shooter_output;
	uint32_t hurd_type;
  uint32_t last_hurd_type;
	uint8_t  target;
	uint8_t  stop_run;
	float    bullet_speed;							//����
	uint8_t  bullet_freq;			        //��Ƶ
	uint16_t volt;							      //��ѹ
	uint16_t heat;                    //����
	uint8_t  Lv;                      //�����˵ȼ�
	uint16_t Hp;											//ʣ��Ѫ��
	uint16_t last_Hp;                 //�ϴ�Ѫ��
	float	power;											//����
	float power_buffer;               //���̻�������
	float tof_l;											//���tof
	float tof_r;											//�ұ�tof
  float vx;                         //�����ٶ� ǰ������ ǰΪ������λ m/s
  float vy;                         //�����ٶ� ���ҷ��� ��Ϊ��  ��λ m/s
  float wz;                         //������ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
  float vx_set;                     //�����趨�ٶ� ǰ������ ǰΪ������λ m/s
  float vy_set;                     //�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
  float wz_set;                     //�����趨��ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
	float dis_x_raw;									//����X�����ԭʼֵ		
	float dis_y_raw;									//����Y�����ԭʼֵ	
	float cail_dis;                   //У׼λ��
	float dis_x;											//����X��λ�� ��λcm
	float dis_y;
	float v_z;												//z����ٶ�
	float gyro_rate;                  //z��Ƕȱ仯��
	float last_angle_z;								//�ϴ�z��Ƕ�
	float angle_z;										//z��Ƕ�
	
	int32_t six_pos;
	int32_t last_six_pos;
//  fp32 chassis_relative_angle;     //��������̨����ԽǶȣ���λ rad/s
//  fp32 chassis_relative_angle_set; //���������̨���ƽǶ�
//  fp32 chassis_yaw_set;

  float vx_max_speed;  //ǰ����������ٶ� ��λm/s
  float vx_min_speed;  //ǰ��������С�ٶ� ��λm/s
  float vy_max_speed;  //���ҷ�������ٶ� ��λm/s
  float vy_min_speed;  //���ҷ�����С�ٶ� ��λm/s
//  fp32 chassis_yaw;   //�����Ǻ���̨������ӵ�yaw�Ƕ�
//  fp32 chassis_pitch; //�����Ǻ���̨������ӵ�pitch�Ƕ�
//  fp32 chassis_roll;  //�����Ǻ���̨������ӵ�roll�Ƕ�
} chassis_move_t;

extern chassis_move_t chassis_move;

//��������
void chassis_task(void *pvParameters);
//���̳�ʼ������Ҫ��pid��ʼ��
void chassis_init(chassis_move_t *chassis_move_init);
//�������ݸ���
void chassis_feedback_update(chassis_move_t *chassis_move_update);
//���̿���������
void chassis_set_contorl(chassis_move_t *chassis_move_control);
//ң���������ݴ���ɵ��̵�ǰ��vx�ٶȣ�vy�ٶ�
void chassis_rc_to_control_vector(float *vx_set, float *vy_set, chassis_move_t *chassis_move_rc_to_vector);
//���̿���PID����
void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
//���ص�������״̬
uint8_t get_chassis_state(void);
#endif
