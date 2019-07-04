#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "rc.h"
#include "optical_flow.h"
#include "judg_info.h"

#include "can_receive.h"
#include "pid.h"
#include "user_lib.h"

//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357

//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

//前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 1
//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 0
//遥控器死区限制
#define CHASSIS_RC_DEADLINE 10
//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.001f
//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.001f

//底盘3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 5000.0f

//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 3.0f
//底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 2.9f
//底盘电机最大速度
#define MAX_WHEEL_SPEED 30.0f

//m3508转化成底盘速度(m/s)的比例，做两个宏 是因为可能换电机需要更换比例
#define M3508_MOTOR_RPM_TO_VECTOR 0.00015597718771929826
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//底盘电机速度环PID
#define M3505_MOTOR_SPEED_PID_KP 15000.0f
#define M3505_MOTOR_SPEED_PID_KI 10.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//1.0 in = 2.54 cm	
//240DPI	每240点一英寸
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
  const RC_ctrl_t *chassis_RC;               //底盘使用的遥控器指针
	const optic_flow_data_t *optic_flow_data;
	const InfantryJudge_Struct * judg_data;
//  const Gimbal_Motor_t *chassis_yaw_motor;   //底盘使用到yaw云台电机的相对角度来计算底盘的欧拉角
//  const Gimbal_Motor_t *chassis_pitch_motor; //底盘使用到pitch云台电机的相对角度来计算底盘的欧拉角
//  const fp32 *chassis_INS_angle;             //获取陀螺仪解算出的欧拉角指针
//  chassis_mode_e chassis_mode;               //底盘控制状态机
//  chassis_mode_e last_chassis_mode;          //底盘上次控制状态机
  Chassis_Motor_t motor_chassis[2];          //底盘电机数据
  PidTypeDef motor_speed_pid[2];             //底盘电机速度pid
	PidTypeDef motor_pos_pid[2];               //底盘电机位置pid
//  PidTypeDef chassis_angle_pid;              //底盘跟随角度pid

  first_order_filter_type_t chassis_cmd_slow_set_vx;
  first_order_filter_type_t chassis_cmd_slow_set_vy;

	uint16_t pass_curve_1;						//过第一个弯道
	uint16_t return_curve_1;					//返回第一个弯道
	uint16_t pass_curve_2;						//过第二个弯道
	uint16_t return_curve_2;					//返回第二个弯道
	
	uint8_t  mains_power_shooter_output;
	uint32_t hurd_type;
  uint32_t last_hurd_type;
	uint8_t  target;
	uint8_t  stop_run;
	float    bullet_speed;							//射速
	uint8_t  bullet_freq;			        //射频
	uint16_t volt;							      //电压
	uint16_t heat;                    //热量
	uint8_t  Lv;                      //机器人等级
	uint16_t Hp;											//剩余血量
	uint16_t last_Hp;                 //上次血量
	float	power;											//功率
	float power_buffer;               //底盘缓冲能量
	float tof_l;											//左边tof
	float tof_r;											//右边tof
  float vx;                         //底盘速度 前进方向 前为正，单位 m/s
  float vy;                         //底盘速度 左右方向 左为正  单位 m/s
  float wz;                         //底盘旋转角速度，逆时针为正 单位 rad/s
  float vx_set;                     //底盘设定速度 前进方向 前为正，单位 m/s
  float vy_set;                     //底盘设定速度 左右方向 左为正，单位 m/s
  float wz_set;                     //底盘设定旋转角速度，逆时针为正 单位 rad/s
	float dis_x_raw;									//底盘X轴光流原始值		
	float dis_y_raw;									//底盘Y轴光流原始值	
	float cail_dis;                   //校准位移
	float dis_x;											//底盘X轴位移 单位cm
	float dis_y;
	float v_z;												//z轴角速度
	float gyro_rate;                  //z轴角度变化率
	float last_angle_z;								//上次z轴角度
	float angle_z;										//z轴角度
	
	int32_t six_pos;
	int32_t last_six_pos;
//  fp32 chassis_relative_angle;     //底盘与云台的相对角度，单位 rad/s
//  fp32 chassis_relative_angle_set; //设置相对云台控制角度
//  fp32 chassis_yaw_set;

  float vx_max_speed;  //前进方向最大速度 单位m/s
  float vx_min_speed;  //前进方向最小速度 单位m/s
  float vy_max_speed;  //左右方向最大速度 单位m/s
  float vy_min_speed;  //左右方向最小速度 单位m/s
//  fp32 chassis_yaw;   //陀螺仪和云台电机叠加的yaw角度
//  fp32 chassis_pitch; //陀螺仪和云台电机叠加的pitch角度
//  fp32 chassis_roll;  //陀螺仪和云台电机叠加的roll角度
} chassis_move_t;

extern chassis_move_t chassis_move;

//底盘任务
void chassis_task(void *pvParameters);
//底盘初始化，主要是pid初始化
void chassis_init(chassis_move_t *chassis_move_init);
//底盘数据更新
void chassis_feedback_update(chassis_move_t *chassis_move_update);
//底盘控制量设置
void chassis_set_contorl(chassis_move_t *chassis_move_control);
//遥控器的数据处理成底盘的前进vx速度，vy速度
void chassis_rc_to_control_vector(float *vx_set, float *vy_set, chassis_move_t *chassis_move_rc_to_vector);
//底盘控制PID计算
void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
//返回底盘任务状态
uint8_t get_chassis_state(void);
#endif
