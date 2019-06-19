#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H
#include "stm32f4xx_hal.h"
#include "can.h"



//���can����ID
typedef enum
{
	CAN_CHASSIS_ALL_ID = 0x200,
	CAN_3508_M1_ID = 0x201,
	CAN_3508_M2_ID = 0x202,
	CAN_TRIGGER_MOTOR_ID = 0x203,
	
	CAN_YAW_MOTOR_ID = 0x205,
	CAN_PIT_MOTOR_ID = 0x206,

	CAN_GIMBAL_ALL_ID = 0x1FF,
} can_msg_id_e;

//rm���ͳһ���ݽṹ��
typedef struct
{
	uint8_t chassis_stop;
	uint16_t ecd;
	int16_t speed_rpm;
	int16_t given_current;
	uint8_t temperate;
	int16_t last_ecd;
	int16_t offset_ecd;
	float round_cnt;
	float ecd_raw_rate;
	float total_ecd;
	float angle;
	int32_t  rate_buf[5];          //buf��for filter
  uint8_t  buf_cut;                       //������
  int32_t  filter_rate;                   //���ٶ�
	int32_t  last_filter_rate;              //�ϴν��ٶ�
	
	float speed_raw;
	float speed_set;
	
	uint16_t bullet_launch;								//����������־
	uint16_t chassis_mode;                //����ģʽ
} motor_measure_t;

//���������ݽṹ��
typedef struct
{
	float v_x;
	float v_z;
	float pit;
	float yaw;
} gyro_info_t;

//tof���ݽṹ��
typedef struct
{
	uint16_t dis_r;   //tof����
	uint16_t dis_l;   //tof����
	uint16_t str;     //tof�ź�ǿ��
}tof_data_t;


//ͳһ����can�жϺ���
void CAN_hook(CAN_RxHeaderTypeDef *rx_message, uint8_t *Data);
//���͵��̵����������
void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
//���������ǿ�������
void CAN_CMD_GYRO_CALI(uint8_t mode, uint16_t time);
//���ص��̵��������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i);
//���ز������������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Trigger_Motor_Measure_Point(void);
//������̨�����������
void CAN_CMD_GIMBAL(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
//��̨���ݻ�ȡ
void get_motor_measuer(motor_measure_t* ptr, uint8_t* Data, uint16_t Id);
//���������Ǳ�����ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern const gyro_info_t *get_GYRO_Measure_Point(void);
//����tof������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const tof_data_t *get_tof_Info_Measure_Point(void);
#endif
