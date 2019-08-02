#include "can_receive.h"

#include "stm32f4xx_hal.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "projdefs.h"

//���̵�����ݶ�ȡ
#define get_motor_measure(ptr, Data)                             \
    {                                                            \
			(ptr)->last_ecd = (ptr)->ecd;                              \
			(ptr)->ecd = (uint16_t)(Data[0] << 8 | Data[1]);           \
			(ptr)->speed_rpm = (uint16_t)(Data[2] << 8 | Data[3]);     \
			(ptr)->given_current = (uint16_t)(Data[4] << 8 | Data[5]); \
			(ptr)->temperate = Data[6];                                \
    }

uint8_t autoshoot_open;
//����tof����
static tof_data_t tof_data;
//�����������
static motor_measure_t motor_chassis[4], motor_trigger;
//�����Ǳ���
static gyro_info_t gyro_info;
//CAN�����жϻص�����
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan == (&hcan1))
	{
		uint8_t Data[8];
		CAN_RxHeaderTypeDef RxMeg;
		HAL_StatusTypeDef	HAL_RetVal;
		HAL_RetVal = HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxMeg, Data);
		if(HAL_OK==HAL_RetVal)
		{
			switch (RxMeg.StdId)
			{
				case CAN_3508_M1_ID:
				case CAN_3508_M2_ID:
				{
					static uint8_t i = 0;
					//������ID��
					i = RxMeg.StdId - CAN_3508_M1_ID;
					//���������ݺ꺯��
					get_motor_measuer(&motor_chassis[i], Data, RxMeg.StdId);
					break;
				}
				case CAN_TRIGGER_MOTOR_ID:
				{
					//���������ݺ꺯��
					get_motor_measuer(&motor_trigger, Data, RxMeg.StdId);
					break;
				}
				case 0x300:
				{
					motor_chassis[0].speed_raw = (float)((short)(Data[2]<<8 | Data[3]));
					motor_chassis[0].speed_set = (float)(motor_chassis[0].speed_raw / 100);
					motor_trigger.bullet_launch = Data[4];//ң����߲���
					motor_chassis[0].chassis_stop = Data[4];
					motor_chassis[0].chassis_mode = motor_chassis[1].chassis_mode = motor_trigger.shoot_mode = Data[6];//ң���ұ߲���
					motor_chassis[0].target = motor_trigger.target =  Data[7];//����Ŀ��
//					printf("%d %d\r\n",motor_trigger.bullet_launch, motor_chassis[0].target);
					break;
				}
				default:
				{
					break;
				}			
			}
		}
	}
	if(hcan == (&hcan2))
	{
		uint8_t Data[8];
		CAN_RxHeaderTypeDef RxMeg;
		HAL_StatusTypeDef	HAL_RetVal;
		HAL_RetVal = HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxMeg, Data);
		if(HAL_OK==HAL_RetVal)
		{
			switch (RxMeg.StdId)
			{
				case 0x401:
				{
					gyro_info.yaw = (float)(0.008571428571f)*((int32_t)(Data[0]<<24)|(int32_t)(Data[1]<<16) | (int32_t)(Data[2]<<8) | (int32_t)(Data[3])); 
					break;
				}
				case 101:
				{
					static int16_t raw_v_z, raw_yaw, yaw_connt = 0;
					static float yaw_angle, last_yaw_angle = 0;
					raw_v_z = Data[2]<<8 | Data[3];
					raw_yaw = Data[6]<<8 | Data[7];
					
					//������ԭʼ�����ǻ��ȣ��ѻ���ת��Ϊ�Ƕ�
					gyro_info.v_z = (float)raw_v_z * 0.057295f;
					
					//������ԭʼ���ݱ�����100��
					yaw_angle = (float)raw_yaw/100;
						
					//���Ƕȸĳ������ģ�����360����0��
					if((yaw_angle - last_yaw_angle) > 330)
						yaw_connt--;
					else if((yaw_angle - last_yaw_angle) < -330)
						yaw_connt++;
					
					gyro_info.yaw_cheap = yaw_angle + yaw_connt * 360;
					last_yaw_angle = yaw_angle;
					break;
				}
				case 0x300:
				{
					if((Data[0]<<8 | Data[1]) < 1000)
					tof_data.dis_l = Data[0]<<8 | Data[1];
					if((Data[2]<<8 | Data[3]) < 1000)
					tof_data.dis_r = Data[2]<<8 | Data[3];
					break;
				}
				default:
				{
					break;
				}
			}
		}
	}
}

//������ݻ�ȡ
void get_motor_measuer(motor_measure_t* ptr, uint8_t* Data, uint16_t Id)
{
  ptr->last_ecd = ptr->ecd;
  ptr->ecd      = Data[0] << 8 | Data[1];
	ptr->speed_rpm = (uint16_t)(Data[2] << 8 | Data[3]); 

	if (ptr->ecd - ptr->last_ecd > 4096)
	{
    ptr->round_cnt--;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd - 8192;
  }
	
	else if (ptr->ecd - ptr->last_ecd < -4096)
  {
    ptr->round_cnt++;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd + 8192;
  }
  else
  {
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
  }

	int32_t temp_sum = 0;
  ptr->rate_buf[ptr->buf_cut++] = ptr->ecd_raw_rate;
  if (ptr->buf_cut >= 5)
    ptr->buf_cut = 0;
  for (uint8_t i = 0; i < 5; i++)
  {
    temp_sum += ptr->rate_buf[i];
  }
	ptr->last_filter_rate = ptr->filter_rate;
	if(Id == CAN_TRIGGER_MOTOR_ID)
	{
		ptr->filter_rate = (int32_t)(temp_sum/5/36);
		
		ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
		/* total angle, unit is degree */
		ptr->angle = ptr->total_ecd / (8192.0f/360.0f*36.0f);
	}
	else
	{
		ptr->filter_rate = (int32_t)(temp_sum/5);
		
		ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
		/* total angle, unit is degree */
		ptr->angle = -ptr->total_ecd / (8192.0f/360.0f*19.0f);
	}
}
//���͵��̵����������
void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
	uint8_t Data[8];
	uint32_t pTxMailbox;
	CAN_TxHeaderTypeDef TxMeg;
	
	TxMeg.StdId = CAN_CHASSIS_ALL_ID;
	TxMeg.IDE = CAN_ID_STD;
	TxMeg.RTR = CAN_RTR_DATA;
	TxMeg.DLC = 0x08;
	Data[0] = motor1 >> 8;
	Data[1] = motor1;
	Data[2] = motor2 >> 8;
	Data[3] = motor2;
	Data[4] = motor3 >> 8;
	Data[5] = motor3;
	Data[6] = (uint8_t)(motor4 >> 8);
	Data[7] = (uint8_t)motor4;

	HAL_CAN_AddTxMessage(&hcan1, &TxMeg, Data, &pTxMailbox);
}

//������̨�����������
void CAN_CMD_GIMBAL(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
	uint8_t Data[8];
	uint32_t pTxMailbox;
	CAN_TxHeaderTypeDef TxMeg;
	
	TxMeg.StdId = CAN_GIMBAL_ALL_ID;
	TxMeg.IDE = CAN_ID_STD;
	TxMeg.RTR = CAN_RTR_DATA;
	TxMeg.DLC = 0x08;
	Data[0] = motor1 >> 8;
	Data[1] = motor1;
	Data[2] = motor2 >> 8;
	Data[3] = motor2;
	Data[4] = motor3 >> 8;
	Data[5] = motor3;
	Data[6] = motor4 >> 8;
	Data[7] = motor4;

	HAL_CAN_AddTxMessage(&hcan1, &TxMeg, Data, &pTxMailbox);
}

//���͵���Ħ���ֵ�Դ��Ϣ
void CAN_FRIC_INFO(uint8_t mode)
{
	uint8_t Data[8];
	uint32_t pTxMailbox;
	CAN_TxHeaderTypeDef TxMeg;
	
	TxMeg.StdId = 0x301;
	TxMeg.IDE = CAN_ID_STD;
	TxMeg.RTR = CAN_RTR_DATA;
	TxMeg.DLC = 0x01;
  Data[0] = mode;
	Data[1] = 0;
  Data[2] = 0 ;
	Data[3] = 0;
	Data[4] = 0;
	Data[5] = 0;
	Data[6] = 0;
	Data[7] = 0;
	
	HAL_CAN_AddTxMessage(&hcan1, &TxMeg, Data, &pTxMailbox);
}

//���ص��̵��������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i)
{
	return &motor_chassis[(i & 0x03)];//i��0x03ֻ����0~3����ֹ�������
}
//���ز������������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Trigger_Motor_Measure_Point(void)
{
	return &motor_trigger;
}
//����TOF������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const tof_data_t *get_tof_Info_Measure_Point(void)
{
	return &tof_data;
}
//���������Ǳ�����ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const gyro_info_t *get_GYRO_Measure_Point(void)
{
	return &gyro_info;
}
