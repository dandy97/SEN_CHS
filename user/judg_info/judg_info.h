#ifndef __JUDG_INFO_H__
#define __JUDG_INFO_H__

#include "stm32f4xx.h"

#define JudgeBufferLength       300
#define JudgeFrameHeader        0xA5        //֡ͷ 

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef  UART3RxDMA_Handler;

typedef union
{
 uint8_t U[4];
 float FF;
 int I;
}FormatTrans;

//����ϵͳ�ṹ��
typedef struct
{
	uint8_t    Lv;               //�ȼ�
	uint16_t   Hp;               //ʣ��Ѫ��
	uint16_t  volt;							//��ѹ
	uint16_t  current; 					//��Դ
	float	    power;						//����
	uint16_t  power_buffer;  		//��������
	uint16_t  shooter_heat;  		//ǹ������
	uint8_t   bullet_freq;			//��Ƶ
	float     bullet_speed;     //����
}InfantryJudge_Struct;

void USART3_Init(void);

unsigned char Get_CRC8_Check_Sum_1(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
unsigned int Verify_CRC8_Check_Sum_1(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum_1(unsigned char *pchMessage, unsigned int dwLength);
uint16_t Get_CRC16_Check_Sum_1(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum_1(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum_1(uint8_t * pchMessage,uint32_t dwLength);

void Judge_Receive(uint8_t *pData);

//���ز���ϵͳ������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const InfantryJudge_Struct *get_Judg_Info_Measure_Point(void);


#endif
