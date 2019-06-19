#ifndef __JUDG_INFO_H__
#define __JUDG_INFO_H__

#include "stm32f4xx.h"

#define JudgeBufferLength       300
#define JudgeFrameHeader        0xA5        //帧头 

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef  UART3RxDMA_Handler;

typedef union
{
 uint8_t U[4];
 float FF;
 int I;
}FormatTrans;

//裁判系统结构体
typedef struct
{
	uint8_t    Lv;               //等级
	uint16_t   Hp;               //剩余血量
	uint16_t  volt;							//电压
	uint16_t  current; 					//电源
	float	    power;						//功率
	uint16_t  power_buffer;  		//缓冲能量
	uint16_t  shooter_heat;  		//枪口热量
	uint8_t   bullet_freq;			//射频
	float     bullet_speed;     //射速
}InfantryJudge_Struct;

void USART3_Init(void);

unsigned char Get_CRC8_Check_Sum_1(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
unsigned int Verify_CRC8_Check_Sum_1(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum_1(unsigned char *pchMessage, unsigned int dwLength);
uint16_t Get_CRC16_Check_Sum_1(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum_1(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum_1(uint8_t * pchMessage,uint32_t dwLength);

void Judge_Receive(uint8_t *pData);

//返回裁判系统变量地址，通过指针方式获取原始数据
const InfantryJudge_Struct *get_Judg_Info_Measure_Point(void);


#endif
