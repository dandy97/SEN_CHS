#ifndef OPTICAL_FLOW_H
#define OPTICAL_FLOW_H

#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart4;

typedef struct
{
	float DX;
	float DY;
	float DX_DIS;
	float DY_DIS;
}optic_flow_data_t;

//���ߴ���4��ʼ����������
void UART4_Init(void);

//1.0 in = 2.54 cm	
//240DPI	ÿ240��һӢ��
#define DPI_POINT_TO_CENTIMETER	(2.54f / 240.0f)
//���ع���������ͨ��ָ�봫�ݷ�ʽ������Ϣ
const optic_flow_data_t *get_optic_flow_point(void);

#endif
