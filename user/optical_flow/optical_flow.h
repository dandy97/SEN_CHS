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

//无线串口4初始化（光流）
void UART4_Init(void);

//1.0 in = 2.54 cm	
//240DPI	每240点一英寸
#define DPI_POINT_TO_CENTIMETER	(2.54f / 240.0f)
//返回光流变量，通过指针传递方式传递信息
const optic_flow_data_t *get_optic_flow_point(void);

#endif
