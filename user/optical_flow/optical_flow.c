#include "optical_flow.h"
#include "usart1.h"
#include "stm32f4xx_hal.h"

UART_HandleTypeDef huart4;

uint8_t optic_flow[128];
static optic_flow_data_t optic_flow_data;

//光流串口4初始化
void UART4_Init(void)
{
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(&huart4);

	SET_BIT(huart4.Instance->CR1, USART_CR1_IDLEIE);
	HAL_UART_Receive_DMA(&huart4, (uint8_t *)optic_flow, 128);	
}

int16_t temp = 0;
//DMA1 STREAM2
void UART4_IRQHandler(void)
{
	if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_IDLE) && 
      __HAL_UART_GET_IT_SOURCE(&huart4, UART_IT_IDLE))
    {
			
      uint16_t tmp = huart4.Instance->DR;
      tmp = huart4.Instance->SR;
      tmp--;
      CLEAR_BIT(huart4.Instance->SR, USART_SR_IDLE);
			__HAL_DMA_DISABLE(huart4.hdmarx);
			
     	temp = huart4.hdmarx->Instance->NDTR;  
			if(optic_flow[0] == 0x57 && optic_flow[1] == 0xab && optic_flow[2] == 0x02)
			{
				//uint8_t x = optic_flow[4];
				uint8_t y = optic_flow[5];
				
				//optic_flow_data.DX = -(((x & 0x80)?(-1*(((~(x-1))|0x80)+128)):(x)) * DPI_POINT_TO_CENTIMETER);
				optic_flow_data.DY = ((y & 0x80)?(-1*(((~(y-1))|0x80)+128)):(y)) * DPI_POINT_TO_CENTIMETER;
				
				//车身坐标系 	单位为毫米cm 轨道全长 5486
				//optic_flow_data.DX_DIS += optic_flow_data.DX;
				optic_flow_data.DY_DIS += optic_flow_data.DY;
			}
		}
    DMA1->LIFCR = DMA_FLAG_DMEIF2_6 | DMA_FLAG_FEIF2_6 | DMA_FLAG_HTIF2_6 | DMA_FLAG_TCIF2_6 | DMA_FLAG_TEIF2_6; 
		__HAL_DMA_SET_COUNTER(huart4.hdmarx, 128);
		__HAL_DMA_ENABLE(huart4.hdmarx);
}

//返回光流变量，通过指针传递方式传递信息
const optic_flow_data_t *get_optic_flow_point(void)
{
	return &optic_flow_data;
}
