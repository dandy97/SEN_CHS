#include "led.h"
 /**TIM9 GPIO Configuration    
    PE5     ------> TIM9_CH1   ÂÌµÆ
    PE6     ------> TIM9_CH2 	 ºìµÆ
    */
		
void LED_Init(void)
{
	TIM_OC_InitTypeDef sConfigOC;
	TIM_HandleTypeDef htim9;
	
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 168-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 1000;			//1000ms
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init(&htim9);
	
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
 
	HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2);
		
	HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_1);  //ÂÌµÆ
	HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_2);  //ºìµÆ
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
	__HAL_RCC_TIM9_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();	
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF3_TIM9;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

//¿ØÖÆºìµÆÁÁ¶È
void GREEN_LED(uint16_t bright)
{
	TIM9->CCR1 = bright;
}

//¿ØÖÆÂÌµÆÁÁ¶È
void RED_LED(uint16_t bright)
{
	TIM9->CCR2 = bright;
}
