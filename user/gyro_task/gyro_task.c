#include "gyro_task.h"
#include "can_receive.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

uint32_t gyro_high_water;

void GYRO_task(void *pvParameters)
{
	while(1)
	{
		gyro_high_water = uxTaskGetStackHighWaterMark(NULL);
	}
}
