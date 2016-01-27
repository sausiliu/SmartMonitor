/*
 * File: infrared_motition.c
 * The system has two infrared sensors.
 * GPIO Source: PA6 PA7
 *              PA6 - Infrared sensor[1]
 *              PA7 - Infrared sensor[2]
 * GPIO PAx config whit input funcion
*/

/* Scheduler include files. */
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"

/* Library includes. */
#include "stm32f10x_lib.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_map.h"
#include "stm32f10x_it.h"
#include "stm32f10x_rcc.h"

#include "infrared_motition.h"


void vWDTTask( void *pvParameters )
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* GPA 6 control the sensor */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* GPA 7 control the sensor */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    for(;;) {
				if(Bit_SET == GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6))
					printf("GPIO_pin 6 is high\n\r");
				
				if(Bit_SET == GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7))
					printf("GPIO_pin 6 is high\n\r");				
        vTaskDelay(2000);
    }
}

