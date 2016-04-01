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
#include "alertor.h"

#define ALERT_TIME 5000

unsigned char alert_flag;
unsigned char led_flag;


/*
 * It will be on alert when the flowing conditions
 * are met.
 *					1. Infrared
 *					2. UltrasonicRangefinder
 */

void doAlert(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    led_flag = 1;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);//GPB8 -- B_1A
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOB, &GPIO_InitStructure);//GPB9-- B_1B
    vTaskDelay(1);
    GPIO_WriteBit(GPIOB, GPIO_Pin_8, Bit_SET);
    GPIO_WriteBit(GPIOB, GPIO_Pin_9, Bit_RESET);
}

void disableAlert(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);//GPB8 -- B_1A

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOB, &GPIO_InitStructure);//GP9 -- B_1B
	    vTaskDelay(1);
//    GPIO_WriteBit(GPIOB, GPIO_Pin_8, Bit_RESET);
//    GPIO_WriteBit(GPIOB, GPIO_Pin_9, Bit_RESET);
    led_flag = 0;
}
void vAlertorTask(void * pvParameters)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);//GPB8 -- B_1A

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOB, &GPIO_InitStructure);//GP9 -- B_1B
    led_flag = 1;
    vTaskDelay(2000);
    led_flag = 0;
    for(;;) {

#ifdef ENABLE_LIGHT_SENSOR
        /*disable light sensor*/
        if((alert_flag & INFRARED1_ALERT)
#ifdef INFRARED_SENSOR_2
                &&(alert_flag & INFRARED2_ALERT)
#endif
#ifdef ENBALE_RANGEFINDER_SENSOR
                &&(alert_flag & RANGEFINDER_ALERT)
#endif
          ) {
            doAlert();
						vTaskDelay(15000);
						doAlert();
            printf("Alertor action!! \n\r");
            vTaskDelay(ALERT_TIME);
            alert_flag = 0;//clear flag
            disableAlert();
            vTaskDelay(30000);
        } else
            vTaskDelay(500);
        }
#else  /*--- disable light sensor -----*/


        if((alert_flag & INFRARED1_ALERT)
#ifdef INFRARED_SENSOR_2
                &&(alert_flag & INFRARED2_ALERT)
#endif
#ifdef ENBALE_RANGEFINDER_SENSOR
                &&(alert_flag & RANGEFINDER_ALERT)
#endif
          ) {
            doAlert();
						vTaskDelay(7000);
						doAlert();
            printf("Alertor action!! \n\r");
            vTaskDelay(ALERT_TIME);
            alert_flag = 0;//clear flag
            disableAlert();
            vTaskDelay(500);
        } else
            vTaskDelay(500);
#endif
    }
}
