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

unsigned char alert_flag;

/*
 * It will be on alert when the flowing conditions
 * are met.
 *					1. Infrared
 *					2. UltrasonicRangefinder
 */

void doAlert(void)
{
    GPIO_WriteBit(GPIOB, GPIO_Pin_8, Bit_RESET);
    GPIO_WriteBit(GPIOB, GPIO_Pin_9, Bit_RESET);
}
void disableAlert(void)
{

    GPIO_WriteBit(GPIOB, GPIO_Pin_8, Bit_SET);
}
void vAlertorTask(void * pvParameters)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    vTaskDelay(2000);

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
          ){
            doAlert();
						vTaskDelay(1500);
            printf("Alertor action!! \n\r");
            alert_flag = 0;//clear flag
            disableAlert();
        }
#else  /*--- disable light sensor -----*/
				
				
        if((alert_flag & INFRARED1_ALERT)
#ifdef INFRARED_SENSOR_2
                &&(alert_flag & INFRARED2_ALERT)
#endif
#ifdef ENBALE_RANGEFINDER_SENSOR
                &&(alert_flag & RANGEFINDER_ALERT)
#endif
          ){
            doAlert();
						vTaskDelay(1500);
            printf("Alertor action!! \n\r");
            alert_flag = 0;//clear flag
            disableAlert();
        }
#endif
        vTaskDelay(1000);

    }
}
