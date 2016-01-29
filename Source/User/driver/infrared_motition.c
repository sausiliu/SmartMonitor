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

#include "infrared_monitor.h"
#include "alertor.h"

/*
 * Task
 */
#ifdef	INFRARED_SENSOR_1
static void vInfrared1Task( void *pvParameters );
#endif
#ifdef INFRARED_SENSOR_2
static void vInfrared2Task( void *pvParameters );
#endif

extern unsigned char alert_flag;

void vInfraredTask(void )
{
#ifdef INFRARED_SENSOR_1
    xTaskCreate( vInfrared1Task, "infrared sensor 1", configMINIMAL_STACK_SIZE, NULL, 6, NULL );
#endif
#ifdef INFRARED_SENSOR_2
    xTaskCreate( vInfrared2Task, "infrared sensor 2", configMINIMAL_STACK_SIZE, NULL, 6, NULL );
#endif
}

#ifdef INFRARED_SENSOR_1
/* infrared sensor 1*/
static void vInfrared1Task( void *pvParameters )
{
    GPIO_InitTypeDef GPIO_InitStructure;
    /* GPB 5 control the sensor */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    for(;;) {

        if(alert_flag & INFRARED1_ALERT) {
            vTaskDelay(3000);
            continue;
        }
        if(Bit_SET == GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5)) {
            vTaskDelay(1000);
            if(Bit_SET == GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5)) {
                alert_flag |= INFRARED1_ALERT;
                printf("GPIO GPB_5 is high\n\r");
                vTaskDelay(3000);
            }
            vTaskDelay(10);
        }
        vTaskDelay(100);
    }
}
#endif

#ifdef INFRARED_SENSOR_2
/* infrared sensor 2*/
static void vInfrared2Task( void *pvParameters )
{
    GPIO_InitTypeDef GPIO_InitStructure;
    /* GPB 4 control the sensor */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    for(;;) {
        if(Bit_SET == GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4)) {
            vTaskDelay(1000);
            if(Bit_SET == GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4)) {
                alert_flag |= INFRARED2_ALERT;
                printf("GPIO GPB_4 is high\n\r");
            }
            vTaskDelay(10);
        }
        vTaskDelay(100);
    }
}
#endif
