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

/* app includes. */
#include "ultrasonic_rangefinder.h"
#include "alertor.h"

/* The highest available interrupt priority. */
#define timerHIGHEST_PRIORITY			( 0 )
#define SOUND_VELOCITY	340.0f
#define ALERT_DISTANCE	180.0f
#define TICK_COUNT_ALERT	3000
#define COUNT_ALERT				3

#define FAST_SAMPLING			30
#define NORMAL_SAMPLING		1500

/**
	*
	*Distance = time * SOUND_VELOCITY / 2
	**/

static char start_flags, flags;
static unsigned short usThisCount = 0, usLastCount = 0;

void vURFSensor( void *pvParameters )
{
    TickType_t xThisTime, xLastTime;
    unsigned char count;
    float distance;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    unsigned short usDifference = 0;

    /* GPA 5 control the sensor */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_RESET);

    /*enable EXTI4 PA4*/
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource4);

    EXTI_InitStructure.EXTI_Line = EXTI_Line4;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init( &NVIC_InitStructure);

    for(;;) {
#if 0
        if (alert_flag & RANGEFINDER_ALERT) {
            vTaskDelay(500);
            continue;
        }
#endif

//        printf("vURFSensor task\n\r");
        vTaskDelay(2);
        /*Enable the sensor, Output H-level  __/````\__ time > 10uS */
        start_flags = 1;
        GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_SET);
				vTaskDelay(3);
        GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_RESET);
        vTaskDelay(200);
        if(usThisCount > usLastCount)
            usDifference = usThisCount - usLastCount;
        else
            usDifference = 0xffff - (usLastCount - usThisCount);
        distance = usDifference * SOUND_VELOCITY / (2 * 1000.0f);//Period = 10uS
        printf("distance: %3.3fcm \n\r", distance);
        if(distance < ALERT_DISTANCE) {
            if(count == 0) {
                xThisTime = xTaskGetTickCount();
                count++;
            } else if(count >= 1) {
                xLastTime = xThisTime;
                xThisTime = xTaskGetTickCount();
                if (TICK_COUNT_ALERT < (xThisTime - xLastTime)) {
                    count = 0;
                } else {
                    count++;
                }
            }
            if(count > COUNT_ALERT) {
                alert_flag |= RANGEFINDER_ALERT;
                count = 0;
            }
            printf("dis: %d \n\r", xThisTime - xLastTime);
            printf("count : %d \n\r", count);
            vTaskDelay(FAST_SAMPLING);
        } else
            vTaskDelay(NORMAL_SAMPLING);
    }
}
/*-----------------------------------------------------------*/

void EXTI4_IRQHandler( void )
{
    /* Capture the free running timer 3 value as we enter the interrupt. */
    if(start_flags) {
        usLastCount = TIM3->CNT;
        start_flags = 0;
    } else
        usThisCount = TIM3->CNT;

    /* Remember what the timer value was this time through, so we can calculate
    the difference the next time through. */

    if(EXTI_GetITStatus(EXTI_Line4) != RESET) {
        EXTI_ClearITPendingBit(EXTI_Line4);
    }
}
