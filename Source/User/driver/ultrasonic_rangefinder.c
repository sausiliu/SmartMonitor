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

/* The highest available interrupt priority. */
#define timerHIGHEST_PRIORITY			( 0 )
#define SOUND_VELOCITY	340.0f

/**
	*
	*Distance = time * SOUND_VELOCITY / 2
	**/

static char start_flags, flags;
static unsigned short usThisCount = 0, usLastCount = 0;


void vURFSensor( void *pvParameters )
{
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

    /*enable EXTI4 PB4*/
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
        printf("vURFSensor task\n\r");
        vTaskDelay(5);
        /*Enable the sensor, Output H-level  __/````\__ time > 10uS */
        start_flags = 1;
        GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_SET);
        vTaskDelay(5);
        GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_RESET);
        vTaskDelay(500);
        if(usThisCount > usLastCount)
            usDifference = usThisCount - usLastCount;
        else
            usDifference = 0xffff - (usLastCount - usThisCount);
//        printf("This: %d \n\r", usThisCount);
//        printf("Last: %d \n\r", usLastCount);
//				printf("Dis: %d \n\r", usDifference);
				distance = usDifference * SOUND_VELOCITY / (2 * 1000.0f);
        printf("distance: %3.3fcm \n\r", distance);
				vTaskDelay(2000);
        if(flags) {
            printf("\n\r");
        }
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

