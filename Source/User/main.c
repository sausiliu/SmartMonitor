/*
    FreeRTOS V8.2.3 - Copyright (C) 2015 Real Time Engineers Ltd.
    All rights reserved

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************
*/
/*
	PC 13  -- LED  PC13 --|<|--~~-o 3.3v
	Sensor: Light
	Sensor: Infrared
	uart1: PA9-TX PA10-RX
*/

/* Standard includes. */
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Library includes. */
#include "stm32f10x_it.h"
#include "stm32f10x_usart.h"

/* Demo app includes. */
#include "BlockQ.h"
#include "death.h"
#include "integer.h"
#include "blocktim.h"
#include "partest.h"
#include "semtest.h"
#include "PollQ.h"
#include "flash.h"
#include "comtest2.h"
#include "serial.h"
#include "semphr.h"
#include "ultrasonic_rangefinder.h"
#include "infrared_monitor.h"
#include "alertor.h"

/* Task priorities. */
#define mainQUEUE_POLL_PRIORITY					( tskIDLE_PRIORITY + 2 )
#define mainCHECK_TASK_PRIORITY					( tskIDLE_PRIORITY + 3 )
#define mainSEM_TEST_PRIORITY						( tskIDLE_PRIORITY + 1 )
#define mainBLOCK_Q_PRIORITY						( tskIDLE_PRIORITY + 2 )
#define mainCREATOR_TASK_PRIORITY       ( tskIDLE_PRIORITY + 3 )
#define mainFLASH_TASK_PRIORITY					( tskIDLE_PRIORITY + 1 )
#define mainCOM_TEST_PRIORITY						( tskIDLE_PRIORITY + 1 )
#define mainINTEGER_TASK_PRIORITY       ( tskIDLE_PRIORITY )

/* The check task uses the sprintf function so requires a little more stack. */
#define mainCHECK_TASK_STACK_SIZE			( configMINIMAL_STACK_SIZE + 50 )

/* Dimensions the buffer into which the jitter time is written. */
#define mainMAX_MSG_LEN						100

/* The time between cycles of the 'check' task. */
#define mainCHECK_DELAY						( ( TickType_t ) 5000 / portTICK_PERIOD_MS )

/* The number of nano seconds between each processor clock. */
#define mainNS_PER_CLOCK ( ( unsigned long ) ( ( 1.0 / ( double ) configCPU_CLOCK_HZ ) * 1000000000.0 ) )

/* Baud rate used by the comtest tasks. */
#define mainCOM_TEST_BAUD_RATE		( 115200 )

/* The LED used by the comtest tasks. See the comtest.c file for more
information. */
#define mainCOM_TEST_LED			( 3 )

/* The maximum number of message that can be waiting for uart at any one
time. */
#define mainUART_QUEUE_SIZE					( 10 )

/*-----------------------------------------------------------*/

/*
 * Configure the clocks, GPIO and other peripherals as required by the demo.
 */
static void prvSetupHardware( void );

/*
 * Retargets the C library printf function to the USART.
 */
int fputc( int ch, FILE *f );

/*
 * Task
 */
static void vLEDTask( void *pvParameters );
#ifdef ENABLE_LIGHT_SENSOR
static void vLightSensorTask( void *pvParameters );
#endif
static void vWDTTask( void *pvParameters );

/*
 * Configures the timers and interrupts for the fast interrupt test as
 * described at the top of this file.
 */
extern void vSetupTimerTest( void );
extern signed portBASE_TYPE xSerialPutChar( xComPortHandle, signed char , TickType_t);
extern unsigned char led_flag;
/*-----------------------------------------------------------*/

/* The queue used to send messages to the uart task. */
QueueHandle_t xUARTQueue;
xSemaphoreHandle xSem, xSem1;

/*-----------------------------------------------------------*/
int main( void )
{
#ifdef DEBUG
    debug();
#endif

    /* Create the queue used by the uart task.  Messages for display on the uart1
    are received via this queue. */
    xUARTQueue = xQueueCreate( mainUART_QUEUE_SIZE, sizeof( xUARTMessage ) );
//	xSem = xSemaphoreCreateBinary();
//	xSem1	= xSemaphoreCreateBinary();
    prvSetupHardware();

    /* Start the standard monitor tasks. */
    vAltStartComTestTasks( mainCOM_TEST_PRIORITY, mainCOM_TEST_BAUD_RATE, mainCOM_TEST_LED );
    xTaskCreate( vLEDTask, "led_test", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL );
#ifdef ENABLE_LIGHT_SENSOR
    xTaskCreate( vLightSensorTask, "light sensor", configMINIMAL_STACK_SIZE, NULL, 6, NULL );
#endif
#ifdef ENBALE_RANGEFINDER_SENSOR
    xTaskCreate( vURFSensor, "UltrasonicRangefinder", configMINIMAL_STACK_SIZE, NULL, 6, NULL );
#endif
//    xTaskCreate( vCheckTask, "Check", mainCHECK_TASK_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY, NULL );
#ifdef ENABLE_INFRARED_SENSOR
    vInfraredTask( );
#endif
    xTaskCreate( vWDTTask, "watch dog task", configMINIMAL_STACK_SIZE, NULL, 6, NULL );
    xTaskCreate( vAlertorTask, "Aletor task", configMINIMAL_STACK_SIZE, NULL, 6, NULL );
    /* The suicide tasks must be created last as they need to know how many
    tasks were running prior to their creation in order to ascertain whether
    or not the correct/expected number of tasks are running at any given time. */
    vCreateSuicidalTasks( mainCREATOR_TASK_PRIORITY );

    /* Configure the timers used by the fast interrupt timer test. */
    vSetupTimerTest();

    /* Start the scheduler. */
    vTaskStartScheduler();

    /* Will only get here if there was not enough heap space to create the
    idle task. */
    return 0;
}
/*-----------------------test-------------------------------*/

void vLEDTask(void * pvParameters)
{
    unsigned char i, j;
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    vTaskDelay(200);

    for(;;) {
        if (led_flag == FAST_LED_FLASH) {
            GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
            vTaskDelay(100);
            GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
            vTaskDelay(100);
        } else {
            for(i=1; i < 20; i++) {
                GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
                vTaskDelay(i);
                GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
                vTaskDelay(20 - i);
            }
            for(; i>=1; i-- ) {
                GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
                vTaskDelay(i);
                GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
                vTaskDelay(20 - i);
            }
        }
    }
}

/*-----------------------------------------------------------*/
#ifdef ENABLE_LIGHT_SENSOR
static void vLightSensorTask( void *pvParameters )
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    for(;;) {
        if(Bit_SET == GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11))
            printf("In night\n\r");
        vTaskDelay(100);
    }
}
/*-----------------------------------------------------------*/
#endif

static void vWDTTask( void *pvParameters )
{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_32);	/*Lock 40KHz */
    IWDG_SetReload(0xFF);
    IWDG_Enable();
    for(;;) {
        vTaskDelay(100);
        IWDG_ReloadCounter();
    }
}

static void prvSetupHardware( void )
{
    /* Start with the clocks in their expected state. */
    RCC_DeInit();

    /* Enable HSE (high speed external clock). */
    RCC_HSEConfig( RCC_HSE_ON );

    /* Wait till HSE is ready. */
    while( RCC_GetFlagStatus( RCC_FLAG_HSERDY ) == RESET ) {
    }

    /* 2 wait states required on the flash. */
    *( ( unsigned long * ) 0x40022000 ) = 0x02;

    /* HCLK = SYSCLK */
    RCC_HCLKConfig( RCC_SYSCLK_Div1 );

    /* PCLK2 = HCLK */
    RCC_PCLK2Config( RCC_HCLK_Div1 );

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config( RCC_HCLK_Div2 );

    /* PLLCLK = 8MHz * 9 = 72 MHz. */
    RCC_PLLConfig( RCC_PLLSource_HSE_Div1, RCC_PLLMul_9 );

    /* Enable PLL. */
    RCC_PLLCmd( ENABLE );

    /* Wait till PLL is ready. */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {
    }

    /* Select PLL as system clock source. */
    RCC_SYSCLKConfig( RCC_SYSCLKSource_PLLCLK );

    /* Wait till PLL is used as system clock source. */
    while( RCC_GetSYSCLKSource() != 0x08 ) {
    }

    /* Enable GPIOA, GPIOB, GPIOC, GPIOD, GPIOE and AFIO clocks */
    RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOC
                            | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE );

    /* SPI2 Periph clock enable */
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_SPI2, ENABLE );

    /* Set the Vector Table base address at 0x08000000 */
    NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x0 );

    NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

    /* Configure HCLK clock as SysTick clock source. */
    SysTick_CLKSourceConfig( SysTick_CLKSource_HCLK );

    vParTestInitialise();
}


/*-----------------------------------------------------------*/
int fputc( int ch, FILE *f )
{
    xSerialPutChar(NULL, ch, 5);
    return ch;
}

/*-----------------------------------------------------------*/

#ifdef  DEBUG
/* Keep the linker happy. */
void assert_failed( unsigned char* pcFile, unsigned long ulLine )
{
    for( ;; ) {
    }
}
#endif
