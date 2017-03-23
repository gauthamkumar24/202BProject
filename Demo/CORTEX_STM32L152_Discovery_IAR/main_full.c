/*
FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
All rights reserved

VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

This file is part of the FreeRTOS distribution.

FreeRTOS is free software; you can redistribute it and/or modify it under
the terms of the GNU General Public License (version 2) as published by the
Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

***************************************************************************
>>!   NOTE: The modification to the GPL is included to allow you to     !<<
>>!   distribute a combined work that includes FreeRTOS without being   !<<
>>!   obliged to provide the source code for proprietary components     !<<
>>!   outside of the FreeRTOS kernel.                                   !<<
***************************************************************************

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

http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
the FAQ page "My application does not run, what could be wrong?".  Have you
defined configASSERT()?

http://www.FreeRTOS.org/support - In return for receiving this top quality
embedded software for free we request you assist our global community by
participating in the support forum.

http://www.FreeRTOS.org/training - Investing in training allows your team to
be as productive as possible as early as possible.  Now you can receive
FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
Ltd, and the world's leading authority on the world's leading RTOS.

http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
including FreeRTOS+Trace - an indispensable productivity tool, a DOS
compatible FAT file system, and our tiny thread aware UDP/IP stack.

http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
licenses offer ticketed support, indemnification and commercial middleware.

http://www.SafeRTOS.com - High Integrity Systems also provide a safety
engineered and independently SIL3 certified version for use in safety and
mission critical applications that require provable dependability.

1 tab == 4 spaces!
*/

/******************************************************************************
* NOTE 1:  This project provides two demo applications.  A low power tickless
* project, and a more comprehensive test and demo application.  The
* configCREATE_LOW_POWER_DEMO setting in FreeRTOSConfig.h is used to
* select between the two.  See the notes on using
* configCREATE_LOW_POWER_DEMO in FreeRTOSConfig.h.  This file implements
* the comprehensive test and demo version.
*
* NOTE 2:  This file only contains the source code that is specific to the
* full demo.  Generic functions, such FreeRTOS hook functions, and functions
* required to configure the hardware, are defined in main.c.
******************************************************************************
*
* main_full() creates all the demo application tasks and a software timer, then
* starts the scheduler.  The web documentation provides more details of the
* standard demo application tasks, which provide no particular functionality,
* but do provide a good example of how to use the FreeRTOS API.
*
* In addition to the standard demo tasks, the following tasks and tests are
* defined and/or created within this file:
*
* "Check" timer - The check software timer period is initially set to three
* seconds.  The callback function associated with the check software timer
* checks that all the standard demo tasks are not only still executing, but
* are executing without reporting any errors.  If the check software timer
* discovers that a task has either stalled, or reported an error, then it
* changes its own execution period from the initial three seconds, to just
* 200ms.  The check software timer callback function also toggles an LED each
* time it is called.  This provides a visual indication of the system status:
* If the LED toggles every three seconds, then no issues have been discovered.
* If the LED toggles every 200ms, then an issue has been discovered with at
* least one task.
*
* See the documentation page for this demo on the FreeRTOS.org web site for
* full information, including hardware setup requirements.
*/

/* Standard includes. */
#include <stdio.h>
#include "math.h"
/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "arm_math.h"

/* Standard demo application includes. */

/* ST library functions. */
#include "stm32l1xx.h"
#include "discover_board.h"
#include "stm32l_discovery_lcd.h"

/* Priorities for the demo application tasks. */
#define mainQUEUE_POLL_PRIORITY				( tskIDLE_PRIORITY + 2UL )
#define mainSEM_TEST_PRIORITY				( tskIDLE_PRIORITY + 1UL )
#define mainBLOCK_Q_PRIORITY				( tskIDLE_PRIORITY + 2UL )

/* A block time of zero simply means "don't block". */
#define mainDONT_BLOCK						( 0UL )
#define PI                                                      3.14


#define mainCHECK_CPUUSAGE_TIMER_PERIOD_MS      ( 1000UL )  //value=5000 for past_mix_freq demo
#define mainTASK_TIMER_PERIOD_MS                ( 20000UL )     //value=5000 for past_mix_freq demos
/*-----------------------------------------------------------*/

/*
* The check timer callback function, as described at the top of this file.
*/
static void CheckUsage(void *pvParameters);
static void prvTask1TimerCallback( TimerHandle_t xTimer );
static void prvTask2TimerCallback( TimerHandle_t xTimer );
static void prvTask3TimerCallback( TimerHandle_t xTimer );

/*
* Configure the LCD, then write welcome message.
*/
static void prvConfigureLCD( void );
/*-----------------------------------------------------------*/
unsigned long ulIdleCycleCount = 0UL;
static int numCheckUsageRuns = 0;

/*-----------------------------------------------------------*/

void main_full( void )
{
    TimerHandle_t xTask1Timer = NULL;
    TimerHandle_t xTask2Timer = NULL;
    TimerHandle_t xTask3Timer = NULL;
    //TimerHandle_t xCheckTimer_Power = NULL;
    
    /* The LCD is only used in the Full demo. */
    prvConfigureLCD();
    //vStartDynamicPriorityTasks();
    
    //vStartGenericQueueTasks( tskIDLE_PRIORITY );+
    
    /* Start all the other standard demo/test tasks.  They have no particular
    functionality, but do demonstrate how to use the FreeRTOS API and test the
    kernel port. */
    
    
    
    
    /* Create the software timer that performs the 'check' functionality,
    as described at the top of this file. */
    xTask1Timer = xTimerCreate( "Task1Timer",                   /* A text name, purely to help debugging. */
    ( mainTASK_TIMER_PERIOD_MS ),  /* The timer period, in this case 3000ms (3s). */
    pdTRUE,                         /* This is an auto-reload timer, so xAutoReload is set to pdTRUE. */
    ( void * ) 0,                   /* The ID is not used, so can be set to anything. */
    prvTask1TimerCallback           /* The callback function that inspects the status of all the other tasks. */
    );
    xTask2Timer = xTimerCreate( "Task2Timer",                   /* A text name, purely to help debugging. */
    ( mainTASK_TIMER_PERIOD_MS ),  /* The timer period, in this case 3000ms (3s). */
    pdTRUE,                         /* This is an auto-reload timer, so xAutoReload is set to pdTRUE. */
    ( void * ) 0,                   /* The ID is not used, so can be set to anything. */
    prvTask2TimerCallback           /* The callback function that inspects the status of all the other tasks. */
    );
    xTask3Timer = xTimerCreate( "Task3Timer",                   /* A text name, purely to help debugging. */
    ( mainTASK_TIMER_PERIOD_MS ),  /* The timer period, in this case 3000ms (3s). */
    pdTRUE,                         /* This is an auto-reload timer, so xAutoReload is set to pdTRUE. */
    ( void * ) 0,                   /* The ID is not used, so can be set to anything. */
    prvTask3TimerCallback           /* The callback function that inspects the status of all the other tasks. */
    );
    
    if( xTask1Timer != NULL )
    {
        xTimerStart( xTask1Timer, mainDONT_BLOCK );
    }
    if( xTask2Timer != NULL )
    {
        xTimerStart( xTask2Timer, mainDONT_BLOCK );
    }
    if( xTask3Timer != NULL )
    {
        xTimerStart( xTask3Timer, mainDONT_BLOCK );
    }
    #if  configPERFORMANCE_SCALING_MODE == 3
        xTaskCreate( CheckUsage, "CPU", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 5, NULL );
    #endif
    
    #if  configPERFORMANCE_SCALING_MODE == 4
        xTaskCreate( CheckUsage, "CPU", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 5, NULL );
    
    #endif
    
    /* Start the scheduler. */
    vTaskStartScheduler();
    
    
    /* If all is well, the scheduler will now be running, and the following line
    will never be reached.  If the following line does execute, then there was
    insufficient FreeRTOS heap memory available for the idle and/or timer tasks
    to be created.  See the memory management section on the FreeRTOS web site
    for more details. */
    for( ;; );
}
/*-----------------------------------------------------------*/
static void CheckUsage(void *pvParameters)
{   static float f=0;
    for(;;)
    {
        numCheckUsageRuns++;
        vTaskDelay( mainCHECK_CPUUSAGE_TIMER_PERIOD_MS );
        CheckUsageCallback(&f);
if (numCheckUsageRuns >= 20) {
            
#if  configPERFORMANCE_SCALING_MODE == 3 
            if(f > 55){
                On_Demand(1);
            }
            if(f < 45){
                On_Demand(0);
            }
#endif
            
#if  configPERFORMANCE_SCALING_MODE == 4
              
     PAST_MixFreq_Governor(f, mainCHECK_CPUUSAGE_TIMER_PERIOD_MS);
#endif
        }
        
    }
}


static void prvTask1TimerCallback( TimerHandle_t xTimer )
{
    static char cBuffer[ 512 ];
    static int j=0;
    
    int i=234;
    j++;
    
    GPIO_TOGGLE( LD_GPIO_PORT, LD_GREEN_GPIO_PIN );
    
    i=i/45;
    i=i*32;
    for(i=0;i<10000;i++)
    i++;
    /*k==300000 for past_mix_freq demo*/
    for (int k=1;k<=1300000;k++){
        i=i/45;
        i*=4587;
        i/=9;
        i*=237;
    }
    
    //api to get runtime stats
    //vTaskGetRunTimeStats(cBuffer);
    
}

static void prvTask2TimerCallback( TimerHandle_t xTimer )
{
    static char cBuffer[ 512 ];
    static int j=0;
    //uint32_t cpuidletime;
    //uint32_t runtime;
    int i=234;
    j++;
    /* Toggle the check LED to give an indication of the system status.  If
    the LED toggles every mainCHECK_TIMER_PERIOD_MS milliseconds then
    everything is ok.  A faster toggle indicates an error. */
    GPIO_TOGGLE( LD_GPIO_PORT, LD_BLUE_GPIO_PIN );
    
    i=i/45;
    i=i*32;
    for(i=0;i<10000;i++)
    i++;
    /*k==300000 for past_mix_freq demo*/
    for (int k=1;k<=1300000;k++){
        i=i/45;
        i*=4587;
        i/=9;
        i*=237;
    }
    
    
    //vTaskGetRunTimeStats(cBuffer);
    
}

static void prvTask3TimerCallback( TimerHandle_t xTimer )
{
    static char cBuffer[ 512 ];
    static int j=0;
    //uint32_t cpuidletime;
    //uint32_t runtime;
    int i=234;
    
    j++;
    
    i=i/45;
    i=i*32;
    for(i=0;i<10000;i++)
    i++;
    /*k==300000 for past_mix_freq demo*/
    for (int k=1;k<=1300000;k++){
        i=i/45;
        i*=4587;
        i/=9;
        i*=237;
    }
    
    
    //vTaskGetRunTimeStats(cBuffer);
    
}

/*-----------------------------------------------------------*/

static void prvConfigureLCD( void )
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /* Enable necessary clocks. */
    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC, ENABLE );
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_LCD, ENABLE );
    PWR_RTCAccessCmd( ENABLE );
    RCC_LSEConfig( ENABLE );
    RCC_RTCCLKConfig( RCC_RTCCLKSource_LSE );
    RCC_RTCCLKCmd( ENABLE );
    
    /* Configure Port A LCD Output pins as alternate function. */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_8 | GPIO_Pin_9 |GPIO_Pin_10 |GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init( GPIOA, &GPIO_InitStructure );
    
    /* Select LCD alternate function for Port A LCD Output pins. */
    GPIO_PinAFConfig( GPIOA, GPIO_PinSource1, GPIO_AF_LCD );
    GPIO_PinAFConfig( GPIOA, GPIO_PinSource2, GPIO_AF_LCD );
    GPIO_PinAFConfig( GPIOA, GPIO_PinSource3, GPIO_AF_LCD );
    GPIO_PinAFConfig( GPIOA, GPIO_PinSource8, GPIO_AF_LCD );
    GPIO_PinAFConfig( GPIOA, GPIO_PinSource9, GPIO_AF_LCD );
    GPIO_PinAFConfig( GPIOA, GPIO_PinSource10, GPIO_AF_LCD );
    GPIO_PinAFConfig( GPIOA, GPIO_PinSource15, GPIO_AF_LCD );
    
    /* Configure Port B LCD Output pins as alternate function */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init( GPIOB, &GPIO_InitStructure );
    
    /* Select LCD alternate function for Port B LCD Output pins */
    GPIO_PinAFConfig( GPIOB, GPIO_PinSource3, GPIO_AF_LCD );
    GPIO_PinAFConfig( GPIOB, GPIO_PinSource4, GPIO_AF_LCD );
    GPIO_PinAFConfig( GPIOB, GPIO_PinSource5, GPIO_AF_LCD );
    GPIO_PinAFConfig( GPIOB, GPIO_PinSource8, GPIO_AF_LCD );
    GPIO_PinAFConfig( GPIOB, GPIO_PinSource9, GPIO_AF_LCD );
    GPIO_PinAFConfig( GPIOB, GPIO_PinSource10, GPIO_AF_LCD );
    GPIO_PinAFConfig( GPIOB, GPIO_PinSource11, GPIO_AF_LCD );
    GPIO_PinAFConfig( GPIOB, GPIO_PinSource12, GPIO_AF_LCD );
    GPIO_PinAFConfig( GPIOB, GPIO_PinSource13, GPIO_AF_LCD );
    GPIO_PinAFConfig( GPIOB, GPIO_PinSource14, GPIO_AF_LCD );
    GPIO_PinAFConfig( GPIOB, GPIO_PinSource15, GPIO_AF_LCD );
    
    /* Configure Port C LCD Output pins as alternate function */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |GPIO_Pin_11 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init( GPIOC, &GPIO_InitStructure );
    
    /* Select LCD alternate function for Port B LCD Output pins */
    GPIO_PinAFConfig( GPIOC, GPIO_PinSource0, GPIO_AF_LCD );
    GPIO_PinAFConfig( GPIOC, GPIO_PinSource1, GPIO_AF_LCD );
    GPIO_PinAFConfig( GPIOC, GPIO_PinSource2, GPIO_AF_LCD );
    GPIO_PinAFConfig( GPIOC, GPIO_PinSource3, GPIO_AF_LCD );
    GPIO_PinAFConfig( GPIOC, GPIO_PinSource6, GPIO_AF_LCD );
    GPIO_PinAFConfig( GPIOC, GPIO_PinSource7, GPIO_AF_LCD );
    GPIO_PinAFConfig( GPIOC, GPIO_PinSource8, GPIO_AF_LCD );
    GPIO_PinAFConfig( GPIOC, GPIO_PinSource9, GPIO_AF_LCD );
    GPIO_PinAFConfig( GPIOC, GPIO_PinSource10, GPIO_AF_LCD );
    GPIO_PinAFConfig( GPIOC, GPIO_PinSource11, GPIO_AF_LCD );
    
    LCD_GLASS_Init();
    LCD_GLASS_DisplayString( "F'RTOS" );
}

