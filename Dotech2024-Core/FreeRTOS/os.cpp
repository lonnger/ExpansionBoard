/**
 * @file os.c
 * @author JIANG Yicheng  Dotech2023 (EthenJ@outlook.sg)
 * @brief
 * @version 0.1
 * @date 2022-08-20
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "FreeRTOS.h"
#include "cmsis_compiler.h"
#include "main.h"
#include "task.h"

#define HIGHEST_PRIORITY 0x0U

extern void startUserTasks();  // user should implement this cpp function in UserTask.cpp

extern "C"
{
    void startRTOS(void)
    {
        configASSERT(xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED);  // already started?

        traceSTART();

        startUserTasks();  // create user tasks

        configASSERT(NVIC_GetPriority(SVCall_IRQn) == HIGHEST_PRIORITY);  // SVCall_IRQn should be the highest priority

        vTaskStartScheduler();  // start FreeRTOS scheduler

        configASSERT(0);  // should never reach here
    }

#if (configSUPPORT_STATIC_ALLOCATION == 1)
    /*
      vApplicationGetIdleTaskMemory gets called when configSUPPORT_STATIC_ALLOCATION
      equals to 1 and is required for static memory allocation support.
    */
    void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
    {
        /* Idle task control block and stack */
        static StaticTask_t Idle_TCB;
        static StackType_t Idle_Stack[configMINIMAL_STACK_SIZE];

        *ppxIdleTaskTCBBuffer = &Idle_TCB;
        *ppxIdleTaskStackBuffer = &Idle_Stack[0];
        *pulIdleTaskStackSize = (uint32_t)configMINIMAL_STACK_SIZE;
    }

    /*
      vApplicationGetTimerTaskMemory gets called when configSUPPORT_STATIC_ALLOCATION
      equals to 1 and is required for static memory allocation support.
    */
    void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize)
    {
        /* Timer task control block and stack */
        static StaticTask_t Timer_TCB;
        static StackType_t Timer_Stack[configTIMER_TASK_STACK_DEPTH];

        *ppxTimerTaskTCBBuffer = &Timer_TCB;
        *ppxTimerTaskStackBuffer = &Timer_Stack[0];
        *pulTimerTaskStackSize = (uint32_t)configTIMER_TASK_STACK_DEPTH;
    }
#endif

    /**
     * @brief Override HAL_Delay() to use FreeRTOS delay when not in ISR, to fit HAL_Delay in HAL API
     *
     * @param Delay
     *
     * @attention User should avoid using HAL_Delay(); use vTaskDelay() instead
     */
    // void HAL_Delay(uint32_t Delay)
    // {
    //     if (__get_IPSR() == 0)
    //     {
    //         /* Not in ISR */
    //         vTaskDelay(Delay);  // use FreeRTOS delay
    //     }
    //     else
    //     {
    //         /* In ISR, use Legacy HAL_Delay() */
    //         uint32_t tickstart = HAL_GetTick();
    //         uint32_t wait = Delay;

    //         extern uint32_t uwTickFreq;

    //         /* Add a freq to guarantee minimum wait */
    //         if (wait < HAL_MAX_DELAY)
    //         {
    //             wait += uwTickFreq;
    //         }

    //         while ((HAL_GetTick() - tickstart) < wait)
    //         {
    //         }
    //     }
    // }

    /**
     * @brief This function handles System tick timer.
     * @remark Make FreeRTOS workable when SysTick_Handler is not defined in STM32xxxx_it.c
     */
    __WEAK void SysTick_Handler(void)
    {
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        {
            extern void xPortSysTickHandler(void);
            xPortSysTickHandler();
        }
    }
}
