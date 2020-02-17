#include "app_main.h"
#include "FreeRTOS.h"
#include "main.h"
#include "semphr.h"
#include "task.h"
#include <stdlib.h>

/*****************************************************************************/

SemaphoreHandle_t semaphore = NULL;

void SysTick_Handler ()
{
        if (!semaphore) {
                return;
        }

        BaseType_t pxHigherPriorityTaskWoken;
        if (xSemaphoreGiveFromISR (semaphore, &pxHigherPriorityTaskWoken) != pdTRUE) {
                //                Error_Handler ();
        }
}

/*****************************************************************************/

static void prvFlashTask1 (void *pvParameters)
{
        (void)pvParameters;
        // TickType_t xLastExecutionTime = xTaskGetTickCount ();

        for (;;) {

                if (!semaphore) {
                        continue;
                }

                if (xSemaphoreTake (semaphore, (TickType_t)100 / portTICK_PERIOD_MS) == pdTRUE) {
                        HAL_GPIO_TogglePin (GPIOA, GPIO_PIN_1);

                        static int i = 0;

                        // Blocks every 5th time the semaphore is taken.
                        if ((i++ % 5) == 0) {
                                i = 1;

                                for (int i = 0; i < 8000000; ++i) {
                                }
                        }
                }
        }
}

/*---------------------------------------------------------------------------*/

//static void prvFlashTask2 (void *pvParameters)
//{
//        (void)pvParameters;
//        TickType_t xLastExecutionTime = xTaskGetTickCount ();

//        for (;;) {
//                vTaskDelayUntil (&xLastExecutionTime, (TickType_t)500 / portTICK_PERIOD_MS);
//                HAL_GPIO_TogglePin (GPIOA, GPIO_PIN_1);
//        }
//}

/*---------------------------------------------------------------------------*/

static void prvFlashTask3 (void *pvParameters)
{
        (void)pvParameters;
        TickType_t xLastExecutionTime = xTaskGetTickCount ();

        for (;;) {
                vTaskDelayUntil (&xLastExecutionTime, (TickType_t)100 / portTICK_PERIOD_MS);
                HAL_GPIO_TogglePin (GPIOA, GPIO_PIN_4);
        }
}

/*****************************************************************************/

void app_main (void)
{
        xTaskCreate (prvFlashTask1, "Flash1", 64, NULL, 2, NULL);
        // xTaskCreate (prvFlashTask2, "Flash2", 64, NULL, 3, NULL);
        xTaskCreate (prvFlashTask3, "Flash3", 64, NULL, 4, NULL);

        semaphore = xSemaphoreCreateBinary ();

        vTaskStartScheduler ();

        while (1) {
        }
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook (TaskHandle_t pxTask, char *pcTaskName)
{
        /* If configCHECK_FOR_STACK_OVERFLOW is set to either 1 or 2 then this
        function will automatically get called if a task overflows its stack. */
        (void)pxTask;
        (void)pcTaskName;

        while (1) {
        }
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook (void)
{
        /* If configUSE_MALLOC_FAILED_HOOK is set to 1 then this function will
        be called automatically if a call to pvPortMalloc() fails.  pvPortMalloc()
        is called automatically when a task, queue or semaphore is created. */

        while (1) {
        }
}
/*-----------------------------------------------------------*/

/* configUSE_STATIC_ALLOCATION is set to 1, so the application must provide an
implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
used by the Idle task. */
void vApplicationGetIdleTaskMemory (StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
        /* If the buffers to be provided to the Idle task are declared inside this
        function then they must be declared static - otherwise they will be allocated on
        the stack and so not exists after this function exits. */
        static StaticTask_t xIdleTaskTCB;
        static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

        /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
        state will be stored. */
        *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

        /* Pass out the array that will be used as the Idle task's stack. */
        *ppxIdleTaskStackBuffer = uxIdleTaskStack;

        /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
        Note that, as the array is necessarily of type StackType_t,
        configMINIMAL_STACK_SIZE is specified in words, not bytes. */
        *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
/*-----------------------------------------------------------*/

/* configUSE_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
application must provide an implementation of vApplicationGetTimerTaskMemory()
to provide the memory that is used by the Timer service task. */
void vApplicationGetTimerTaskMemory (StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer,
                                     uint32_t *pulTimerTaskStackSize)
{
        /* If the buffers to be provided to the Timer task are declared inside this
        function then they must be declared static - otherwise they will be allocated on
        the stack and so not exists after this function exits. */
        static StaticTask_t xTimerTaskTCB;
        static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

        /* Pass out a pointer to the StaticTask_t structure in which the Timer
        task's state will be stored. */
        *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

        /* Pass out the array that will be used as the Timer task's stack. */
        *ppxTimerTaskStackBuffer = uxTimerTaskStack;

        /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
        Note that, as the array is necessarily of type StackType_t,
        configMINIMAL_STACK_SIZE is specified in words, not bytes. */
        *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

void *malloc (size_t size) { return pvPortMalloc (size); }

void *calloc (size_t num, size_t size)
{
        (void)num;
        (void)size;
        return NULL;
}

void *realloc (void *ptr, size_t size)
{
        (void)ptr;
        (void)size;
        return NULL;
}

void free (void *ptr) { vPortFree (ptr); }
