#include "app_main.h"
#include "FreeRTOS.h"
#include "main.h"
#include "queue.h"
#include "task.h"
#include <stdlib.h>
#include <stm32l4xx_hal.h>

extern UART_HandleTypeDef huart2;
QueueHandle_t printQueue = NULL;

typedef char Message[32];
#define LOGGING

/*****************************************************************************/

static void printTask (void *pvParameters)
{
        (void)pvParameters;
        // TickType_t xLastExecutionTime = xTaskGetTickCount ();

        for (;;) {

                Message m;

                // Wait indefinitely long for a message.
                if (xQueueReceive (printQueue, &m, portMAX_DELAY) == pdPASS) {
                    HAL_UART_Transmit (&huart2, (uint8_t*)&m[0], 20, 1000);
                }
        }
}

/*---------------------------------------------------------------------------*/

static void prvFlashTask1 (void *pvParameters)
{
        (void)pvParameters;
        TickType_t xLastExecutionTime = xTaskGetTickCount ();

        for (;;) {
                vTaskDelayUntil (&xLastExecutionTime, (TickType_t)500 / portTICK_PERIOD_MS);
                HAL_GPIO_TogglePin (GPIOA, GPIO_PIN_1);

#ifdef LOGGING
                if (printQueue) {
                        Message m = {"Hello from task1\r\n"};

                        // Don't have to check the returned value, since we are waiting indefnetly.
                        xQueueSendToBack (printQueue, &m, portMAX_DELAY);
                }
#endif
        }
}

static void prvFlashTask2 (void *pvParameters)
{
        (void)pvParameters;
        TickType_t xLastExecutionTime = xTaskGetTickCount ();

        for (;;) {
                vTaskDelayUntil (&xLastExecutionTime, (TickType_t)500 / portTICK_PERIOD_MS);
                HAL_GPIO_TogglePin (GPIOA, GPIO_PIN_1);

#ifdef LOGGING
                if (printQueue) {
                        Message m = {"HELLO FROM TASK2\r\n"};

                        // Don't have to check the returned value, since we are waiting indefnetly.
                        xQueueSendToBack (printQueue, &m, portMAX_DELAY);
                }
#endif
        }
}

/*---------------------------------------------------------------------------*/

static void prvFlashTask3 (void *pvParameters)
{
        (void)pvParameters;
        TickType_t xLastExecutionTime = xTaskGetTickCount ();
        //        void *p = malloc (65536);
        //        configASSERT(p);
        //        free (p);

        for (;;) {
                vTaskDelayUntil (&xLastExecutionTime, (TickType_t)500 / portTICK_PERIOD_MS);
                HAL_GPIO_TogglePin (GPIOA, GPIO_PIN_4);

#ifdef LOGGING
                if (printQueue) {
                        Message m = {"================\r\n"};

                        // Don't have to check the returned value, since we are waiting indefnetly.
                        xQueueSendToBack (printQueue, &m, portMAX_DELAY);
                }
#endif
        }
}

/*****************************************************************************/

void app_main (void)
{
#ifdef LOGGING
        xTaskCreate (printTask, "print", 64, NULL, 0, NULL);
#endif
        xTaskCreate (prvFlashTask1, "task1", 64, NULL, 1, NULL);
        xTaskCreate (prvFlashTask2, "task2", 64, NULL, 1, NULL);
        xTaskCreate (prvFlashTask3, "task3", 64, NULL, 1, NULL);

        printQueue = xQueueCreate (8, sizeof (Message));

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
