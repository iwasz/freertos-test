#include "app_main.h"
#include <FreeRTOS.h>
#include <queue.h>
#include <stdlib.h>
#include <stm32l4xx_hal.h>
#include <task.h>

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
                        HAL_UART_Transmit (&huart2, (uint8_t *)&m[0], 20, 1000);
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
