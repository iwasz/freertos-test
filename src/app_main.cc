/****************************************************************************
 *                                                                          *
 *  Author : lukasz.iwaszkiewicz@gmail.com                                  *
 *  ~~~~~~~~                                                                *
 *  License : see COPYING file for details.                                 *
 *  ~~~~~~~~~                                                               *
 ****************************************************************************/

#include "logging.h"
#include <FreeRTOS.h>
#include <cstdlib>
#include <queue.h>
#include <stm32l4xx_hal.h>
#include <task.h>

// #if configUSE_TICKLESS_IDLE == 2
// // extern "C" void SysTick_Handler ()
// // { /* HAL_GPIO_TogglePin (GPIOA, GPIO_PIN_1); */
// // }
// #endif

/****************************************************************************/

void prvFlashTask1 (void * /* pvParameters */)
{
        TickType_t xLastExecutionTime = xTaskGetTickCount ();

        for (;;) {
                vTaskDelayUntil (&xLastExecutionTime, pdMS_TO_TICKS (10));
                HAL_GPIO_TogglePin (GPIOA, GPIO_PIN_1);
                logging::log ("TESTTEST\r\n");
        }
}

/****************************************************************************/

void prvFlashTask2 (void * /* pvParameters */)
{
        TickType_t xLastExecutionTime = xTaskGetTickCount ();

        for (;;) {
                vTaskDelayUntil (&xLastExecutionTime, pdMS_TO_TICKS (10));
                HAL_GPIO_TogglePin (GPIOA, GPIO_PIN_0);
                logging::log ("HELLO\r\n");
        }
}

/****************************************************************************/

static void prvFlashTask3 (void * /* pvParameters */)
{
        TickType_t xLastExecutionTime = xTaskGetTickCount ();

        for (;;) {
                vTaskDelayUntil (&xLastExecutionTime, pdMS_TO_TICKS (10));
                HAL_GPIO_TogglePin (GPIOA, GPIO_PIN_4);
                logging::log ("Hello\r\n");
        }
}

/*****************************************************************************/

void appMain ()
{
        xTaskCreate (prvFlashTask1, "task1", 256, nullptr, 1, nullptr);
        xTaskCreate (prvFlashTask2, "task2", 256, nullptr, 1, nullptr);
        xTaskCreate (prvFlashTask3, "task3", 256, nullptr, 1, nullptr);

        vTaskStartScheduler ();

        while (true) {
        }
}
