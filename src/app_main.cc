/****************************************************************************
 *                                                                          *
 *  Author : lukasz.iwaszkiewicz@gmail.com                                  *
 *  ~~~~~~~~                                                                *
 *  License : see COPYING file for details.                                 *
 *  ~~~~~~~~~                                                               *
 ****************************************************************************/

#include "logging.h"
#include "projdefs.h"
#include "uart.h"
#include <FreeRTOS.h>
#include <cstdlib>
#include <exception>
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
                // logging::log ("TESTTEST\r\n");
        }
}

/****************************************************************************/

void prvFlashTask2 (void * /* pvParameters */)
{
        TickType_t xLastExecutionTime = xTaskGetTickCount ();

        for (;;) {
                // vTaskDelayUntil (&xLastExecutionTime, pdMS_TO_TICKS (10));
                // logging::log ("HELLO\r\n");

                static std::array<char, 4> buf{'a', 'b', 'c', 'd'};

                if (!uart::receive (buf, pdMS_TO_TICKS (5000))) {
                        std::terminate ();
                }

                HAL_GPIO_TogglePin (GPIOA, GPIO_PIN_0);

                if (!uart::send (buf, pdMS_TO_TICKS (5000))) {
                        std::terminate ();
                }
        }
}

/****************************************************************************/

static void prvFlashTask3 (void * /* pvParameters */)
{
        TickType_t xLastExecutionTime = xTaskGetTickCount ();

        for (;;) {
                vTaskDelayUntil (&xLastExecutionTime, pdMS_TO_TICKS (100));
                // HAL_GPIO_TogglePin (GPIOA, GPIO_PIN_4);
                // logging::log ("Hello\r\n");

                static std::array<uint8_t, 128> data{};
                std::generate (data.begin (), data.end (), [] () -> uint8_t { return std::rand () % ('z' - 'a' + 1) + 'a'; });

                if (!uart::send (data)) {
                        std::terminate ();
                }
        }
}

/*****************************************************************************/

void appMain ()
{
        // xTaskCreate (prvFlashTask1, "task1", 256, nullptr, 1, nullptr);
        xTaskCreate (prvFlashTask2, "task2", 256, nullptr, 1, nullptr);
        // xTaskCreate (prvFlashTask3, "task3", 256, nullptr, 1, nullptr);

        vTaskStartScheduler ();

        while (true) {
        }
}
