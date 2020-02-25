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

void readLineTest (void * /* pvParameters */)
{
        using namespace uart;

        while (true) {
                Vector line = receiveLine (nullptr, 0, pdMS_TO_TICKS (5000));

                if (uart::send (line) != uart::Status::OK) {
                        std::terminate ();
                }
        }
}

/**
 * This test reads 16B (using DMA) from the UART, toggles a GPIO if that succeeded, and
 * then sends the buffer back.
 */
void echoOverrunTest (void * /* pvParameters */)
{
        using namespace uart;

        while (true) {
                static std::array<char, 16> buf{};

                /*
                 * Try to receive the data. If it does not get anything in 5s, it stops the reception,
                 * and checks for UART error statuses (ISR register). If no error status was set in the
                 * register, it simply means, that we didn't get anything, and that's no error.
                 *
                 * If you type on the keyboard as you would normally do, the program runs without a problem,
                 * but if you were to paste, say, 1000B of text, you would get an OVERRUN_ERROR. This is because
                 * the program would try to send first 16B it received, and wait for them to be sent. During
                 * this waiting period 17th byte woud land into the RDR, but it wouldn't get read, and 18th byte
                 * would cause the OVERRUN_ERROR. Try to paste the first line of this whole comment. You would
                 * get an OVERUN_ERROR then, and RDR would equal 0x68 which is letter 'h'.
                 */
                if (Status status = receive (buf, pdMS_TO_TICKS (5000)); status != Status::OK && status != Status::TIMEOUT) {
                        std::terminate ();
                }

                HAL_GPIO_TogglePin (GPIOA, GPIO_PIN_0);

                // Send if there is anything to send.
                if (Status status = send (buf, pdMS_TO_TICKS (5000)); status != Status::OK) {
                        std::terminate ();
                }
        }
}

/**
 * This test simply outputs a buffer (using DMA).
 */
void sendBufferTest (void * /* pvParameters */)
{
        TickType_t xLastExecutionTime = xTaskGetTickCount ();

        while (true) {
                /*
                 * It does that once every 100ms. If this delay was commented out, the task would
                 * send the buffer with maximum speed taking short breaks only to generate the data.
                 */
                vTaskDelayUntil (&xLastExecutionTime, pdMS_TO_TICKS (100));

                // Generate 128B of random gibberish.
                static std::array<uint8_t, 128> data{};
                std::generate (data.begin (), data.end (), [] () -> uint8_t { return std::rand () % ('z' - 'a' + 1) + 'a'; });

                // Send 128B and block this thread until it's done.
                if (uart::send (data) != uart::Status::OK) {
                        std::terminate ();
                }
        }
}

/****************************************************************************/

void logA (void * /* params */)
{
        TickType_t xLastExecutionTime = xTaskGetTickCount ();

        while (true) {
                vTaskDelayUntil (&xLastExecutionTime, pdMS_TO_TICKS (10));
                logging::log ("Hello world");
        }
}

void logB (void * /* params */)
{
        TickType_t xLastExecutionTime = xTaskGetTickCount ();

        while (true) {
                vTaskDelayUntil (&xLastExecutionTime, pdMS_TO_TICKS (10));
                logging::log ("Hello TASK B");
        }
}

/*****************************************************************************/

void appMain ()
{
        // xTaskCreate (readLineTest, "task1", 256, nullptr, 1, nullptr);
        // xTaskCreate (echoOverrunTest, "task2", 256, nullptr, 1, nullptr);
        // xTaskCreate (sendBufferTest, "task3", 256, nullptr, 1, nullptr);

        xTaskCreate (logA, "logA", configMINIMAL_STACK_SIZE, nullptr, 0, nullptr);
        xTaskCreate (logB, "logB", configMINIMAL_STACK_SIZE, nullptr, 0, nullptr);

        vTaskStartScheduler ();

        while (true) {
        }
}
