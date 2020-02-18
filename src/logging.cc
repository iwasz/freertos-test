/****************************************************************************
 *                                                                          *
 *  Author : lukasz.iwaszkiewicz@gmail.com                                  *
 *  ~~~~~~~~                                                                *
 *  License : see COPYING file for details.                                 *
 *  ~~~~~~~~~                                                               *
 ****************************************************************************/

#include "logging.h"
#include <algorithm>
#include <charconv>
#include <cstdlib>
#include <cstring>
#include <etl/vector.h>
#include <limits>
#ifndef UNIT_TEST
#include <FreeRTOS.h>
#include <semphr.h>
#include <stm32l4xx_hal.h>
#endif

namespace logging {
namespace {
        etl::vector<char, 256> buffer;
        SemaphoreHandle_t mutex{};
        UART_HandleTypeDef huart2{};

        extern "C" void loggingTask (void * /* pvParameters */)
        {
                while (true) {
                        if (xSemaphoreTake (mutex, portMAX_DELAY)) {
                                // TODO DMA
                                HAL_UART_Transmit (&huart2, reinterpret_cast<uint8_t *> (buffer.data ()), buffer.size (), 100);
                                buffer.clear ();
                                xSemaphoreGive (mutex);
                        }
                }
        }

        /****************************************************************************/

        void usart2Init ()
        {
                huart2.Instance = USART2;
                huart2.Init.BaudRate = 115200;
                huart2.Init.WordLength = UART_WORDLENGTH_8B;
                huart2.Init.StopBits = UART_STOPBITS_1;
                huart2.Init.Parity = UART_PARITY_NONE;
                huart2.Init.Mode = UART_MODE_TX_RX;
                huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
                huart2.Init.OverSampling = UART_OVERSAMPLING_16;
                huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
                huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

                if (HAL_UART_Init (&huart2) != HAL_OK) {
                        while (true) {
                        }
                }
        }
} // namespace

/****************************************************************************/

void init ()
{
        buffer.clear ();

        if (mutex == nullptr) {
                mutex = xSemaphoreCreateMutex ();
        }

        usart2Init ();
        xTaskCreate (loggingTask, "log", 64, nullptr, 0, nullptr);
}

/****************************************************************************/

bool log (gsl::czstring<> str)
{
        auto name = pcTaskGetName (nullptr);
        int nameLen = strlen (name);

        auto ticks = xTaskGetTickCount ();
        std::array<char, std::numeric_limits<decltype (ticks)>::digits10 + 1 + 2> ticksStr{};
        static_assert (sizeof (ticksStr) > 6);
        ticksStr[0] = '[';

        char *b = &ticksStr[1];
        auto [p, ec] = std::to_chars (b, &ticksStr[sizeof (ticksStr) - 1], ticks);

        int ticksStrLen{};
        if (ec == std::errc ()) {
                *p = ']';
                std::advance (p, 1);
                ticksStrLen = std::distance (&ticksStr[0], p);
        }

        if (xSemaphoreTake (mutex, pdMS_TO_TICKS (10))) {

                int len = strlen (str);

                if (buffer.size () + len + nameLen + ticksStrLen + 2 > buffer.max_size ()) {
                        return false;
                }

                auto s = gsl::span<const char>{ticksStr.data (), ticksStrLen};
                std::copy (s.cbegin (), s.cend (), std::back_inserter (buffer));

                buffer.push_back (' ');

                s = gsl::span<const char>{name, nameLen};
                std::copy (s.cbegin (), s.cend (), std::back_inserter (buffer));

                buffer.push_back (':');

                s = gsl::span<const char>{str, len};
                std::copy (s.cbegin (), s.cend (), std::back_inserter (buffer));
                xSemaphoreGive (mutex);
                return true;
        }

        return false;
}

} // namespace logging
