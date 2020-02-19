/****************************************************************************
 *                                                                          *
 *  Author : lukasz.iwaszkiewicz@gmail.com                                  *
 *  ~~~~~~~~                                                                *
 *  License : see COPYING file for details.                                 *
 *  ~~~~~~~~~                                                               *
 ****************************************************************************/

#include "logging.h"
#include "itoa.h"
#include "stm32l4xx_hal_def.h"
#include <algorithm>
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
        etl::vector<char, MAX_LINE_SIZE> buffer;
        SemaphoreHandle_t mutex{};
        UART_HandleTypeDef huart2{};

        extern "C" void loggingTask (void * /* pvParameters */)
        {
                while (true) {
                        if (xSemaphoreTake (mutex, pdMS_TO_TICKS (10))) {
                                if (buffer.empty ()) {
                                        xSemaphoreGive (mutex);
                                        continue;
                                }

                                HAL_StatusTypeDef status
                                        = HAL_UART_Transmit (&huart2, reinterpret_cast<uint8_t *> (buffer.data ()), buffer.size (), 100);

                                if (status != HAL_OK) {
                                        while (true) {
                                        }
                                }

                                buffer.clear ();
                                xSemaphoreGive (mutex);
                        }
                }
        }

        /****************************************************************************/

        void usart2Init ()
        {
                GPIO_InitTypeDef GPIO_InitStruct = {0};

                __HAL_RCC_USART2_CLK_ENABLE ();
                __HAL_RCC_GPIOA_CLK_ENABLE ();
                GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
                GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
                GPIO_InitStruct.Pull = GPIO_NOPULL;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
                GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
                HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

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
        xTaskCreate (loggingTask, "log", 256, nullptr, 1, nullptr);
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
        itoau (ticks, b, 0);

        int ticksStrLen = strlen (ticksStr.data ());
        ticksStr.at (ticksStrLen) = ']';
        ++ticksStrLen;

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
