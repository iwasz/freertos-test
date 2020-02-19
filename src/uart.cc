/****************************************************************************
 *                                                                          *
 *  Author : lukasz.iwaszkiewicz@gmail.com                                  *
 *  ~~~~~~~~                                                                *
 *  License : see COPYING file for details.                                 *
 *  ~~~~~~~~~                                                               *
 ****************************************************************************/

#include "uart.h"
#include <FreeRTOS.h>

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <semphr.h>
#include <stm32l4xx_hal.h>

namespace uart {
namespace {
        UART_HandleTypeDef huart2{};
        SemaphoreHandle_t txSemaphore{};
} // namespace

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

        if (txSemaphore == nullptr) {
                txSemaphore = xSemaphoreCreateBinary ();
        }
}

// TODO template with container.
BaseType_t send (uint8_t *pucDataSource, size_t uxLength)
{
        BaseType_t xReturn;

        /* Ensure the UART's transmit semaphore is not already available by attempting to take the semaphore without a timeout. */
        xSemaphoreTake (txSemaphore, 0);

        // /* Start the transmission. */
        // // UART_low_level_send (pxUARTInstance, pucDataSource, uxLength);

        // /* Block on the semaphore to wait for the transmission to complete.  If the semaphore is obtained then xReturn will
        //               get set to pdPASS.  If the semaphore take operation times out then xReturn will get set to pdFAIL.Note that, if the
        //               interrupt occurs between UART_low_level_send() being called, and xSemaphoreTake() being called, thenthe event will
        //               be latched in the binary semaphore, and the call to xSemaphoreTake() will return immediately.*/
        // xReturn = xSemaphoreTake (txSemaphore, pxUARTInstance->xTxTimeout);
        return xReturn;
}

BaseType_t receive () { return {}; }

/* The service routine for the UART's transmit end interrupt, which executes after the last byte has been sent to the UART. */
// void xUART_TransmitEndISR (xUART *pxUARTInstance)
// {
//         BaseType_t xHigherPriorityTaskWoken = pdFALSE; /* Clear the interrupt. */

//         UART_low_level_interrupt_clear (pxUARTInstance); /* Give the Tx semaphore to signal the end of the transmission.  If a taskis Blocked
//                                                             waiting for the semaphore then the task will be removed fromthe Blocked state. */
//         xSemaphoreGiveFromISR (pxUARTInstance->xTxSemaphore, &xHigherPriorityTaskWoken);

//         portYIELD_FROM_ISR (xHigherPriorityTaskWoken);
// }

} // namespace uart