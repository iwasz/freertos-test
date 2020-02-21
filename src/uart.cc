/****************************************************************************
 *                                                                          *
 *  Author : lukasz.iwaszkiewicz@gmail.com                                  *
 *  ~~~~~~~~                                                                *
 *  License : see COPYING file for details.                                 *
 *  ~~~~~~~~~                                                               *
 ****************************************************************************/

#include "uart.h"
#include "gsl/gsl_assert"
#include "portmacro.h"
#include "projdefs.h"
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <gsl/gsl>
#include <semphr.h>
#include <stm32l4xx_hal.h>
#include <stream_buffer.h>
#include <sys/_stdint.h>

namespace uart {
namespace {
        UART_HandleTypeDef huart2{};

        DMA_HandleTypeDef txDma{};
        SemaphoreHandle_t txSemaphore{};

        DMA_HandleTypeDef rxDma{};
        SemaphoreHandle_t rxSemaphore{};

        StreamBufferHandle_t xStreamBuffer{};

        // extern "C" void lineReceivingTask (void *pvParameters);

} // namespace

/****************************************************************************/

void init ()
{
        Expects (!txSemaphore);
        txSemaphore = xSemaphoreCreateBinary ();

        Expects (!rxSemaphore);
        rxSemaphore = xSemaphoreCreateBinary ();

        xStreamBuffer = xStreamBufferCreate (100, 1);
        // xTaskCreate (lineReceivingTask, nullptr, configMINIMAL_STACK_SIZE, nullptr, tskIDLE_PRIORITY + 1, nullptr);

        __HAL_RCC_GPIOA_CLK_ENABLE ();
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

        __HAL_RCC_USART2_CLK_ENABLE ();
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

        // Set to defaults.
        huart2.Instance->CR1 = 0x00;
        huart2.Instance->CR2 = 0x04;
        huart2.Instance->CR3 = 0x08;

        if (HAL_UART_Init (&huart2) != HAL_OK) {
                while (true) {
                }
        }

        HAL_NVIC_SetPriority (USART2_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
        HAL_NVIC_EnableIRQ (USART2_IRQn);

        // For TX transfers
        __HAL_RCC_DMA1_CLK_ENABLE ();
        txDma.Instance = DMA1_Channel7;
        txDma.Init.Request = DMA_REQUEST_2;
        txDma.Init.Direction = DMA_MEMORY_TO_PERIPH;
        txDma.Init.PeriphInc = DMA_PINC_DISABLE;
        txDma.Init.MemInc = DMA_MINC_ENABLE;
        txDma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        txDma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        txDma.Init.Mode = DMA_NORMAL;
        txDma.Init.Priority = DMA_PRIORITY_LOW;

        // __HAL_LINKDMA (&huart2, hdmatx, txDma);
        HAL_DMA_Init (&txDma);

        __HAL_DMA_DISABLE_IT (&txDma, DMA_IT_HT);
        __HAL_DMA_ENABLE_IT (&txDma, (DMA_IT_TC | DMA_IT_TE));

        HAL_NVIC_SetPriority (DMA1_Channel7_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
        HAL_NVIC_EnableIRQ (DMA1_Channel7_IRQn);

        // For RX transfers
        rxDma.Instance = DMA1_Channel6;
        rxDma.Init.Request = DMA_REQUEST_2;
        rxDma.Init.Direction = DMA_PERIPH_TO_MEMORY;
        rxDma.Init.PeriphInc = DMA_PINC_DISABLE;
        rxDma.Init.MemInc = DMA_MINC_ENABLE;
        rxDma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        rxDma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        rxDma.Init.Mode = DMA_NORMAL;
        rxDma.Init.Priority = DMA_PRIORITY_LOW;

        // __HAL_LINKDMA (&huart2, hdmarx, rxDma);
        HAL_DMA_Init (&rxDma);

        // Half transfer
        __HAL_DMA_DISABLE_IT (&rxDma, DMA_IT_HT);

        // Full transfer (transfer complete) and transfer error
        __HAL_DMA_ENABLE_IT (&rxDma, (DMA_IT_TC | DMA_IT_TE));

        HAL_NVIC_SetPriority (DMA1_Channel6_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
        HAL_NVIC_EnableIRQ (DMA1_Channel6_IRQn);
}

/****************************************************************************/

static void DMA_SetConfig (DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
        /* Clear all flags */
        hdma->DmaBaseAddress->IFCR = (DMA_ISR_GIF1 << (hdma->ChannelIndex & 0x1cU));

        /* Configure DMA Channel data length */
        hdma->Instance->CNDTR = DataLength;

        /* Memory to Peripheral */
        if ((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH) {
                /* Configure DMA Channel destination address */
                hdma->Instance->CPAR = DstAddress;

                /* Configure DMA Channel source address */
                hdma->Instance->CMAR = SrcAddress;
        }
        /* Peripheral to Memory */
        else {
                /* Configure DMA Channel source address */
                hdma->Instance->CPAR = SrcAddress;

                /* Configure DMA Channel destination address */
                hdma->Instance->CMAR = DstAddress;
        }
}

/****************************************************************************/

void lowLevelSend (uint8_t const *pData, uint16_t Size)
{
        __HAL_DMA_DISABLE (&txDma);
        DMA_SetConfig (&txDma, (uint32_t)pData, (uint32_t)&huart2.Instance->TDR, Size);
        __HAL_DMA_ENABLE (&txDma);

        /* Clear the TC flag in the ICR register */
        __HAL_UART_CLEAR_FLAG (&huart2, UART_CLEAR_TCF);

        /* Enable the DMA transfer for transmit request by setting the DMAT bit
        in the UART CR3 register */
        SET_BIT (huart2.Instance->CR3, USART_CR3_DMAT);
}

/****************************************************************************/

void lowLevelReceive (uint8_t const *pData, uint16_t Size)
{
        __HAL_DMA_DISABLE (&rxDma);
        DMA_SetConfig (&rxDma, (uint32_t)&huart2.Instance->RDR, (uint32_t)pData, Size);
        __HAL_DMA_ENABLE (&rxDma);

        // TODO don't turn the ityerrupt on
        /* Enable the UART Parity Error Interrupt */
        // SET_BIT (huart2.Instance->CR1, USART_CR1_PEIE);

        /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
        // SET_BIT (huart2.Instance->CR3, USART_CR3_EIE);

        /* Enable the DMA transfer for the receiver request by setting the DMAR bit
        in the UART CR3 register */
        SET_BIT (huart2.Instance->CR3, USART_CR3_DMAR);
}

/****************************************************************************/

Status send (uint8_t const *data, size_t len, TickType_t timeout)
{
        if (len == 0) {
                return Status::OK;
        }

        Expects (txSemaphore);

        /*
         * This "clears" the semaphore. If at this point this semaphore was already "given", the next call
         * (after the actual sending) wound immediately return without waiting for the ISR. Had we used a
         * regular boolean variable, we would write "variable = false" here.
         */
        xSemaphoreTake (txSemaphore, 0);
        lowLevelSend (data, len);

        /* Block on the semaphore to wait for the transmission to complete.  If the semaphore is obtained then xReturn will
                      get set to pdPASS.  If the semaphore take operation times out then xReturn will get set to pdFAIL.Note that, if the
                      interrupt occurs between UART_low_level_send() being called, and xSemaphoreTake() being called, thenthe event will
                      be latched in the binary semaphore, and the call to xSemaphoreTake() will return immediately.*/
        return (xSemaphoreTake (txSemaphore, timeout) == pdTRUE) ? (Status::OK) : (Status::TIMEOUT);
}

/****************************************************************************/

Status receive (uint8_t *data, size_t len, TickType_t timeout)
{
        Expects (rxSemaphore);
        xSemaphoreTake (rxSemaphore, 0);
        lowLevelReceive (data, len);

        if (xSemaphoreTake (rxSemaphore, timeout) == pdFALSE) {
                uint32_t isr = huart2.Instance->ISR;

                // Only one of the errors is returned, even thouh more than 1 can be present.
                if ((isr & USART_ISR_PE) != 0) {
                        return Status::PARITY_ERROR;
                }

                if ((isr & USART_ISR_FE) != 0) {
                        return Status::FRAMING_ERROR;
                }

                if ((isr & USART_ISR_NE) != 0) {
                        return Status::NOISE_ERROR;
                }

                if ((isr & USART_ISR_ORE) != 0) {
                        return Status::OVERRUN_ERROR;
                }

                return Status::TIMEOUT;
        }

        return Status::OK;
}

/****************************************************************************/

Vector receiveLine (uint8_t *data, size_t maxlen, TickType_t timeout, LineEnd lineEnd)
{
        Expects (xStreamBuffer);
        SET_BIT (huart2.Instance->CR1, USART_CR1_RXNEIE);

        Vector line;
        uint8_t c{};

        while (true) {
                if (xStreamBufferReceive (xStreamBuffer, &c, 1, timeout) == 0) {
                        return line; // return TIMEOUT;
                }

                line.push_back (c);

                if ((lineEnd == LineEnd::CR_LF && line.size () >= 2 && c == '\n' && line.at (line.size () - 2) == '\r')
                    || (((lineEnd == LineEnd::EITHER && (c == '\r' || c == '\n')) || (lineEnd == LineEnd::LF && c == '\n')
                         || (lineEnd == LineEnd::CR && c == '\r'))
                        && !line.empty ())) {
                        break;
                }
        }

        return line;
}

bool clearLineBuffer () { return xStreamBufferReset (xStreamBuffer) == pdPASS; }

/**
 * Uart TX dma.
 */
extern "C" void DMA1_Channel7_IRQHandler ()
{
        uint32_t flag_it = txDma.DmaBaseAddress->ISR;
        uint32_t source_it = txDma.Instance->CCR;

        /* Half Transfer Complete Interrupt management ******************************/
        if ((0U != (flag_it & (DMA_FLAG_HT1 << (txDma.ChannelIndex & 0x1cU)))) && (0U != (source_it & DMA_IT_HT))) {

                /* Disable the half transfer interrupt if the DMA mode is not CIRCULAR */
                // if ((dmaTx.Instance->CCR & DMA_CCR_CIRC) == 0U) {
                //         /* Disable the half transfer interrupt */
                //         __HAL_DMA_DISABLE_IT (&dmaTx, DMA_IT_HT);
                // }

                /* Clear the half transfer complete flag */
                txDma.DmaBaseAddress->IFCR = DMA_ISR_HTIF1 << (txDma.ChannelIndex & 0x1cU);

                /* DMA peripheral state is not updated in Half Transfer */
                /* but in Transfer Complete case */
                // if (dmaTx.XferHalfCpltCallback != nullptr) {
                //         /* Half transfer callback */
                //         dmaTx.XferHalfCpltCallback (&dmaTx);
                // }
        }

        /* Transfer Complete Interrupt management ***********************************/
        else if ((0U != (flag_it & (DMA_FLAG_TC1 << (txDma.ChannelIndex & 0x1cU)))) && (0U != (source_it & DMA_IT_TC))) {

                // if ((dmaTx.Instance->CCR & DMA_CCR_CIRC) == 0U) {
                //         /* Disable the transfer complete and error interrupt */
                //         __HAL_DMA_DISABLE_IT (&dmaTx, DMA_IT_TE | DMA_IT_TC);

                //         /* Change the DMA state */
                //         dmaTx.State = HAL_DMA_STATE_READY;
                // }

                /* Clear the transfer complete flag */
                txDma.DmaBaseAddress->IFCR = (DMA_ISR_TCIF1 << (txDma.ChannelIndex & 0x1cU));

                /* Process Unlocked */
                // __HAL_UNLOCK (&dmaTx);

                // if (dmaTx.XferCpltCallback != nullptr) {
                //         /* Transfer complete callback */
                //         dmaTx.XferCpltCallback (&dmaTx);
                // }

                BaseType_t xHigherPriorityTaskWoken = pdFALSE;

                /*
                 * Give the Tx semaphore to signal the end of the transmission.  If a taskis Blocked
                 * waiting for the semaphore then the task will be removed fromthe Blocked state.
                 */
                xSemaphoreGiveFromISR (txSemaphore, &xHigherPriorityTaskWoken);
                portYIELD_FROM_ISR (xHigherPriorityTaskWoken);
        }

        /* Transfer Error Interrupt management **************************************/
        else if ((0U != (flag_it & (DMA_FLAG_TE1 << (txDma.ChannelIndex & 0x1cU)))) && (0U != (source_it & DMA_IT_TE))) {
                /* When a DMA transfer error occurs */
                /* A hardware clear of its EN bits is performed */
                /* Disable ALL DMA IT */
                __HAL_DMA_DISABLE_IT (&txDma, (DMA_IT_TC | DMA_IT_HT | DMA_IT_TE));

                /* Clear all flags */
                txDma.DmaBaseAddress->IFCR = (DMA_ISR_GIF1 << (txDma.ChannelIndex & 0x1cU));

                /* Update error code */
                // dmaTx.ErrorCode = HAL_DMA_ERROR_TE;

                /* Change the DMA state */
                // dmaTx.State = HAL_DMA_STATE_READY;

                /* Process Unlocked */
                // __HAL_UNLOCK (&dmaTx);

                // if (dmaTx.XferErrorCallback != NULL) {
                //         /* Transfer error callback */
                //         dmaTx.XferErrorCallback (&dmaTx);
                // }

                std::terminate ();
        }
}

/**
 * Uart RX dma.
 */
extern "C" void DMA1_Channel6_IRQHandler ()
{
        uint32_t flag_it = rxDma.DmaBaseAddress->ISR;
        uint32_t source_it = rxDma.Instance->CCR;

        // Half Transfer Complete
        if ((0U != (flag_it & (DMA_FLAG_HT1 << (rxDma.ChannelIndex & 0x1cU)))) && (0U != (source_it & DMA_IT_HT))) {
                // Clear the half transfer complete flag
                rxDma.DmaBaseAddress->IFCR = DMA_ISR_HTIF1 << (rxDma.ChannelIndex & 0x1cU);
        }

        // Transfer Complete
        else if ((0U != (flag_it & (DMA_FLAG_TC1 << (rxDma.ChannelIndex & 0x1cU)))) && (0U != (source_it & DMA_IT_TC))) {

                // Clear the transfer complete flag
                rxDma.DmaBaseAddress->IFCR = (DMA_ISR_TCIF1 << (rxDma.ChannelIndex & 0x1cU));
                BaseType_t xHigherPriorityTaskWoken = pdFALSE;

                /*
                 * Give the Tx semaphore to signal the end of the transmission.  If a taskis Blocked
                 * waiting for the semaphore then the task will be removed fromthe Blocked state.
                 */
                xSemaphoreGiveFromISR (rxSemaphore, &xHigherPriorityTaskWoken);
                portYIELD_FROM_ISR (xHigherPriorityTaskWoken);
        }

        // Transfer Error
        else if ((0U != (flag_it & (DMA_FLAG_TE1 << (rxDma.ChannelIndex & 0x1cU)))) && (0U != (source_it & DMA_IT_TE))) {
                /* When a DMA transfer error occurs */
                /* A hardware clear of its EN bits is performed */
                /* Disable ALL DMA IT */
                __HAL_DMA_DISABLE_IT (&rxDma, (DMA_IT_TC | DMA_IT_HT | DMA_IT_TE));

                /* Clear all flags */
                rxDma.DmaBaseAddress->IFCR = (DMA_ISR_GIF1 << (rxDma.ChannelIndex & 0x1cU));
                std::terminate ();
        }
}

/****************************************************************************/

extern "C" void USART2_IRQHandler ()
{
        uint32_t isrflags = READ_REG (huart2.Instance->ISR);
        uint32_t cr1its = READ_REG (huart2.Instance->CR1);

        if (((isrflags & USART_ISR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET)) {
                auto c = (uint8_t) (huart2.Instance->RDR & (uint8_t)0x00FF);
                BaseType_t higherPriorityTaskWoken{};
                Expects (xStreamBuffer);
                xStreamBufferSendFromISR (xStreamBuffer, &c, 1, &higherPriorityTaskWoken);
                portYIELD_FROM_ISR (higherPriorityTaskWoken);
                return;
        }

        /*
         * Only to indicate, that some other IRQ source is turned on. I want only RXNE
         * for efficiency reasons.
         */
        std::terminate ();
}

// extern "C" void lineReceivingTask (void *pvParameters) {}

} // namespace uart