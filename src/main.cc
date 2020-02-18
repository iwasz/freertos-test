/****************************************************************************
 *                                                                          *
 *  Author : lukasz.iwaszkiewicz@gmail.com                                  *
 *  ~~~~~~~~                                                                *
 *  License : see COPYING file for details.                                 *
 *  ~~~~~~~~~                                                               *
 ****************************************************************************/

#include "app_main.h"
#include <FreeRTOS.h>
#include <cassert>
#include <cstdint>
#include <portmacro.h>
#include <stm32l4xx_hal.h>
#include <task.h>

UART_HandleTypeDef huart2;

static void MX_GPIO_Init ();
static void MX_USART2_UART_Init ();
extern "C" void SystemClock_Config ();
extern "C" void Error_Handler ();

/*****************************************************************************/

#if configUSE_TICKLESS_IDLE == 2
void SysTick_Handler () { HAL_GPIO_TogglePin (GPIOA, GPIO_PIN_1); }
#endif

int main ()
{
        /*
         * This thread is useful:
         * https://www.freertos.org/FreeRTOS_Support_Forum_Archive/November_2018/freertos_Tickless_idle_mode_and_wakeup_interrupts_30a858262fj.html
         */
        HAL_Init ();
        SystemClock_Config ();

#if configUSE_TICKLESS_IDLE == 2
        /*
         * 10ms to disturb sleep mode and thus test if time is tracked correctly.
         * Shortest delay requested in a task is 100ms so we are going to be woken
         * up 9 times on average.
         *
         * Changed to some other values.
         */
        if (HAL_SYSTICK_Config (SystemCoreClock / 100UL) == HAL_OK) {
                HAL_NVIC_SetPriority (SysTick_IRQn, 1, 0);
        }
#endif

        MX_GPIO_Init ();
        MX_USART2_UART_Init ();

        app_main ();

        while (true) {
        }
}

/*****************************************************************************/

static void MX_USART2_UART_Init ()
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
                Error_Handler ();
        }
}

/*****************************************************************************/

static void MX_GPIO_Init ()
{
        GPIO_InitTypeDef GPIO_InitStruct = {0};

        __HAL_RCC_GPIOA_CLK_ENABLE ();

        GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

        HAL_GPIO_WritePin (GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4, GPIO_PIN_SET);
}
