/****************************************************************************
 *                                                                          *
 *  Author : lukasz.iwaszkiewicz@gmail.com                                  *
 *  ~~~~~~~~                                                                *
 *  License : see COPYING file for details.                                 *
 *  ~~~~~~~~~                                                               *
 ****************************************************************************/

#include "app_main.h"
// #include "logging.h"
#include "uart.h"
#include <FreeRTOS.h>
#include <cstdint>
#include <portmacro.h>
#include <stm32l4xx_hal.h>
#include <task.h>

static void MX_GPIO_Init ();
extern "C" void SystemClock_Config ();
extern "C" void Error_Handler ();

/*****************************************************************************/

int main ()
{
        /*
         * This thread is useful:
         * https://www.freertos.org/FreeRTOS_Support_Forum_Archive/November_2018/freertos_Tickless_idle_mode_and_wakeup_interrupts_30a858262fj.html
         */
        __HAL_RCC_SYSCFG_CLK_ENABLE ();
        __HAL_RCC_PWR_CLK_ENABLE ();

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
        // if (HAL_SYSTICK_Config (SystemCoreClock / 100UL) == HAL_OK) {
        //         HAL_NVIC_SetPriority (SysTick_IRQn, 1, 0);
        // }
#endif

        // logging::init ();
        MX_GPIO_Init ();

        uart::init ();
        appMain ();

        while (true) {
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

        HAL_GPIO_WritePin (GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4, GPIO_PIN_RESET);
}

/****************************************************************************/

namespace __gnu_cxx {
void __verbose_terminate_handler ()
{
        while (true) {
        }
}
} // namespace __gnu_cxx
