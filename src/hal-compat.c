#include <FreeRTOS.h>
#include <assert.h>
#include <portmacro.h>
#include <stdint.h>
#include <stm32l4xx_hal.h>
#include <task.h>

/*****************************************************************************/

void Error_Handler (void)
{
        while (1) {
        }
}

/*****************************************************************************/

void SystemClock_Config (void)
{
        RCC_OscInitTypeDef RCC_OscInitStruct = {0};
        RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
        RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

        /** Initializes the CPU, AHB and APB busses clocks.
         */
        RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
        RCC_OscInitStruct.HSIState = RCC_HSI_ON;
        RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
        RCC_OscInitStruct.LSIState = RCC_LSI_ON;
        RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
        RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
        RCC_OscInitStruct.PLL.PLLM = 1;
        RCC_OscInitStruct.PLL.PLLN = 10;
        RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
        RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
        RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;

        if (HAL_RCC_OscConfig (&RCC_OscInitStruct) != HAL_OK) {
                Error_Handler ();
        }
        /** Initializes the CPU, AHB and APB busses clocks.
         */
        RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
        RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
        RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
        RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
        RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

        if (HAL_RCC_ClockConfig (&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
                Error_Handler ();
        }

        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPTIM1;
        PeriphClkInit.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_LSI;

        if (HAL_RCCEx_PeriphCLKConfig (&PeriphClkInit) != HAL_OK) {
                Error_Handler ();
        }

        /** Configure the main internal regulator output voltage.
         */
        if (HAL_PWREx_ControlVoltageScaling (PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
                Error_Handler ();
        }
}

/*****************************************************************************/

HAL_StatusTypeDef HAL_InitTick (uint32_t TickPriority)
{
        (void)TickPriority;

        /*
         * Freertos will init the main System Tick timer (which is LPTIM1 in this case).
         * TODO Until then, the stm32 cube (hal) will run without any clock so this is
         * not good if any polling (with timeout) function is called. Program will hang
         * in such a case.
         */
        return HAL_OK;
}

/*****************************************************************************/

void HAL_IncTick (void) {}

/*****************************************************************************/

uint32_t HAL_GetTick (void) { return xTaskGetTickCount (); }

/*****************************************************************************/

void HAL_Delay (uint32_t Delay)
{
        uint32_t tickstart = xTaskGetTickCount ();
        uint32_t wait = Delay;

        /* Add a period to guaranty minimum wait */
        if (wait < HAL_MAX_DELAY) {
                wait++;
        }

        while ((xTaskGetTickCount () - tickstart) < wait) {
        }
}

/*****************************************************************************/

void HAL_SuspendTick (void)
{
        // I can't let the stm32 cube to disable this iterrupt.
}

/*****************************************************************************/

void HAL_ResumeTick (void)
{
        // I can't let the stm32 cube to disable this iterrupt.
}
