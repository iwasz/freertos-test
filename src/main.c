#include "main.h"
#include "app_main.h"

UART_HandleTypeDef huart1;

void SystemClock_Config (void);
static void MX_GPIO_Init (void);
static void MX_USART1_UART_Init (void);

/*****************************************************************************/

int main (void)
{
        HAL_Init ();
        SystemClock_Config ();

        HAL_NVIC_SetPriority (LPTIM1_IRQn, 0xf, 0);
        HAL_NVIC_EnableIRQ (LPTIM1_IRQn);

        // HAL_Delay(1000);
        // HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);

        MX_GPIO_Init ();
        MX_USART1_UART_Init ();

        //        while (1) {
        //                HAL_Delay (500);
        //                HAL_GPIO_TogglePin (GPIOA, GPIO_PIN_4);
        //        }

        app_main ();

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

static void MX_USART1_UART_Init (void)
{
        huart1.Instance = USART1;
        huart1.Init.BaudRate = 115200;
        huart1.Init.WordLength = UART_WORDLENGTH_8B;
        huart1.Init.StopBits = UART_STOPBITS_1;
        huart1.Init.Parity = UART_PARITY_NONE;
        huart1.Init.Mode = UART_MODE_TX_RX;
        huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
        huart1.Init.OverSampling = UART_OVERSAMPLING_16;
        huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
        huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

        if (HAL_UART_Init (&huart1) != HAL_OK) {
                Error_Handler ();
        }
}

/*****************************************************************************/

static void MX_GPIO_Init (void)
{
        GPIO_InitTypeDef GPIO_InitStruct = {0};

        __HAL_RCC_GPIOA_CLK_ENABLE ();

        GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

        HAL_GPIO_WritePin (GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4, 0);
}

// void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)
//{
//        if (htim->Instance == TIM6) {
//                HAL_IncTick ();
//        }
//}

void HAL_LPTIM_AutoReloadMatchCallback (LPTIM_HandleTypeDef *lptim) { HAL_IncTick (); }

void Error_Handler (void)
{
        while (1) {
        }
}

/**
 *
 * Taken from the FreeRTOS itself
 */
void vPortSuppressTicksAndSleep (TickType_t xExpectedIdleTime)
{
        uint32_t ulReloadValue, ulCompleteTickPeriods, ulCompletedSysTickDecrements;
        TickType_t xModifiableIdleTime;

        /* Make sure the SysTick reload value does not overflow the counter. */
        if (xExpectedIdleTime > xMaximumPossibleSuppressedTicks) {
                xExpectedIdleTime = xMaximumPossibleSuppressedTicks;
        }

        /* Stop the SysTick momentarily.  The time the SysTick is stopped for
                    is accounted for as best it can be, but using the tickless mode will
                    inevitably result in some tiny drift of the time maintained by the
                    kernel with respect to calendar time. */
        portNVIC_SYSTICK_CTRL_REG &= ~portNVIC_SYSTICK_ENABLE_BIT;

        /* Calculate the reload value required to wait xExpectedIdleTime
                    tick periods.  -1 is used because this code will execute part way
                    through one of the tick periods. */
        ulReloadValue = portNVIC_SYSTICK_CURRENT_VALUE_REG + (ulTimerCountsForOneTick * (xExpectedIdleTime - 1UL));
        if (ulReloadValue > ulStoppedTimerCompensation) {
                ulReloadValue -= ulStoppedTimerCompensation;
        }

        /* Enter a critical section but don't use the taskENTER_CRITICAL()
                    method as that will mask interrupts that should exit sleep mode. */
        __asm volatile("cpsid i" ::: "memory");
        __asm volatile("dsb");
        __asm volatile("isb");

        /* If a context switch is pending or a task is waiting for the scheduler
                    to be unsuspended then abandon the low power entry. */
        if (eTaskConfirmSleepModeStatus () == eAbortSleep) {
                /* Restart from whatever is left in the count register to complete
                                this tick period. */
                portNVIC_SYSTICK_LOAD_REG = portNVIC_SYSTICK_CURRENT_VALUE_REG;

                /* Restart SysTick. */
                portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;

                /* Reset the reload register to the value required for normal tick
                                periods. */
                portNVIC_SYSTICK_LOAD_REG = ulTimerCountsForOneTick - 1UL;

                /* Re-enable interrupts - see comments above the cpsid instruction()
                                above. */
                __asm volatile("cpsie i" ::: "memory");
        }
        else {
                /* Set the new reload value. */
                portNVIC_SYSTICK_LOAD_REG = ulReloadValue;

                /* Clear the SysTick count flag and set the count value back to
                                zero. */
                portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;

                /* Restart SysTick. */
                portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;

                /* Sleep until something happens.  configPRE_SLEEP_PROCESSING() can
                                set its parameter to 0 to indicate that its implementation contains
                                its own wait for interrupt or wait for event instruction, and so wfi
                                should not be executed again.  However, the original expected idle
                                time variable must remain unmodified, so a copy is taken. */
                xModifiableIdleTime = xExpectedIdleTime;
                configPRE_SLEEP_PROCESSING (xModifiableIdleTime);
                if (xModifiableIdleTime > 0) {
                        __asm volatile("dsb" ::: "memory");
                        __asm volatile("wfi");
                        __asm volatile("isb");
                }
                configPOST_SLEEP_PROCESSING (xExpectedIdleTime);

                /* Re-enable interrupts to allow the interrupt that brought the MCU
                                out of sleep mode to execute immediately.  see comments above
                                __disable_interrupt() call above. */
                __asm volatile("cpsie i" ::: "memory");
                __asm volatile("dsb");
                __asm volatile("isb");

                /* Disable interrupts again because the clock is about to be stopped
                                and interrupts that execute while the clock is stopped will increase
                                any slippage between the time maintained by the RTOS and calendar
                                time. */
                __asm volatile("cpsid i" ::: "memory");
                __asm volatile("dsb");
                __asm volatile("isb");

                /* Disable the SysTick clock without reading the
                                portNVIC_SYSTICK_CTRL_REG register to ensure the
                                portNVIC_SYSTICK_COUNT_FLAG_BIT is not cleared if it is set.  Again,
                                the time the SysTick is stopped for is accounted for as best it can
                                be, but using the tickless mode will inevitably result in some tiny
                                drift of the time maintained by the kernel with respect to calendar
                                time*/
                portNVIC_SYSTICK_CTRL_REG = (portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT);

                /* Determine if the SysTick clock has already counted to zero and
                                been set back to the current reload value (the reload back being
                                correct for the entire expected idle time) or if the SysTick is yet
                                to count to zero (in which case an interrupt other than the SysTick
                                must have brought the system out of sleep mode). */
                if ((portNVIC_SYSTICK_CTRL_REG & portNVIC_SYSTICK_COUNT_FLAG_BIT) != 0) {
                        uint32_t ulCalculatedLoadValue;

                        /* The tick interrupt is already pending, and the SysTick count
                                            reloaded with ulReloadValue.  Reset the
                                            portNVIC_SYSTICK_LOAD_REG with whatever remains of this tick
                                            period. */
                        ulCalculatedLoadValue = (ulTimerCountsForOneTick - 1UL) - (ulReloadValue - portNVIC_SYSTICK_CURRENT_VALUE_REG);

                        /* Don't allow a tiny value, or values that have somehow
                                            underflowed because the post sleep hook did something
                                            that took too long. */
                        if ((ulCalculatedLoadValue < ulStoppedTimerCompensation) || (ulCalculatedLoadValue > ulTimerCountsForOneTick)) {
                                ulCalculatedLoadValue = (ulTimerCountsForOneTick - 1UL);
                        }

                        portNVIC_SYSTICK_LOAD_REG = ulCalculatedLoadValue;

                        /* As the pending tick will be processed as soon as this
                                            function exits, the tick value maintained by the tick is stepped
                                            forward by one less than the time spent waiting. */
                        ulCompleteTickPeriods = xExpectedIdleTime - 1UL;
                }
                else {
                        /* Something other than the tick interrupt ended the sleep.
                                            Work out how long the sleep lasted rounded to complete tick
                                            periods (not the ulReload value which accounted for part
                                            ticks). */
                        ulCompletedSysTickDecrements = (xExpectedIdleTime * ulTimerCountsForOneTick) - portNVIC_SYSTICK_CURRENT_VALUE_REG;

                        /* How many complete tick periods passed while the processor
                                            was waiting? */
                        ulCompleteTickPeriods = ulCompletedSysTickDecrements / ulTimerCountsForOneTick;

                        /* The reload value is set to whatever fraction of a single tick
                                            period remains. */
                        portNVIC_SYSTICK_LOAD_REG = ((ulCompleteTickPeriods + 1UL) * ulTimerCountsForOneTick) - ulCompletedSysTickDecrements;
                }

                /* Restart SysTick so it runs from portNVIC_SYSTICK_LOAD_REG
                                again, then set portNVIC_SYSTICK_LOAD_REG back to its standard
                                value. */
                portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;
                portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;
                vTaskStepTick (ulCompleteTickPeriods);
                portNVIC_SYSTICK_LOAD_REG = ulTimerCountsForOneTick - 1UL;

                /* Exit with interrupts enabled. */
                __asm volatile("cpsie i" ::: "memory");
        }
}

 void vPortSetupTimerInterrupt( void )
{
/* Calculate the constants required to configure the tick interrupt. */
#if( configUSE_TICKLESS_IDLE == 1 )
    {
        ulTimerCountsForOneTick = ( configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ );
        xMaximumPossibleSuppressedTicks = portMAX_24_BIT_NUMBER / ulTimerCountsForOneTick;
        ulStoppedTimerCompensation = portMISSED_COUNTS_FACTOR / ( configCPU_CLOCK_HZ / configSYSTICK_CLOCK_HZ );
    }
#endif /* configUSE_TICKLESS_IDLE */

    /* Stop and clear the SysTick. */
    portNVIC_SYSTICK_CTRL_REG = 0UL;
    portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;

    /* Configure SysTick to interrupt at the requested rate. */
    portNVIC_SYSTICK_LOAD_REG = ( configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ ) - 1UL;
    portNVIC_SYSTICK_CTRL_REG = ( portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT | portNVIC_SYSTICK_ENABLE_BIT );
}
