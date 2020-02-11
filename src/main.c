#include "main.h"
#include "FreeRTOS.h"
#include "app_main.h"
#include "portmacro.h"
#include "task.h"

UART_HandleTypeDef huart1;
LPTIM_HandleTypeDef hlptim1; // For system tick

void SystemClock_Config (void);
static void MX_GPIO_Init (void);
static void MX_USART1_UART_Init (void);

/*****************************************************************************/

int main (void)
{
        HAL_Init ();
        SystemClock_Config ();

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

// void HAL_LPTIM_AutoReloadMatchCallback (LPTIM_HandleTypeDef *lptim) { HAL_IncTick (); }

void Error_Handler (void)
{
        while (1) {
        }
}

/**
 * According to the stm32LPTIMER documentation reading CNT may return unreliable results when
 * LPTIM is fed with an asynchronous clock. I think this is my case, because LPTIM is run using
 * LSI oscillator, while the CPU runs on HSI.
 */
unsigned short getLpTimCounter ()
{
        unsigned short cnt = 0, prevCnt = 0;

        while (prevCnt != (cnt = (unsigned short)(hlptim1.Instance->CNT))) {
                prevCnt = cnt;
        }

        return cnt;
}

void enterSleep (TickType_t tick) { HAL_PWREx_EnterSTOP0Mode (PWR_STOPENTRY_WFI); }
void returnFromSleep (TickType_t tick) {}

/* A fiddle factor to estimate the number of SysTick counts that would have
occurred while the SysTick counter is stopped during tickless idle
calculations. */
#define portMISSED_COUNTS_FACTOR (45UL)
static uint32_t ulStoppedTimerCompensation = portMISSED_COUNTS_FACTOR / (configCPU_CLOCK_HZ / configSYSTICK_CLOCK_HZ);

/**
 * Taken from the FreeRTOS itself and modified. I left the comments almost intact, so
 * they refer to the SysTick timer instead of LPTIM1 which I use.
 */
void vPortSuppressTicksAndSleep (TickType_t xExpectedIdleTime)
{
        uint32_t ulReloadValue, ulCompleteTickPeriods, ulCompletedSysTickDecrements;
        TickType_t xModifiableIdleTime;

        /*
         * We don't have to "Make sure the SysTick reload value does not overflow the counter"
         * since TickType_t == unsigned short which is the same type as LPTIM1 ARR register
         * which is 16 bit unsigned number.
         */

        /* Stop the SysTick momentarily.  The time the SysTick is stopped for
                    is accounted for as best it can be, but using the tickless mode will
                    inevitably result in some tiny drift of the time maintained by the
                    kernel with respect to calendar time. */
        hlptim1.Instance->CR = 0;

        // For my initial configuration this variable equals 1 (1000Hz / 1000Hz).
        unsigned short ulTimerCountsForOneTick = (configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ);

        /* Calculate the reload value required to wait xExpectedIdleTime
                    tick periods.  -1 is used because this code will execute part way
                    through one of the tick periods. */
        ulReloadValue = getLpTimCounter () + (ulTimerCountsForOneTick * (xExpectedIdleTime - 1UL));

        // ???
        //        if (ulReloadValue > ulStoppedTimerCompensation) {
        //                ulReloadValue -= ulStoppedTimerCompensation;
        //        }

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
                //                hlptim1.Instance->ARR = getLpTimCounter ();

                /* Restart SysTick. */
                hlptim1.Instance->CR = LPTIM_CR_CNTSTRT | LPTIM_CR_ENABLE;

                /* Reset the reload register to the value required for normal tick periods. */
                hlptim1.Instance->ARR = ulTimerCountsForOneTick - 1UL;

                /* Re-enable interrupts - see comments above the cpsid instruction() above. */
                __asm volatile("cpsie i" ::: "memory");
        }
        else {
                // Start LPTIM. ARR can be set only when the counter runs.
                hlptim1.Instance->CR = LPTIM_CR_CNTSTRT | LPTIM_CR_ENABLE;

                /* Set the new reload value. */
                hlptim1.Instance->ARR = ulReloadValue;

                /* Clear the SysTick count flag and set the count value back to zero. */
                // hlptim1.Instance->CNT = 0UL;
                // TODO CNT is read only

                /* Sleep until something happens.  configPRE_SLEEP_PROCESSING() can
                                set its parameter to 0 to indicate that its implementation contains
                                its own wait for interrupt or wait for event instruction, and so wfi
                                should not be executed again.  However, the original expected idle
                                time variable must remain unmodified, so a copy is taken. */
                xModifiableIdleTime = xExpectedIdleTime;

                // configPRE_SLEEP_PROCESSING (xModifiableIdleTime);
                // enterSleep (xModifiableIdleTime);
                if (xModifiableIdleTime > 0) {
                        __asm volatile("dsb" ::: "memory");
                        __asm volatile("wfi");
                        __asm volatile("isb");
                }
                // configPOST_SLEEP_PROCESSING (xExpectedIdleTime);
                // returnFromSleep (xModifiableIdleTime);

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
                hlptim1.Instance->CR = 0;

                /* Determine if the SysTick clock has already counted to zero and
                                been set back to the current reload value (the reload back being
                                correct for the entire expected idle time) or if the SysTick is yet
                                to count to zero (in which case an interrupt other than the SysTick
                                must have brought the system out of sleep mode). */
                //                if ((portNVIC_SYSTICK_CTRL_REG & portNVIC_SYSTICK_COUNT_FLAG_BIT) != 0) {
                uint32_t ulCalculatedLoadValue;

                /* The tick interrupt is already pending, and the SysTick count
                                    reloaded with ulReloadValue.  Reset the
                                    portNVIC_SYSTICK_LOAD_REG with whatever remains of this tick
                                    period. */
                ulCalculatedLoadValue = (ulTimerCountsForOneTick - 1UL) - (ulReloadValue - getLpTimCounter ());

                /* Don't allow a tiny value, or values that have somehow
                                    underflowed because the post sleep hook did something
                                    that took too long. */
                if ((ulCalculatedLoadValue < ulStoppedTimerCompensation) || (ulCalculatedLoadValue > ulTimerCountsForOneTick)) {
                        ulCalculatedLoadValue = (ulTimerCountsForOneTick - 1UL);
                }

                // hlptim1.Instance->ARR = ulCalculatedLoadValue;

                /* As the pending tick will be processed as soon as this
                                    function exits, the tick value maintained by the tick is stepped
                                    forward by one less than the time spent waiting. */
                ulCompleteTickPeriods = xExpectedIdleTime - 1UL;
                //                }
                //                else {
                //                        /* Something other than the tick interrupt ended the sleep.
                //                                            Work out how long the sleep lasted rounded to complete tick
                //                                            periods (not the ulReload value which accounted for part
                //                                            ticks). */
                //                        ulCompletedSysTickDecrements = (xExpectedIdleTime * ulTimerCountsForOneTick) - getLpTimCounter ();

                //                        /* How many complete tick periods passed while the processor
                //                                            was waiting? */
                //                        ulCompleteTickPeriods = ulCompletedSysTickDecrements / ulTimerCountsForOneTick;

                //                        /* The reload value is set to whatever fraction of a single tick
                //                                            period remains. */
                //                        hlptim1.Instance->ARR = ((ulCompleteTickPeriods + 1UL) * ulTimerCountsForOneTick) -
                //                        ulCompletedSysTickDecrements;
                //                }

                /* Restart SysTick so it runs from portNVIC_SYSTICK_LOAD_REG
                                again, then set portNVIC_SYSTICK_LOAD_REG back to its standard
                                value. */
                //                hlptim1.Instance->CNT = 0UL;
                hlptim1.Instance->CR = LPTIM_CR_CNTSTRT | LPTIM_CR_ENABLE;
                vTaskStepTick (ulCompleteTickPeriods);
                hlptim1.Instance->ARR = ulTimerCountsForOneTick - 1UL;

                /* Exit with interrupts enabled. */
                __asm volatile("cpsie i" ::: "memory");
        }
}

/**
 * This is called by the FreeRTOS during scheduler startup to configure
 * the system tick timer (not to be confused with the SysTick timer).
 */
void vPortSetupTimerInterrupt ()
{
        __HAL_RCC_LPTIM1_CLK_ENABLE ();

        hlptim1.Instance = LPTIM1;
        hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC; // LSI 0.032MHz
        hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV32;         // Divide by 32 equals 1kHz
        hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
        hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
        hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
        hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
        hlptim1.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
        hlptim1.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;

        if (HAL_LPTIM_Init (&hlptim1) != HAL_OK) {
                Error_Handler ();
        }

        if (HAL_LPTIM_Counter_Start_IT (&hlptim1, (configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ) - 1UL) != HAL_OK) {
                Error_Handler ();
        }

        // Make sure only the autoreload match interrupt is on.
        __HAL_LPTIM_ENABLE_IT (&hlptim1, LPTIM_IT_ARRM);

        __HAL_LPTIM_DISABLE_IT (&hlptim1, LPTIM_IT_ARROK);
        __HAL_LPTIM_DISABLE_IT (&hlptim1, LPTIM_IT_DOWN);
        __HAL_LPTIM_DISABLE_IT (&hlptim1, LPTIM_IT_UP);
        __HAL_LPTIM_DISABLE_IT (&hlptim1, LPTIM_IT_CMPOK);
        __HAL_LPTIM_DISABLE_IT (&hlptim1, LPTIM_IT_EXTTRIG);
        __HAL_LPTIM_DISABLE_IT (&hlptim1, LPTIM_IT_CMPM);

        /*
         * According to port.c SysTick ISR should have the lowest priority, but
         * I don't really understand why. In  FreeRTOSConfig.h I set the xPortSysTickHandler
         * macro to LPTIM1_IRQHandler.
         */
        HAL_NVIC_SetPriority (LPTIM1_IRQn, 0xf, 0);
        HAL_NVIC_EnableIRQ (LPTIM1_IRQn);
}

void LPTIM1_IRQHandler (void)
{
        // Something other than ARR interrupt, ignore.
        if (__HAL_LPTIM_GET_FLAG (&hlptim1, LPTIM_FLAG_ARRM) == RESET) {
                // Clear all but LPTIM_FLAG_ARRM
                __HAL_LPTIM_CLEAR_FLAG (
                        &hlptim1, LPTIM_FLAG_DOWN | LPTIM_FLAG_UP | LPTIM_FLAG_ARROK | LPTIM_FLAG_CMPOK | LPTIM_FLAG_EXTTRIG | LPTIM_FLAG_CMPM);
                return;
        }

        /* Clear Autoreload match flag */
        __HAL_LPTIM_CLEAR_FLAG (&hlptim1, LPTIM_FLAG_ARRM);

        /* The SysTick runs at the lowest interrupt priority, so when this interrupt
            executes all interrupts must be unmasked.  There is therefore no need to
            save and then restore the interrupt mask value as its value is already
            known. */
        portDISABLE_INTERRUPTS ();
        {
                /* Increment the RTOS tick. */
                if (xTaskIncrementTick () != pdFALSE) {
                        /* A context switch is required.  Context switching is performed in
                                    the PendSV interrupt.  Pend the PendSV interrupt. */
                        portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;
                }
        }
        portENABLE_INTERRUPTS ();
}

HAL_StatusTypeDef HAL_InitTick (uint32_t TickPriority)
{
        (void)TickPriority;
        // Freertos will init the main System Tick timer (which is LPTIM1 in this case).
        return HAL_OK;
}

void HAL_IncTick (void) {}

uint32_t HAL_GetTick (void) { return xTaskGetTickCount (); }

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

void HAL_SuspendTick (void)
{
        // I can't let the stm32 cube to disable this iterrupt.
}

void HAL_ResumeTick (void)
{
        // I can't let the stm32 cube to disable this iterrupt.
}
