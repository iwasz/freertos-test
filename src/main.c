#include "main.h"
#include "FreeRTOS.h"
#include "app_main.h"
#include "portmacro.h"
#include "task.h"
#include <assert.h>
#include <stdint.h>

static UART_HandleTypeDef huart1;
static LPTIM_HandleTypeDef hlptim1; // For system tick

static void SystemClock_Config (void);
static void MX_GPIO_Init (void);
static void MX_USART1_UART_Init (void);

void setArr (unsigned short value);

/*****************************************************************************/

void SysTick_Handler () {}

int main (void)
{
        /*
         * This thread is useful:
         * https://www.freertos.org/FreeRTOS_Support_Forum_Archive/November_2018/freertos_Tickless_idle_mode_and_wakeup_interrupts_30a858262fj.html
         */
        HAL_Init ();
        SystemClock_Config ();

        /*
         * 10ms to disturb sleep mode and thus test if time is tracked correctly.
         * Shortest delay requested in a task is 100ms so we are going to be woken
         * up 9 times on average.
         */
        if (HAL_SYSTICK_Config (SystemCoreClock / 100UL) == HAL_OK) {
                HAL_NVIC_SetPriority (SysTick_IRQn, 1, 0);
        }

        MX_GPIO_Init ();
        MX_USART1_UART_Init ();

        //        while (1){}

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

        /*
         * If hlptim1.Instance->CNT returns 0 for the first time it is read, we would
         * have a false positive. This is why we make sure there are at least 2 reads
         * as the manual tells us.
         */
        int iterations = 0;

        while ((prevCnt != (cnt = (unsigned short)(hlptim1.Instance->CNT))) || iterations < 1) {
                prevCnt = cnt;
                ++iterations;
        }

        return cnt;
}

void enterSleep (TickType_t tick)
{
        (void)tick;
        //         HAL_PWREx_EnterSTOP1Mode (PWR_STOPENTRY_WFI);
        HAL_PWR_EnterSLEEPMode (PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
}

void returnFromSleep (TickType_t tick) { (void)tick; }

// For my initial configuration this variable equals 1 (16kHz / 1kHz).
static const unsigned short TIMER_COUNTS_FOR_ONE_TICK = (configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ);
#define TIMER_COUNTS_FOR_ARR_SET 3

void setArr (unsigned short value)
{
        //        // Should not happen
        //        while ((hlptim1.Instance->ISR & LPTIM_FLAG_ARROK) == 0) {
        //        }

        hlptim1.Instance->ICR |= LPTIM_FLAG_ARROK;

        // Does this clear ARROK?
        hlptim1.Instance->ARR = value;

        // TODO wastes CPU cycles
        // Wait for ARR to settle
        while ((hlptim1.Instance->ISR & LPTIM_FLAG_ARROK) != 0) {
        }
}

/**
 * Taken from the FreeRTOS itself and modified. I left the comments almost intact, so
 * they refer to the SysTick timer instead of LPTIM1 which I use.
 *
 * "Timer counts" - single increments to COUNT register of a counter (16kHz)
 * "Ticks" - OS system time (1kHz)
 */
void vPortSuppressTicksAndSleep (TickType_t xExpectedIdleTime)
{
        TickType_t ulReloadValue;
        TickType_t xModifiableIdleTime;

//         To be sure, taken from Jay Kickliter
//                if (!(hlptim1.Instance->ISR & LPTIM_FLAG_ARROK)) {
//                        return;
//                }

        /*
         * We don't have to "Make sure the SysTick reload value does not overflow the counter"
         * since TickType_t == unsigned short which is the same type as LPTIM1 ARR register
         * which is 16 bit unsigned number.
         *
         * To consider : I could use static_assert if it was C++.
         */
        assert (sizeof (xExpectedIdleTime) == 2);

        // Im pretty sure FreeRTOS would not allow this, but just to be sure.
        if (xExpectedIdleTime < 2) {
                return;
        }

        // We are counting up, so time left is calculated like this:
        TickType_t timerCountsLeftToFullTick = TIMER_COUNTS_FOR_ONE_TICK - getLpTimCounter ();
        assert (timerCountsLeftToFullTick >= 0 && timerCountsLeftToFullTick <= TIMER_COUNTS_FOR_ONE_TICK);

        /*
         * Calculate the value for ARR. required to wait xExpectedIdleTime
         * tick periods.  -1 is used because this code will execute part way
         * through one of the tick periods (and we don't want to declare more
         * ticks than really passed)
         */
        ulReloadValue = timerCountsLeftToFullTick + (TIMER_COUNTS_FOR_ONE_TICK * (xExpectedIdleTime - 1UL)) - 1;

        /*
         * Enter a critical section but don't use the taskENTER_CRITICAL()
         * method as that will mask interrupts that should exit sleep mode.
         */
        __asm volatile("cpsid i" ::: "memory");
        __asm volatile("dsb");
        __asm volatile("isb");

        /*
         * If a context switch is pending or a task is waiting for the scheduler
         * to be unsuspended then abandon the low power entry.
         */
        if (eTaskConfirmSleepModeStatus () == eAbortSleep) {
                __asm volatile("cpsie i" ::: "memory");
                return;
        }

        hlptim1.Instance->CR = 0; // Stop, to be able to restart. Counter is now 0.
        hlptim1.Instance->CR = LPTIM_CR_ENABLE;

        /*
         * CNT is now 0, so we have plenty of time to ARR to propagate. It takes 3 LPTIM_CLK
         * cycles on average to update ARR, but we have 16 LPTIM_CLK cycles ahead of us.
         */
        hlptim1.Instance->ARR = ulReloadValue;
        hlptim1.Instance->CR |= LPTIM_CR_CNTSTRT;

        {
                xModifiableIdleTime = xExpectedIdleTime;
                // Enter the sleep mode using WFI.
                enterSleep (xModifiableIdleTime);
                returnFromSleep (xModifiableIdleTime);
        }

        /* Determine if the SysTick clock has already counted to zero and
                        been set back to the current reload value (the reload back being
                        correct for the entire expected idle time) or if the SysTick is yet
                        to count to zero (in which case an interrupt other than the SysTick
                        must have brought the system out of sleep mode). */

        unsigned short counterRegister = getLpTimCounter ();
        uint32_t isrRegister = hlptim1.Instance->ISR;

        /*
         * Re-enable interrupts to allow the interrupt that brought the MCU
         * out of sleep mode to execute immediately. This will  clear
         * ISR flags as well, if LPTIM1 IRQ is pending. And it will increase
         * the tick by 1 if LPTIM_IRQ woke us up.
         */
        __asm volatile("cpsie i" ::: "memory");
        __asm volatile("dsb");
        __asm volatile("isb");

        /*
         * Disable interrupts again because the clock is about to be stopped
         * and interrupts that execute while the clock is stopped will increase
         * any slippage between the time maintained by the RTOS and calendar
         * time.
         *
         * TODO not true? We don't stop LPTIM as they do with SysTick.
         */
        __asm volatile("cpsid i" ::: "memory");
        __asm volatile("dsb");
        __asm volatile("isb");

        TickType_t ulCompleteTickPeriods = 0;

        // LPTIM counted to the ARR and reset the counter to 0.
        if (isrRegister & LPTIM_FLAG_ARRM) {
                /*
                 * We slept for the whole requested period of fime. -1 is because
                 * we have been woken up by LPTIM IRQ, and we let the ISR to be run,
                 * so it already increased the tick count by 1.
                 */
                ulCompleteTickPeriods = xExpectedIdleTime - 1UL;
        }
        else {
                // LPTIM haven't finished the "single mode" cycle, so the sleep period was shorter.
                ulCompleteTickPeriods = (counterRegister /*+ TIMER_COUNTS_FOR_ARR_SET*/) / TIMER_COUNTS_FOR_ONE_TICK;
                TickType_t ulModuloTicPeriods = (counterRegister /*+ TIMER_COUNTS_FOR_ARR_SET*/) % TIMER_COUNTS_FOR_ONE_TICK;

                if (ulModuloTicPeriods >= TIMER_COUNTS_FOR_ONE_TICK / 2) {
                        ++ulCompleteTickPeriods;
                }
        }

        // Show FreeRTOS how much time has passed while it sleept.
        vTaskStepTick (ulCompleteTickPeriods);

        hlptim1.Instance->CR = 0;                                // This clears the COUNTER
        hlptim1.Instance->CR = LPTIM_CR_ENABLE;                  // Restart the timer with counter == 0
        hlptim1.Instance->ARR = TIMER_COUNTS_FOR_ONE_TICK - 1UL; // And restore ARR to usual "1-tick" period.
        hlptim1.Instance->CR |= LPTIM_CR_CNTSTRT;                // Restart the countinuous mode.

        // Exit with interrupts enabled.
        __asm volatile("cpsie i" ::: "memory");
}

/**
 * This is called by the FreeRTOS during scheduler startup to configure
 * the system tick timer (not to be confused with the SysTick timer).
 */
void vPortSetupTimerInterrupt ()
{
        __HAL_RCC_LPTIM1_CLK_ENABLE ();

        hlptim1.Instance = LPTIM1;
        hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC; // LSI frequency (32kHz throughout the docs) is between 31.04 and 32.96kHz
        hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV2;          // Divide by 2 gives LPTIM_CLK = ~16kHz
        hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
        hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
        hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
        hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
        hlptim1.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
        hlptim1.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;

        if (HAL_LPTIM_Init (&hlptim1) != HAL_OK) {
                Error_Handler ();
        }

        // IER can be modified only if the timer is disabled.
        hlptim1.Instance->CR = 0;
        // Make sure only the autoreload match interrupt is on.
        hlptim1.Instance->IER = 0x2;
        hlptim1.Instance->CR = LPTIM_CR_ENABLE;
        // Set the new reload value.
        setArr (TIMER_COUNTS_FOR_ONE_TICK - 1UL);
        hlptim1.Instance->CR |= LPTIM_CR_CNTSTRT;

        // Not sure what priority it should get. SysTick gets 0 in HAL.
        HAL_NVIC_SetPriority (LPTIM1_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ (LPTIM1_IRQn);
}

/**
 * Handles all the IRQ requests from LPTIM of which only ARRM is turned ON.
 */
void LPTIM1_IRQHandler (void)
{
        uint32_t isrFlags = hlptim1.Instance->ISR;

        // Something other than ARR interrupt, ignore (shouldn't happen).
        if ((isrFlags & LPTIM_FLAG_ARRM) == 0) {
                return;
        }

        // Clear Autoreload match flag
        __HAL_LPTIM_CLEAR_FLAG (&hlptim1, LPTIM_FLAG_ARRM);

        /*
         * The SysTick runs at the lowest interrupt priority, so when this interrupt
         * executes all interrupts must be unmasked.  There is therefore no need to
         * save and then restore the interrupt mask value as its value is already
         * known.
         */
        portDISABLE_INTERRUPTS ();
        {
                // Increment the RTOS tick.
                if (xTaskIncrementTick () != pdFALSE) {

                        /*
                         * A context switch is required.  Context switching is performed in
                         * the PendSV interrupt.  Pend the PendSV interrupt.
                         */
                        portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;
                }
        }
        portENABLE_INTERRUPTS ();
}

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
