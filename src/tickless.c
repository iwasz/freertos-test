#include "FreeRTOS.h"
#include "portmacro.h"
#include "task.h"
#include <assert.h>
#include <stdint.h>
#include <stm32l4xx_hal.h>

#if configUSE_TICKLESS_IDLE == 2

static LPTIM_HandleTypeDef hlptim1; // For system tick

// For my initial configuration this variable equals 1 (16kHz / 1kHz).
static const unsigned short TIMER_COUNTS_FOR_ONE_TICK = (configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ);

/// Compensation when we were woken up by a spurious iterrupt.
static TickType_t tickPeriodsCorrection = 0;

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

/**
 * Actual sleep mode.
 */
void enterSleep (TickType_t tick)
{
        (void)tick;
        // HAL_PWREx_EnterSTOP1Mode (PWR_STOPENTRY_WFI);
        HAL_PWR_EnterSLEEPMode (PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
}

/**
 * @brief returnFromSleep
 * @param tick
 */
void returnFromSleep (TickType_t tick) { (void)tick; }

/**
 * Taken from the FreeRTOS itself and modified. I left the comments almost intact, so
 * they refer to the SysTick timer instead of LPTIM1 which I use.
 *
 * "Timer counts" - single increments to COUNT register of a counter (16kHz)
 * "Ticks" - OS system time (1kHz)
 */
void vPortSuppressTicksAndSleep (TickType_t xExpectedIdleTime)
{
        /*
         * We don't have to "Make sure the SysTick reload value does not overflow the counter"
         * since TickType_t == unsigned short which is the same type as LPTIM1 ARR register
         * which is 16 bit unsigned number.
         */
        static_assert (sizeof (xExpectedIdleTime) == 2, "");

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
        uint32_t ulReloadValue = timerCountsLeftToFullTick + (TIMER_COUNTS_FOR_ONE_TICK * (xExpectedIdleTime - 1UL)) - 1;

        if (ulReloadValue > UINT16_MAX) {
                ulReloadValue = UINT16_MAX;
        }

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
                TickType_t xModifiableIdleTime = xExpectedIdleTime;
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
                ulCompleteTickPeriods = counterRegister / TIMER_COUNTS_FOR_ONE_TICK;
                TickType_t ulModuloTicPeriods = counterRegister % TIMER_COUNTS_FOR_ONE_TICK;

                // Aggregated correction.
                tickPeriodsCorrection += ulModuloTicPeriods;

                /*
                 * The error (ie. the modulo division) aggregates over time, so here we compensate for this.
                 * We simply store the error over time and if it rises over a treshold we add one tick. But
                 * I don't understand why the best treshold is TIMER_COUNTS_FOR_ONE_TICK / 2 instead of the
                 * whole TIMER_COUNTS_FOR_ONE_TICK.
                 *
                 * This condition is exactly like the original correction, but the error is aggregated.
                 */
                if (tickPeriodsCorrection >= TIMER_COUNTS_FOR_ONE_TICK / 2) {
                        tickPeriodsCorrection -= TIMER_COUNTS_FOR_ONE_TICK / 2;
                        ++ulCompleteTickPeriods;
                }

                // This was the first implementation and errors summed up over time
                // if (ulModuloTicPeriods >= TIMER_COUNTS_FOR_ONE_TICK / 2) {
                //         ++ulCompleteTickPeriods;
                // }
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

void setArr (unsigned short value)
{
        hlptim1.Instance->ICR |= LPTIM_FLAG_ARROK;

        // Does this clear ARROK?
        hlptim1.Instance->ARR = value;

        // TODO wastes CPU cycles
        // Wait for ARR to settle
        while ((hlptim1.Instance->ISR & LPTIM_FLAG_ARROK) != 0) {
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

#endif
