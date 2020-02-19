// lptimTick.c -- Jeff Tenney
//
// STM32L No-Drift FreeRTOS Tick via LPTIM
//
// Revision: 2020.02.17
// Tabs: None
// Columns: 127
// Compiler: gcc

#include "FreeRTOS.h"
#include "task.h"
#include "stm32l4xx.h"

//      This FreeRTOS port "extension" for STM32 uses LPTIM1 to generate the OS tick instead of the CM4 systick timer.  The
// benefit of the LPTIM is that it continues running in "stop" mode as long as its clock source does.  A typical clock source
// for the LPTIM is LFE (or LFI), which does keep running in stop mode.
//
//      The resulting FreeRTOS port:
//
//   o Allows use of low-power STOP modes during tickless idle.
//   o Eliminates kernel-time drift associated with tickless idle in official FreeRTOS port (stopping/starting systick).
//   o Eliminates kernel-time drift caused by rounding the OS tick width to the nearest whole number of timer counts.
//   o Avoids drift and errors found in other LPTIM implementations available from ST or the public domain.
//   o Easily adaptable to add ppm steering to correct frequency error in the oscillator (LFE/LFI).
//
//      This software is currently adapted for STM32L4 but is easily adaptable (or already compatible) with other STM32L* MCUs.


// Varying OS Tick Width
//
//      This software varies the number of timer counts per OS tick to achieve the target OS tick frequency.  The OS tick
// generally occurs no more than half a timer count early or half a timer count late.  Occasionally, when a tickless sleep is
// interrupted, the next OS tick *might* come as many as 4 timer counts late, but subsequent timer ticks resume the proper
// (original) schedule.  It's worth noting that the late tick is *not* one the application (or OS) was waiting for.  This
// distinction is important because this software does *not* increase latency in handling OS timer expirations or timeouts
// except by just a handful of CPU instructions.
//
//      With a 32768Hz timer and a 1000Hz OS tick, this software employs 32-count and 33-count tick widths as needed to stay on
// the 1000Hz tick schedule.  With a 4096Hz timer and a 100Hz OS tick, this software uses 40-count and 41-count tick widths to
// stay on schedule.  In each case, the tick is generated on the timer count closest to the ideal tick time.


// Terminology
//
//      "Count" - What a timer does in its "count" register (CNT).
//
//      "Tick" - The OS tick, made up of some number of timer counts.


// Quirks of LPTIM
//
//      The following "quirks" indicate that LPTIM is designed for PWM output control and not for generating timed interrupts:
//
//   o Writes to CMP are delayed by a synchronization mechanism inside the timer.  It takes ~3 timer counts to synchronize.
//   o During the synchronization process, additional writes to CMP are prohibited.
//   o CMPOK (sync completion) comes *after* the CMP value is asynchronously available to the timer for match events.
//     +-- Yes, we've had the ISR detect CMPM and *no* CMPOK for a value recently written to CMP still waiting for CMPOK.
//   o The match condition is not simply "CNT == CMP" but is actually "CNT >= CMP && CNT != ARR".
//   o Once a new CMP value is available to LPTIM, the timer must first be in a "no-match" condition to generate a match event.
//   o Setting CMP == ARR is prohibited.  Sort of.
//   o Changing IER (interrupt-enable register) is prohibited while LPTIM is enabled.
//

#if ( !defined(configUSE_TICKLESS_IDLE) || configUSE_TICKLESS_IDLE != 2 )
#warning Please edit FreeRTOSConfig.h to define configUSE_TICKLESS_IDLE as 2 *or* exclude this file from this build config.
#else

#ifdef xPortSysTickHandler
#error Please edit FreeRTOSConfig.h to eliminate the preprocessor definition for xPortSysTickHandler.
#endif

//      Realistically, the LPTIM clock rate is 32768Hz or a divided version of it.  (Otherwise why would you be using this
// FreeRTOS port extension?)  The LPTIM can optionally divide its input clock by 2, 4, 8, ..., 128.  For simplicity's sake, we
// take responsibility for setting up the LPTIM divider based on configLPTIM_CLOCK_HZ.
//
#ifdef configTICK_USES_LSI
#define LPTIM1SEL_Val 1 // LSI
#else
#define LPTIM1SEL_Val 3 // LSE
#endif

#ifndef configLPTIM_CLOCK_HZ
#define configLPTIM_CLOCK_HZ 32768UL
#endif

#if (configLPTIM_CLOCK_HZ == 32768UL)
#define CFGR_VAL (0 << LPTIM_CFGR_PRESC_Pos)
#elif (configLPTIM_CLOCK_HZ == 32768UL/2)
#define CFGR_VAL (1 << LPTIM_CFGR_PRESC_Pos)
#elif (configLPTIM_CLOCK_HZ == 32768UL/4)
#define CFGR_VAL (2 << LPTIM_CFGR_PRESC_Pos)
#elif (configLPTIM_CLOCK_HZ == 32768UL/8)
#define CFGR_VAL (3 << LPTIM_CFGR_PRESC_Pos)
#elif (configLPTIM_CLOCK_HZ == 32768UL/16)
#define CFGR_VAL (4 << LPTIM_CFGR_PRESC_Pos)
#elif (configLPTIM_CLOCK_HZ == 32768UL/32)
#define CFGR_VAL (5 << LPTIM_CFGR_PRESC_Pos)
#elif (configLPTIM_CLOCK_HZ == 32768UL/64)
#define CFGR_VAL (6 << LPTIM_CFGR_PRESC_Pos)
#elif (configLPTIM_CLOCK_HZ == 32768UL/128)
#define CFGR_VAL (7 << LPTIM_CFGR_PRESC_Pos)
#else
#error Preprocessor symbol configLPTIM_CLOCK_HZ must be 32768 divided by 1, 2, 4, 8, 16, 32, 64, or 128.
#endif

static TickType_t xMaximumSuppressedTicks;   //   We won't try to sleep longer than this many ticks during tickless idle
                                             // because any longer might confuse the logic in our implementation.

static uint32_t ulTimerCountsForOneTick;     //   A "baseline" tick has this many timer counts.  The baseline tick is as close
                                             // as possible to the ideal duration but is a whole number of timer counts.

static int xTimerSubcountErrorPerTick;       //   A "baseline" tick has this much error, measured in timer subcounts.  There
                                             // are configTICK_RATE_HZ subcounts per count.

static volatile uint16_t idealCmp;           //   This field doubles as a write cache for LPTIM->CMP and a way to remember that
                                             // we set CMP to 0 because 0xFFFF isn't allowed.

static volatile int isCmpWriteInProgress;    //   This field helps us remember when we're waiting for the CMP write to finish.

static volatile int runningSubcountError;    //   The error accumulator.  Never exceeds configTICK_RATE_HZ/2 unless ppm
                                             // corrections are in use, then never exceeds configTICK_RATE_HZ.


//=============================================================================================================================
// vPortSetupTimerInterrupt()
//
//      This function overrides the "standard" port function, decorated with __attribute__((weak)), in port.c for ARM CM4F.
// Call with interrupts masked.
//
void vPortSetupTimerInterrupt( void )
{
   //      Enable the APB clock to the LPTIM1.  Then select the 32kHz kernel clock.  Update these statements as needed for your
   // specific STM32.  If you use the ST HAL, you could alternatively call __HAL_RCC_LPTIM1_CLK_ENABLE() and HAL_LPTIM_Init().
   //
   RCC->APB1ENR1 |= RCC_APB1ENR1_LPTIM1EN;
   MODIFY_REG(RCC->CCIPR, RCC_CCIPR_LPTIM1SEL, LPTIM1SEL_Val << RCC_CCIPR_LPTIM1SEL_Pos);

   //      Calculate the constants required to configure the tick interrupt.
   //
   ulTimerCountsForOneTick = ( configLPTIM_CLOCK_HZ / configTICK_RATE_HZ );
   configASSERT(ulTimerCountsForOneTick >= 4UL);  // CLOCK must be at least 4x TICK
   xTimerSubcountErrorPerTick = configLPTIM_CLOCK_HZ % configTICK_RATE_HZ;
   if (xTimerSubcountErrorPerTick > configTICK_RATE_HZ/2)
   {
      ulTimerCountsForOneTick++;
      xTimerSubcountErrorPerTick -= configTICK_RATE_HZ;
   }

   //      Give 1 OS tick of margin between clearly future match events and clearly past match events.  Anything within the
   // previous one tick is clearly past, within one tick before that is in the margin between, which we call the past for
   // convenience, and everything is in the future.
   //
   xMaximumSuppressedTicks = 65535UL * configTICK_RATE_HZ / configLPTIM_CLOCK_HZ - 1 - 1;

   //      Configure and start LPTIM.
   //
   //      Erratum 2.14.1 says that clearing LPTIM_CR_ENABLE can prevent STOP mode.
   //
   // LPTIM1->CR = 0;
   //
   RCC->APB1RSTR1 |= RCC_APB1RSTR1_LPTIM1RST;   // Reset the LPTIM1 module per erratum 2.14.1.
   RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_LPTIM1RST;

   LPTIM1->IER = LPTIM_IER_CMPMIE | LPTIM_IER_CMPOKIE;   // Modify this register only when LPTIM is disabled (ENABLE = 0).
   LPTIM1->CFGR = CFGR_VAL;                              // Modify this register only when LPTIM is disabled (ENABLE = 0).
   LPTIM1->CR = LPTIM_CR_ENABLE;
   LPTIM1->ARR = 0xFFFF;  // timer period = ARR + 1 ticks.  Modify this register only when LPTIM is enabled  (ENABLE = 1).
   LPTIM1->CMP = ulTimerCountsForOneTick;                // Modify this register only when LPTIM is enabled  (ENABLE = 1).
   isCmpWriteInProgress = pdTRUE;
   idealCmp = ulTimerCountsForOneTick;
   runningSubcountError = xTimerSubcountErrorPerTick;
   LPTIM1->CR |= LPTIM_CR_CNTSTRT;

   //      Despite a comment in xPortSysTickHandler(), the system tick interrupt does *not* need to be the lowest priority
   // interrupt.  That handler may claim to enable interrupts, but the NVIC won't enable interrupts more than they were already
   // enabled upon ISR entry.  So use configKERNEL_INTERRUPT_PRIORITY, not simply configLIBRARY_LOWEST_INTERRUPT_PRIORITY.
   // Note that by convention, configKERNEL_INTERRUPT_PRIORITY is pre-shifted for code in port.c, which writes directly to
   // SCB->SHPRx.  So we "unshift" it since NVIC_SetPriority() expects the unshifted value.
   //
   NVIC_SetPriority(LPTIM1_IRQn, configKERNEL_INTERRUPT_PRIORITY >> (8 - configPRIO_BITS));
   NVIC_EnableIRQ(LPTIM1_IRQn);
}


//=============================================================================================================================
// vPortSuppressTicksAndSleep()
//
//      This function overrides the "official" port function, decorated with __attribute__((weak)), in port.c for ARM CM4F.
// The OS calls this function as needed.
//
void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime )
{
   //      The idle task calls this function with the scheduler suspended, and only when xExpectedIdleTime is >= 2.  Any tick
   // that occurs while the scheduler is suspended gets "pended", not immediately reflected in <xTickCount>.

   //      Determine the number of "extra" timer counts to add to the compare register, which is currently set for the next
   // tick.  Because the next tick is less than one tick away, we know we won't set the compare register more than
   // <xMaximumSuppressedTicks> (in timer counts) from the *current* CNT value.
   //
   if (xExpectedIdleTime > xMaximumSuppressedTicks) xExpectedIdleTime = xMaximumSuppressedTicks;
   uint32_t extraCounts = (xExpectedIdleTime - 1UL) * configLPTIM_CLOCK_HZ / configTICK_RATE_HZ;
   int extraError = (xExpectedIdleTime - 1UL) * configLPTIM_CLOCK_HZ % configTICK_RATE_HZ;
   if (extraError > (int)(configTICK_RATE_HZ/2))
   {
      extraCounts++;
      extraError -= configTICK_RATE_HZ;
   }

   //      Enter a critical section so we can safely check the sleep-mode status.  But don't use taskENTER_CRITICAL() because
   // that function masks interrupts that we need to exit sleep mode.  We must stay in the critical section until we go to
   // sleep so that any interrupt starting now wakes us up from sleep.
   //
   __asm volatile( "cpsid i" ::: "memory" );
   __asm volatile( "dsb" );
   __asm volatile( "isb" );

   //      If a context switch is pending or a task is waiting for the scheduler to be unsuspended, then abandon the low power
   // entry and the critical section.  This status cannot change while interrupts are masked.
   //
   if (eTaskConfirmSleepModeStatus() == eAbortSleep) __asm volatile( "cpsie i" ::: "memory" );
   else
   {
      //      Adjust extra counts if needed to maintain proper alignment.  Unlike in the tick ISR, we need both checks here
      // since <extraError> could be "anything" here, not just the error associated with a single baseline tick.
      //
      if (runningSubcountError + extraError > (int)(configTICK_RATE_HZ/2))
      {
         extraCounts++;
         extraError -= configTICK_RATE_HZ;
      }
      else if (runningSubcountError + extraError < -(int)(configTICK_RATE_HZ/2))
      {
         extraCounts--;
         extraError += configTICK_RATE_HZ;
      }

      //      Add the extra counts to the upcoming timer interrupt.  If we can't write to the CMP register right now, the ISR
      // for CMPOK will do it for us.
      //
      idealCmp += extraCounts;  // (idealCmp is a uint16_t)
      if (!isCmpWriteInProgress)
      {
         isCmpWriteInProgress = pdTRUE;
         LPTIM1->CMP = idealCmp == 0xFFFF ? 0 : idealCmp;  // never write 0xFFFF to CMP
      }
      uint32_t expectedEndCmp = idealCmp;

      //      Because our implementation uses an interrupt handler to process a successful write to CMP, we use a loop here so
      // we won't return to our caller merely for that interrupt.  As a bonus, we won't return to our caller for any other ISR
      // that doesn't request a context switch, either.
      //
      //      Stay in the loop until an ISR requests a context switch or until the timer reaches the end of the sleep period.
      // We identify the end of the sleep period by recognizing that the tick ISR has modified <idealCmp> for the next tick
      // after the sleep period ends.
      //
      do
      {
         //      Give the application a chance to arrange for the deepest sleep it can tolerate right now.  Also give it an
         // opportunity to provide its own WFI or WFE instruction.  (In that case, it sets xModifiableIdleTime = 0.)
         //
         TickType_t xModifiableIdleTime = xExpectedIdleTime;
         configPRE_SLEEP_PROCESSING( xModifiableIdleTime );
         if (xModifiableIdleTime > 0)
         {
            //      Wait for an interrupt.
            //
            __asm volatile( "dsb" ::: "memory" );
            __asm volatile( "wfi" );
            __asm volatile( "isb" );
         }
         configPOST_SLEEP_PROCESSING( (const TickType_t)xExpectedIdleTime );

         //      Re-enable interrupts, and then execute the ISR tied to the interrupt that brought the MCU out of sleep mode.
         //
         __asm volatile( "cpsie i" ::: "memory" );
         __asm volatile( "dsb" );
         __asm volatile( "isb" );

         //      Disable interrupts for our call to eTaskConfirmSleepModeStatus() and in case we iterate again in the loop.
         //
         __asm volatile( "cpsid i" ::: "memory" );
         __asm volatile( "dsb" );
         __asm volatile( "isb" );

      } while (eTaskConfirmSleepModeStatus() != eAbortSleep && idealCmp == expectedEndCmp);

      //      Re-enable interrupts.  We try our best to support short ISR latency.
      //
      __asm volatile( "cpsie i" ::: "memory" );

      //      Determine how many tick periods elapsed during our sleep.  And if something other than the tick timer woke us up,
      // reconfigure the tick timer back to normal operation at configTICK_RATE_HZ.
      //
      //      Begin by assuming we managed to stay asleep the entire time.  In that case, the tick ISR already added one tick
      // (well, actually the ISR "pended" the increment because the scheduler is currently suspended, but it's all the same to
      // us), so we use "- 1" here.
      //
      uint32_t ulCompleteTickPeriods = xExpectedIdleTime - 1UL;

      //      We identify that we reached the end of the expected idle time by noting that the tick ISR has modified
      // <idealCmp>.  So if it hasn't, then we probably have to reconfigure the timer as described above.  We temporarily
      // disable the tick interrupt while we make the assessment and manipulate <idealCmp> (and CMP) if necessary.
      //
      NVIC_DisableIRQ(LPTIM1_IRQn);
      if (idealCmp == expectedEndCmp)
      {
         //      Something else woke us up.  See how many timer counts we still had left, and then use that number to determine
         // how many OS ticks actually elapsed.  Then set up the timer for normal operation, in phase where it would have been.
         //
         uint32_t currCount;
         do currCount = LPTIM1->CNT; while (currCount != LPTIM1->CNT);

         //      Since we've blocked only the tick IRQ here, it's possible that ISRs have just now allowed the timer to *pass*
         // <expectedEndCmp>, so watch out for that (rare) situation here.  That would make <countsLeft> appear larger than our
         // longest possible tickless sleep.
         //
         uint32_t countsLeft = (uint16_t)(idealCmp - currCount);
         if (countsLeft <= xMaximumSuppressedTicks * configLPTIM_CLOCK_HZ / configTICK_RATE_HZ)
         {
            uint32_t fullTicksLeft = countsLeft * configTICK_RATE_HZ / configLPTIM_CLOCK_HZ;
            configASSERT(fullTicksLeft < xExpectedIdleTime);

            //      If the time left amounts to at least one full tick, then reschedule the first tick interrupt that we
            // haven't yet skipped.  And update <ulCompleteTickPeriods> not to count the ones we haven't skipped.
            //
            if (fullTicksLeft != 0)
            {
               ulCompleteTickPeriods -= fullTicksLeft;

               int completePeriodsCounts = ulCompleteTickPeriods * configLPTIM_CLOCK_HZ / configTICK_RATE_HZ;
               extraError = ulCompleteTickPeriods * configLPTIM_CLOCK_HZ % configTICK_RATE_HZ;
               if (runningSubcountError + extraError > (int)(configTICK_RATE_HZ/2))
               {
                  completePeriodsCounts++;
                  extraError -= configTICK_RATE_HZ;
               }

               idealCmp -= extraCounts - completePeriodsCounts; // idealCmp is a uint16_t
               if (!isCmpWriteInProgress)
               {
                  isCmpWriteInProgress = pdTRUE;
                  LPTIM1->CMP = idealCmp == 0xFFFF ? 0 : idealCmp;  // never write 0xFFFF to CMP
               }
            }
         }
      }

      runningSubcountError += extraError;

      NVIC_EnableIRQ(LPTIM1_IRQn);

      //      Increment the tick count by the number of (full) "extra" ticks we waited.  Function vTaskStepTick() asserts that
      // this function waited the proper number of ticks, and any ticks we may have hit during this function are pended and not
      // yet reflected in <xTickCount>, so that assertion is accurate and effective.  We don't have to worry about modifying
      // <xTickCount> count here while the tick count ISR is enabled because the scheduler is currently suspended.  That causes
      // the tick ISR to accumulate ticks into a pended-ticks field.
      //
      vTaskStepTick( ulCompleteTickPeriods );
   }
}


//=============================================================================================================================
// LPTIM1_IRQHandler()
//
void LPTIM1_IRQHandler( void )
{
   //      Grab a copy of the ISR register so that any new interrupts (after this reading of ISR) are processed in the next
   // iteration of this function.  This "best practice" gives us the best synchronization in our processing because we grab a
   // coherent "instantaneous" status of the timer.  (That doesn't seem to really matter with LPTIM.)  It also allows us to
   // simulate an interrupt and respond as if it had actually occurred.
   //
   uint32_t isr = LPTIM1->ISR;

   //      Handle the CMPOK interrupt first in case we missed a match event due to the synchronization mechanism inside LPTIM.
   // In that case, we "induce" that interrupt manually before the code further below that processes it.
   //
   if (isr & LPTIM_ISR_CMPOK)
   {
      //      Acknowledge and clear the CMPOK event.
      //
      LPTIM1->ICR = LPTIM_ICR_CMPOKCF;

      //      If there is a "pending" write operation to CMP, do it now.  Otherwise, make note that the write is now complete.
      // Remember to watch for CMP set to 0 when idealCmp is 0xFFFF.  There's no pending write in that case.
      //
      if ((uint16_t)(LPTIM1->CMP - idealCmp) > 1UL)
      {
         LPTIM1->CMP = idealCmp == 0xFFFF ? 0 : idealCmp;  // never write 0xFFFF to CMP
         // isCmpWriteInProgress = pdTRUE;  // already true here in the handler for write completed
      }
      else
      {
         isCmpWriteInProgress = pdFALSE;

         //      Be sure we didn't lose an interrupt due to the synchronization mechanism for CMP.  There are two ways we can
         // lose an interrupt.  First, with very high CMP values (near 0xFFFF), if the counter reaches 0xFFFF before the sync
         // mechanism finishes, the timer won't identify the match.  And second, with *any* CMP value that becomes available to
         // the timer a little too late, the timer won't identify a *new* match if the previous CMP value already matched and
         // was lower than the new CMP value.  Either one of these conditions might occur in a single execution of function
         // vPortSuppressTicksAndSleep().  When that function stops a sleep operation early due to an application interrupt, it
         // tries to revert to wherever the next scheduled tick should be.  In so doing, it may write a CMP value that is
         // imminent.  That CMP value may be "very high", or it may be numerically larger than the previous compare value, as
         // it would be for a reversion that "unwraps" from a CMP value in the next counter epoch back through 0xFFFF to a CMP
         // value in the current epoch.
         //
         //      We could look for these conditions right now and then conditionally simulate the missed match interrupt, but
         // the code that processes match events (below) already qualifies matches by exactly the criteria we would use to look
         // for missed matches.  So simulate a match event unconditionally here, knowing the code below will disqualify it most
         // of the time.  The wasted effort is tiny, and easily worthwhile as a means to keep our code and design clean.
         //
         isr |= LPTIM_ISR_CMPM;
      }
   }

   if (isr & LPTIM_ISR_CMPM)
   {
      //      Acknowledge and clear the CMPM event.  Based on the errata, we dare not clear this flag unless it is already set.
      // Call it an over-abundance of caution.
      //
      if (LPTIM1->ISR & LPTIM_ISR_CMPM) LPTIM1->ICR = LPTIM_ICR_CMPMCF;

      //      Get a coherent copy of the current count value in the timer.  The CNT register is clocked asynchronously, so we
      // keep reading it until we get the same value during a verification read.
      //
      uint32_t countValue;
      do countValue = LPTIM1->CNT; while (countValue != LPTIM1->CNT);

      //      If the ideal CMP value is in the recent past -- within one OS tick time -- assume it's valid.  This logic must
      // coordinate with our definition of <xMaximumSuppressedTicks>.  Note that if we arrive here too late, we'll lose the OS
      // tick for up to an entire timer epoch.  That would be the application's fault for masking interrupts too long or for
      // having too many other interrupt handlers before servicing the LPTIM interrupt.
      //
      //      This logic handles several important cases.  First, it temporarily ignores the unfortunate match condition that
      // occurs prematurely when vPortSuppressTicksAndSleep() wraps CMP into the next timer epoch.  Second, it helps us ignore
      // an imminent tick that vPortSuppressTicksAndSleep() is trying to suppress but may occur anyway due to the CMP sync
      // mechanism.  Third, it helps us honor an interrupt that vPortSuppressTicksAndSleep() has restored even if it happens a
      // little bit late due to the sync mechanism, and even if that interrupt occurs before its corresponding CMPOK event
      // occurs (happens occasionally).  Finally, it handles the CMP-write-complete case, triggered above, where we now check
      // for a tick missed due to the sync mechanism.
      //
      uint32_t ticksLate = (uint16_t)(countValue - idealCmp);
      if (ticksLate < ulTimerCountsForOneTick)
      {
         //      We officially have an OS tick.  Count it, and set up the next one.

         uint32_t numCounts = ulTimerCountsForOneTick;
         runningSubcountError += xTimerSubcountErrorPerTick;
#        if (configLPTIM_CLOCK_HZ % configTICK_RATE_HZ <= configTICK_RATE_HZ/2)
         if (runningSubcountError > (int)(configTICK_RATE_HZ/2))
         {
            numCounts++;
            runningSubcountError -= configTICK_RATE_HZ;
         }
#        else
         if (runningSubcountError < -(int)(configTICK_RATE_HZ/2))
         {
            numCounts--;
            runningSubcountError += configTICK_RATE_HZ;
         }
#        endif

         //      Set up the next tick interrupt.
         //
         idealCmp += numCounts;  // idealCmp is a uint16_t
         if (!isCmpWriteInProgress)  // A write in progress here? Rare but possible; remember, CMPM can come before CMPOK.
         {
            LPTIM1->CMP = idealCmp == 0xFFFF ? 0 : idealCmp;  // never write 0xFFFF to CMP
            isCmpWriteInProgress = pdTRUE;
         }

         //      Tell the OS about the tick.
         //
         extern void xPortSysTickHandler( void );
         xPortSysTickHandler();
      }
   }
}


#if 0
//=============================================================================================================================
// vPortSetRefClockFreqError()
//
// PPM Correction (** UNFINISHED CODE **)
//
//      To correct for frequency error in the reference clock, we could periodically add/subtract half a count to
// <runningSubcountError>.  We leave this unfinished code here as an indication of just how easy it would be.
//
//      Typical 32kHz "tuning-fork" crystals operate in a 150ppm window over temperature, with the fastest frequency around
// 25 degrees C.  A "poor" frequency accuracy at 25 degrees would be +/-100ppm, so our ppm tuning must tolerate -250ppm to
// +100ppm.  At -250ppm, we have half a count correction every 2000 counts.  That means there isn't much error in rounding
// <ulTimerCountsPerSteeringEvent> and not keeping track of subcounts.  Even at -500ppm, it's no worth worrying over; instead
// of correcting for -500ppm, we might correct for -500.1 ppm.  No big deal.
//
//      An application using this feature could update the ppm periodically, presumably in response to temperature changes.
// Since the RTC on modern STM32 MCUs allows ppm steering, most applications would have no need for this feature.

static uint32_t ulTimerCountsPerSteeringEvent;
static int      isPpmNegative;
void vPortSetRefClockFreqError(int ppm)  // designed for -250...+100; functions up to +/- 500000 with reduced accuracy.
{
   if (ppm == 0)
   {
      //      Instruct the ppm-steering system to stand down.
      //
      ulTimerCountsPerSteeringEvent = 0;
   }
   else
   {
      //      Help the ppm-steering system decide whether each steering event should add or subtract half a clock count.
      //
      isPpmNegative = (ppm < 0);

      //      If a steering event adds/subtracts one timer count, then we need |ppm| steering events every 1M+ppm counts.  But
      // we want our steering events to correct by half a timer count so we don't ever accidentally induce a correction of two
      // counts when combined with the standard tick-width modulating.  So we need twice as many steering events since each
      // event corrects only half a tick.
      //
      int countsToTreatAs1Million = 1000000L + ppm;
      if (isPpmNegative) ppm = -ppm;
      ulTimerCountsPerSteeringEvent = (countsToTreatAs1Million + ppm) / (2*ppm);  // round the number of counts
   }
}
#endif // if 0

#endif  // configUSE_TICKLESS_IDLE == 2
