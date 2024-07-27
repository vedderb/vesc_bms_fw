/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio.
              Copyright (C) 2019 Diego Ismirlian, (dismirlian(at)google's mail)

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at
        http://www.apache.org/licenses/LICENSE-2.0
    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/*
 *  To use:
 *
 *  1)
 *  #include this file at the bottom of chconf.h. You may need to
 *	redefine some of the hooks in chconf.h, for example
 *
 *	    CH_CFG_THREAD_INIT_HOOK => _CH_CFG_THREAD_INIT_HOOK.
 *
 *  If you don't use those hooks in your original code, you may just delete
 *  them from chconf.h
 *
 *
 *  2)
 *  Copy the SEGGER_RTT_Conf.h and SEGGER_SYSVIEW_Conf.h files from the
 *  segger_bindings/example_configurations/ directory to the project's
 *  cfg directory.
 *
 *  You can tune the config files to suit your needs; see the SEGGER RTT and
 *  SystemView documentation for details.
 *
 *
 *  3)
 *  Add the following call to main():
 *    SYSVIEW_ChibiOS_Start(STM32_SYSCLK, STM32_SYSCLK, "I#15=SysTick");
 *
 *  The first parameter, SysFreq, is the time base for all the timestamps. It
 *  must match SEGGER_SYSVIEW_GET_TIMESTAMP in SEGGER_SYSVIEW_Conf.h. By
 *  default, SEGGER_SYSVIEW_GET_TIMESTAMP is configured to use the DWT cycle
 *  counter, so this parameter should match the CPU frequency (eg.
 *  STM32_SYSCLK).
 *
 *  The second parameter, CPUFreq, appears to be just for information.
 *
 *  The third parameter can be used to name the interrupts in the system.
 *  For example, on the Cortex-M*, when using the classic periodic tick for
 *  ChibiOS (CH_CFG_ST_TIMEDELTA == 0), this parameter should include
 *  "I#15=OSTick" (interrupt #15 is the SysTick). When using the tick-less
 *  mode, this parameter could be tuned to show the ISR name of the timer
 *  module used as the OS timer.
 *
 *  Also, you can include all other interrupts in this configuration string
 *  (eg. "I#15=OSTick,I#54=USART2").
 *
 *  See the SystemView documentation for more details.
 *
 *
 *  4)
 *  Copy the file SYSVIEW_ChibiOS.txt (in the segger_bindings directory) to
 *  the following directory:
 *
 *    Path\to\SystemView\Description\
 *
 *  This will allow SystemView to map the ChibiOS's task state values to names.
 *
 */

#ifndef SYSVIEW_CHIBIOS_H
#define SYSVIEW_CHIBIOS_H

#include "SEGGER_SYSVIEW.h"
void SYSVIEW_ChibiOS_SendTaskInfo(const void *_tp);
void SYSVIEW_ChibiOS_Start(U32 SysFreq, U32 CPUFreq, const char *isr_description);

/********************************************************************/
/*      Checks                                                      */
/********************************************************************/
#if !(CH_CFG_USE_REGISTRY == TRUE)
#error "SYSVIEW integration requires CH_CFG_USE_REGISTRY"
#endif

#if defined(CH_CFG_THREAD_INIT_HOOK)
#error "SYSVIEW integration: rename CH_CFG_THREAD_INIT_HOOK to _CH_CFG_THREAD_INIT_HOOK"
#endif

#if defined(CH_CFG_THREAD_READY_HOOK)
#error "SYSVIEW integration: rename CH_CFG_THREAD_READY_HOOK to _CH_CFG_THREAD_READY_HOOK"
#endif

#if defined(CH_CFG_CONTEXT_SWITCH_HOOK)
#error "SYSVIEW integration: rename CH_CFG_CONTEXT_SWITCH_HOOK to _CH_CFG_CONTEXT_SWITCH_HOOK"
#endif

#if defined(CH_CFG_THREAD_EXIT_HOOK)
#error "SYSVIEW integration: rename CH_CFG_THREAD_EXIT_HOOK to _CH_CFG_THREAD_EXIT_HOOK"
#endif

#if defined(CH_CFG_IRQ_PROLOGUE_HOOK)
#error "SYSVIEW integration: rename CH_CFG_IRQ_PROLOGUE_HOOK to _CH_CFG_IRQ_PROLOGUE_HOOK"
#endif

#if defined(CH_CFG_IRQ_EPILOGUE_HOOK)
#error "SYSVIEW integration: rename CH_CFG_IRQ_EPILOGUE_HOOK to _CH_CFG_IRQ_EPILOGUE_HOOK"
#endif

#if defined(CH_CFG_SYSTEM_HALT_HOOK)
#error "SYSVIEW integration: rename CH_CFG_SYSTEM_HALT_HOOK to _CH_CFG_SYSTEM_HALT_HOOK"
#endif

#if !defined(_CH_CFG_THREAD_INIT_HOOK)
#define _CH_CFG_THREAD_INIT_HOOK(tp) do {} while(0)
#endif

#if !defined(_CH_CFG_THREAD_READY_HOOK)
#define _CH_CFG_THREAD_READY_HOOK(tp) do {} while(0)
#endif

#if !defined(_CH_CFG_CONTEXT_SWITCH_HOOK)
#define _CH_CFG_CONTEXT_SWITCH_HOOK(ntp, otp) do {} while(0)
#endif

#if !defined(_CH_CFG_THREAD_EXIT_HOOK)
#define _CH_CFG_THREAD_EXIT_HOOK(tp) do {} while(0)
#endif

#if !defined(_CH_CFG_IRQ_PROLOGUE_HOOK)
#define _CH_CFG_IRQ_PROLOGUE_HOOK() do {} while(0)
#endif

#if !defined(_CH_CFG_IRQ_EPILOGUE_HOOK)
#define _CH_CFG_IRQ_EPILOGUE_HOOK() do {} while(0)
#endif

#if !defined(_CH_CFG_SYSTEM_HALT_HOOK)
#define _CH_CFG_SYSTEM_HALT_HOOK(reason) do {} while(0)
#endif

/* CH_CFG_THREAD_INIT_HOOK:
 *
 * We report the thread creation and we immediately send the TaskInfo
 * structure, so that SystemView can show it as early as possible.
 */
#define CH_CFG_THREAD_INIT_HOOK(tp) {                     \
  _CH_CFG_THREAD_INIT_HOOK(tp);                           \
  SEGGER_SYSVIEW_OnTaskCreate((U32)tp);                   \
  SYSVIEW_ChibiOS_SendTaskInfo((const void *)tp);         \
}

/* CH_CFG_THREAD_READY_HOOK:
 *
 * This is an *extra* hook, not present in the "stock" ChibiOS code. It is
 * important if you want SystemView to show all the ready threads, even if
 * they are not executing.
 *
 * The hook should be placed just before the return lines of the chSchReadyI
 * and the chSchReadyAheadI functions, in chschd.c:
 *
 * thread_t *chSchReadyAheadI(thread_t *tp) {
 *   ...
 *   CH_CFG_THREAD_READY_HOOK(tp);
 *   return tp;
 * }
 *
 * thread_t *chSchReadyI(thread_t *tp) {
 *   ...
 *   CH_CFG_THREAD_READY_HOOK(tp);
 *   return tp;
 * }
 */
#define CH_CFG_THREAD_READY_HOOK(tp) {                    \
  _CH_CFG_THREAD_READY_HOOK(tp);                          \
  SEGGER_SYSVIEW_OnTaskStartReady((U32)tp);               \
}

/* CH_CFG_CONTEXT_SWITCH_HOOK:
 *
 * This hook is called when switching context from Thread to Thread, or by the
 * tail ISR exit sequence (see comments at CH_CFG_IRQ_EPILOGUE_HOOK).
 *
 * First, we report the switching-out of the "old" thread (otp), and then the
 * switching-in of the "new" thread. Unfortunately, SystemView treats the idle
 * thread as a special case, so we need to do some ugly handling here.
 */
#define CH_CFG_CONTEXT_SWITCH_HOOK(ntp, otp) {            \
  if (otp->hdr.pqueue.prio != IDLEPRIO) {                            \
    SEGGER_SYSVIEW_OnTaskStopReady((U32)otp, otp->state); \
  }                                                       \
  if (ntp->hdr.pqueue.prio == IDLEPRIO) {                            \
    SEGGER_SYSVIEW_OnIdle();                              \
  } else {                                                \
    SEGGER_SYSVIEW_OnTaskStartExec((U32)ntp);             \
  }                                                       \
  _CH_CFG_CONTEXT_SWITCH_HOOK(ntp, otp);                  \
}

#define CH_CFG_THREAD_EXIT_HOOK(tp) {                     \
  _CH_CFG_THREAD_EXIT_HOOK(tp);                           \
  SEGGER_SYSVIEW_OnTaskStopExec();                        \
}

/* CH_CFG_IRQ_PROLOGUE_HOOK:
 *
 * For the ARM Cortex-M* architectures, the PORT_IRQ_PROLOGUE doesn't contain
 * any code, so the timestamp shown by SystemView for the ISR entry is quite
 * accurate.
 */
#define CH_CFG_IRQ_PROLOGUE_HOOK() {                      \
  _dbg_check_enter_isr();                                 \
  SEGGER_SYSVIEW_RecordEnterISR();                        \
  _CH_CFG_IRQ_PROLOGUE_HOOK();                            \
}

/* CH_CFG_IRQ_EPILOGUE_HOOK:
 *
 * When the ISR is at the tail, and preemption is required, we tell SystemView
 * that we exit the ISR to the scheduler first so that the code between
 * CH_CFG_IRQ_EPILOGUE_HOOK and the actual context switch will be shown as
 * "scheduler". Otherwise, that time will be shown as belonging to the thread
 * that was running before the first ISR. If the ISR is not at the tail, we
 * simply tell SystemView that the ISR has been exited. If the ISR is at the
 * tail but preemption is not required, we tell Systemview that we exit the ISR
 * so that it shows that the last thread resumes execution.
 *
 * When the ISR is at the tail, and preemption is required, this hook will
 * be immediately followed by CH_CFG_CONTEXT_SWITCH_HOOK (see
 * _port_switch_from_isr()).
 *
 * Actually, this hook runs a bit early in the ISR exit sequence, so the
 * scheduler time shown by SystemView will be underestimated. The ideal place
 * to place these calls would be at _port_irq_epilogue.
 *
 * Note: Unfortunately, this hook is specific to the Cortex-M architecture
 * until ChibiOS gets a generic "_isr_is_tail()" macro/function.
 */
#if defined(__GNUC__)
#  if (defined(__ARM_ARCH_6M__) || defined(__ARM_ARCH_8M_BASE__))
#    define _isr_is_tail()    (_saved_lr != (regarm_t)0xFFFFFFF1U)
#  elif (defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__) || defined(__ARM_ARCH_8M_MAIN__))
#    define _isr_is_tail()    ((SCB->ICSR & SCB_ICSR_RETTOBASE_Msk) != 0U)
#  else
#    error "SYSVIEW integration: unsupported architecture"
#  endif
#elif defined(__ICCARM__)
#  if (defined (__ARM6M__) && (__CORE__ == __ARM6M__))
#    define _isr_is_tail()    (_saved_lr != (regarm_t)0xFFFFFFF1U)
#  elif ((defined (__ARM7EM__) && (__CORE__ == __ARM7EM__)) || (defined (__ARM7M__) && (__CORE__ == __ARM7M__)))
#    define _isr_is_tail()    ((SCB->ICSR & SCB_ICSR_RETTOBASE_Msk) != 0U)
#  else
#    error "SYSVIEW integration: unsupported architecture"
#  endif
#elif defined(__CC_ARM)
#  if (defined __TARGET_ARCH_6S_M)
#    define _isr_is_tail()    (_saved_lr != (regarm_t)0xFFFFFFF1U)
#  elif (defined(__TARGET_ARCH_7_M) || defined(__TARGET_ARCH_7E_M))
#    define _isr_is_tail()    ((SCB->ICSR & SCB_ICSR_RETTOBASE_Msk) != 0U)
#  else
#    error "SYSVIEW integration: unsupported architecture"
#  endif
#else
#  error "SYSVIEW integration: unsupported compiler"
#endif

#define CH_CFG_IRQ_EPILOGUE_HOOK() {                      \
  _dbg_check_leave_isr();                                 \
  _CH_CFG_IRQ_EPILOGUE_HOOK();                            \
  port_lock_from_isr();                                   \
  _dbg_enter_lock();                                      \
  if (_isr_is_tail() && chSchIsPreemptionRequired()) {    \
    SEGGER_SYSVIEW_RecordExitISRToScheduler();            \
  } else {                                                \
    SEGGER_SYSVIEW_RecordExitISR();                       \
  }                                                       \
  _dbg_leave_lock();                                      \
}

#define CH_CFG_SYSTEM_HALT_HOOK(reason) {                 \
  _CH_CFG_SYSTEM_HALT_HOOK(reason);                       \
  SEGGER_SYSVIEW_Error(reason);                           \
}

#endif
