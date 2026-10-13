/* USER CODE BEGIN Header */
/*
 * FreeRTOS Kernel V10.3.1
 * Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */
/* USER CODE END Header */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * These parameters and more are described within the 'configuration' section of the
 * FreeRTOS API documentation available on the FreeRTOS.org web site.
 *
 * See http://www.freertos.org/a00110.html
 *----------------------------------------------------------*/

/* USER CODE BEGIN Includes */
/* Section where include file can be added */
/* USER CODE END Includes */

/* Ensure definitions are only used by the compiler, and not by the assembler. */
#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
  #include <stdint.h>
  extern uint32_t SystemCoreClock;
  void xPortSysTickHandler(void);
#endif
#ifndef CMSIS_device_header
#define CMSIS_device_header "stm32f1xx.h"
#endif /* CMSIS_device_header */

#define configUSE_PREEMPTION                     1
#define configSUPPORT_STATIC_ALLOCATION          1
#define configSUPPORT_DYNAMIC_ALLOCATION         1
#define configUSE_IDLE_HOOK                      0
#define configUSE_TICK_HOOK                      0
#define configCPU_CLOCK_HZ                       ( SystemCoreClock )
#define configTICK_RATE_HZ                       ((TickType_t)1000)
#define configMAX_PRIORITIES                     ( 56 )
#define configMINIMAL_STACK_SIZE                 ((uint16_t)128)
/* 任务栈从这块堆里动态分配。实测（docs/GPS模块接入方案.md §6.4）：
   7 个应用任务栈 8448 B（空闲/定时器任务的栈是 cmsis_os2.c 里的静态数组，
   不占本堆）+ TCB 约 850 B + 定时器队列约 240 B ≈ 9.5 KB。
   F103C8 只有 20 KB RAM，而本工程静态占用已到 22.7 KB（超 2.2 KB），
   因此本值由 12288 收到 10240 以让整机装得下；配套把 OLED 栈 512→256 word、
   GPS 栈 192→128 word，堆余量约 2 KB。
   增加任务或加大栈时务必同步核对此值，否则 xTaskCreate 会静默失败。 */
/* ★ 由 10240 降到 9280：这是为 DIY IMU 模块腾 RAM 的关键一步。
   注意性质：configTOTAL_HEAP_SIZE 是 heap_4.c 里一个【固定的 .bss 数组】，
   不管里面装几个任务都占这么多 RAM。所以『少建一个任务』并不省 RAM，
   必须直接缩小这个数组本身。

   实测需求（heap_4：每次分配 = 8 B 块头 + 8 B 对齐后的大小）：
     Tmr Svc 栈(64 word)      256
     Tmr Svc TCB                96
     IDLE    栈(128 word)      512
     IDLE    TCB                96
     7 个应用任务 栈合计      6144
     7 个应用任务 TCB 合计      672
     定时器队列(10 项)            80
     ------------------------------
     合计                      7856 → 取 9280 仍有 1424 B 余量
   （2026-10 实测：含 GPS 任务时也只需 9080 B，故该值对两种配置都够。）

   ⚠️ 调小这个值会让 xTaskCreate 静默失败（configASSERT 是关中断死循环，
      屏幕上没有任何提示）。若怀疑堆不够，看 MX_FREERTOS_Init 里显示的
      FreeHeap 数字 —— 接近 0 就是不够。 */
/* ★ 2026-10-13：GPS 任务恢复创建（三路串口分工落定：USART1=GPS /
   USART2=HC-08 / USART3=DIY IMU），配套在【非 IMU】的地方腾了 864 B：
     FATIGUE 256→192 word  -256
     ALARM   256→192 word  -256
     BLE     256→192 word  -256
     定时器队列 10→4 项      -96
   腾完后的堆账（heap_4：每次分配 = 8 B 块头 + 8 B 对齐后大小）：
     含 GPS 的 7 个应用任务 栈 6144 + TCB 672 + 定时器队列(4 项) 120 ≈ 6936 B
   IMU 那一侧（imu_recv_task.c / diy_imu.c / USART3 配置与栈）本次一行未改。

   ★ 2026-10-14：接入 HC-08（BLE）后静态 RAM 只剩 240 B，于是把本值
   9280 → 8192（省 1088 B 静态 RAM）：堆实际需求约 6.9 KB，降完仍余约 1.2 KB，
   而静态 RAM 余量回到约 1.3 KB —— 静态 RAM 是硬约束（超了直接链接失败），
   堆余量偏小只会让"将来再加任务"时先失败，两者权衡后取 8192。
   ⚠ .ioc 里的 FREERTOS.configTOTAL_HEAP_SIZE 必须同步（已改）。 */
#define configTOTAL_HEAP_SIZE                    ((size_t)8192)
#define configMAX_TASK_NAME_LEN                  ( 16 )
#define configUSE_TRACE_FACILITY                 1
#define configUSE_16_BIT_TICKS                   0
#define configUSE_MUTEXES                        1
#define configQUEUE_REGISTRY_SIZE                8
#define configUSE_RECURSIVE_MUTEXES              1
#define configUSE_COUNTING_SEMAPHORES            1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  0

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES                    0
#define configMAX_CO_ROUTINE_PRIORITIES          ( 2 )

/* Software timer definitions. */
#define configUSE_TIMERS                         1
#define configTIMER_TASK_PRIORITY                ( 2 )
/* 本工程【没有创建任何软件定时器】，队列只在内核启动时用一下：
   10 -> 4 项，为恢复 GPS 任务腾堆（约 96 B）。 */
#define configTIMER_QUEUE_LENGTH                 4
/* 本工程未创建任何软件定时器，128 word(512 B) 足够；
   该栈是 cmsis_os2.c 里的静态数组，直接占 RAM，故从默认 256 收到 128。 */
#define configTIMER_TASK_STACK_DEPTH             64
/* ^ 由 128 降到 64：本工程未使用任何软件定时器，该任务只是空转。
     这是为 DIY IMU 模块腾 RAM 的一部分，详见 Core/Src/freertos.c 的 RAM 账本。 */

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */
#define INCLUDE_vTaskPrioritySet            1
#define INCLUDE_uxTaskPriorityGet           1
#define INCLUDE_vTaskDelete                 1
#define INCLUDE_vTaskCleanUpResources       0
#define INCLUDE_vTaskSuspend                1
#define INCLUDE_vTaskDelayUntil             1
#define INCLUDE_vTaskDelay                  1
#define INCLUDE_xTaskGetSchedulerState      1
#define INCLUDE_xTimerPendFunctionCall      1
#define INCLUDE_xQueueGetMutexHolder        1
#define INCLUDE_uxTaskGetStackHighWaterMark 1
#define INCLUDE_xTaskGetCurrentTaskHandle   1
#define INCLUDE_eTaskGetState               1

/*
 * The CMSIS-RTOS V2 FreeRTOS wrapper is dependent on the heap implementation used
 * by the application thus the correct define need to be enabled below
 */
#define USE_FreeRTOS_HEAP_4

/* Cortex-M specific definitions. */
#ifdef __NVIC_PRIO_BITS
 /* __BVIC_PRIO_BITS will be specified when CMSIS is being used. */
 #define configPRIO_BITS         __NVIC_PRIO_BITS
#else
 #define configPRIO_BITS         4
#endif

/* The lowest interrupt priority that can be used in a call to a "set priority"
function. */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY   15

/* The highest interrupt priority that can be used by any interrupt service
routine that makes calls to interrupt safe FreeRTOS API functions.  DO NOT CALL
INTERRUPT SAFE FREERTOS API FUNCTIONS FROM ANY INTERRUPT THAT HAS A HIGHER
PRIORITY THAN THIS! (higher priorities are lower numeric values. */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

/* Interrupt priorities used by the kernel port layer itself.  These are generic
to all Cortex-M ports, and do not rely on any particular library functions. */
#define configKERNEL_INTERRUPT_PRIORITY 		( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

/* Normal assert() semantics without relying on the provision of an assert.h
header file. */
/* USER CODE BEGIN 1 */
#define configASSERT( x ) if ((x) == 0) {taskDISABLE_INTERRUPTS(); for( ;; );}
/* USER CODE END 1 */

/* Definitions that map the FreeRTOS port interrupt handlers to their CMSIS
standard names. */
#define vPortSVCHandler    SVC_Handler
#define xPortPendSVHandler PendSV_Handler

/* IMPORTANT: After 10.3.1 update, Systick_Handler comes from NVIC (if SYS timebase = systick), otherwise from cmsis_os2.c */

#define USE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION 1

/* USER CODE BEGIN Defines */
/* Section where parameter definitions can be added (for instance, to override default ones in FreeRTOS.h) */
/* USER CODE END Defines */

#endif /* FREERTOS_CONFIG_H */
