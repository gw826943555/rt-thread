/*
 * File      : board.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-09-22     Bernard      add board.h to this bsp
 * 2019-01-05     William      implement to gd32f107
 */

// <<< Use Configuration Wizard in Context Menu >>>
#ifndef __BOARD_H__
#define __BOARD_H__

// <o> Internal SRAM memory size[Kbytes] <8-96>
//	<i>Default: 96
#define GD32_SRAM_SIZE         96
#ifdef __CC_ARM
extern int Image$$RW_IRAM1$$RW$$Base;
extern int Image$$RW_IRAM1$$ZI$$Limit;
#define HEAP_END	  (void*)(&Image$$RW_IRAM1$$RW$$Base+ GD32_SRAM_SIZE * 1024 / 4)
#define HEAP_BEGIN    (void*)(&Image$$RW_IRAM1$$ZI$$Limit)
#elif __ICCARM__
#pragma section="HEAP"
#define HEAP_BEGIN    (__segment_end("HEAP"))
#else
extern int __bss_end;
#define HEAP_BEGIN    (&__bss_end)
#endif


#endif

//*** <<< end of configuration section >>>    ***
