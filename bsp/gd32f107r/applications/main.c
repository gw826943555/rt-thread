/*
 * File      : main.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2015-07-29     Arda.Fu      first implementation
 */
#include <rtthread.h>
#include <board.h>

#include <drivers/pin.h>

int main(void)
{
    /* user app entry */
		rt_pin_mode(37, PIN_MODE_OUTPUT);
		while(1)
		{
			rt_pin_write(37, PIN_HIGH);
			rt_thread_delay(500);
			rt_pin_write(37, PIN_LOW);
			rt_thread_delay(500);
		}

    return 0;
}





