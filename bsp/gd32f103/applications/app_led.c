/*
 * File      : main.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006 - 2018, RT-Thread Development Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-01-05     William      first implementation
 */

#include <rtthread.h>
#include <rtdevice.h>

#define LED_PIN             35

void app_led(void* argu)
{
    rt_pin_mode(LED_PIN, PIN_MODE_OUTPUT);
    
    while(1)
    {
        rt_pin_write(LED_PIN, PIN_HIGH);
        rt_thread_delay(500);
        rt_pin_write(LED_PIN, PIN_LOW);
        rt_thread_delay(500);
    }
}

int app_led_init(void)
{
    rt_thread_t tid;
    
    tid = rt_thread_create("app_led",
                            app_led,
                            RT_NULL,
                            256,
                            RT_THREAD_PRIORITY_MAX - 1,
                            1);
    
    if(tid != RT_NULL)
    {
        rt_thread_startup(tid);
        return 0;
    }
    return RT_ENOMEM;
}
INIT_APP_EXPORT(app_led_init);

