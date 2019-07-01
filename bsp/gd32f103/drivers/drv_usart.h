/*
 * File      : drv_usart.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 */

#ifndef __USART_H__
#define __USART_H__

#define BAUD_RATE_2400                  2400
#define BAUD_RATE_4800                  4800
#define BAUD_RATE_9600                  9600
#define BAUD_RATE_19200                 19200
#define BAUD_RATE_38400                 38400
#define BAUD_RATE_57600                 57600
#define BAUD_RATE_115200                115200
#define BAUD_RATE_230400                230400
#define BAUD_RATE_460800                460800
#define BAUD_RATE_921600                921600
#define BAUD_RATE_2000000               2000000
#define BAUD_RATE_3000000               3000000

#define DATA_BITS_5                     5
#define DATA_BITS_6                     6
#define DATA_BITS_7                     7
#define DATA_BITS_8                     8
#define DATA_BITS_9                     9

#define STOP_BITS_1                     0
#define STOP_BITS_2                     1
#define STOP_BITS_3                     2
#define STOP_BITS_4                     3

#define PARITY_NONE                     0
#define PARITY_ODD                      1
#define PARITY_EVEN                     2

#define BIT_ORDER_LSB                   0
#define BIT_ORDER_MSB                   1

#define NRZ_NORMAL                      0       /* Non Return to Zero : normal mode */
#define NRZ_INVERTED                    1       /* Non Return to Zero : inverted mode */

#ifndef RT_SERIAL_RB_BUFSZ
#define RT_SERIAL_RB_BUFSZ              64
#endif

#define RT_UART_DEFAULT_BAUD_RATE			BAUD_RATE_115200
#define RT_UART_DEFAULT_DATA_BITS			DATA_BITS_8
#define RT_UART_DEFAULT_STOP_BITS			STOP_BITS_1
#define RT_UART_DEFAULT_PARITY				PARITY_NONE

#endif
