/*
 * File      : drv_spi.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009 RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 20012-01-01    aozima       first implementation.
 * 2019-04-15     william      port for gd32f10x
 */

#ifndef __DRV_SPI_H__
#define __DRV_SPI_H__

#include <rtthread.h>
#include <drivers/spi.h>

#include "gd32f10x.h"

struct gd32f1_spi
{
    uint32_t spi_periph;
    rcu_periph_enum spi_clk;
    struct rt_spi_bus *spi_bus;
};


struct gd32_spi_cs
{
    uint32_t GPIOx;
    uint32_t GPIO_Pin;
};

/* public function */
rt_err_t gd32_spi_bus_register(uint32_t spi_periph,
								//struct gd32_spi_bus * gd32_spi,
								const char * spi_bus_name);

rt_err_t rt_hw_spi_device_attach(const char *bus_name, 
                                    const char *device_name, 
                                    uint32_t cs_gpiox, 
                                    uint16_t cs_gpio_pin);
#endif
