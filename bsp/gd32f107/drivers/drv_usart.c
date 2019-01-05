/*
 * File      : drv_usart.c
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
 * 2018-04-19     misonyo      Porting for gd32f10x
 */

#include <drv_usart.h>
#include <rtthread.h>
#include <rthw.h>
#include "gd32f10x.h"

#ifdef RT_USING_SERIAL

#ifndef RT_USING_DEVICE
#error "you must define RT_USING_DEVICE with uart device"
#endif

#ifndef RT_UART_RX_BUFFER_SIZE
#define RT_UART_RX_BUFFER_SIZE 64
#endif
#ifndef RT_UART_TX_BUFFER_SIZE
#define RT_UART_TX_BUFFER_SIZE 512
#endif

#define UART_ENABLE_IRQ(n)            NVIC_EnableIRQ((n))
#define UART_DISABLE_IRQ(n)           NVIC_DisableIRQ((n))

#if !defined(RT_USING_USART0) && !defined(RT_USING_USART1) && \
    !defined(RT_USING_USART2) && !defined(RT_USING_UART3)  && \
    !defined(RT_USING_UART4)
#error "Please define at least one UARTx"

#endif
/* GD32 uart driver */
// Todo: compress uart info
struct uart_cfg
{
    uint32_t baud_rate;
    
    rt_uint32_t data_bits               :4;
    rt_uint32_t stop_bits               :2;
    rt_uint32_t parity                  :2;
    rt_uint32_t bit_order               :1;
    rt_uint32_t invert                  :1;
    rt_uint32_t bufsz                   :16;
    rt_uint32_t reserved                :4;
};

struct gd32_uart
{
    struct rt_device parent;
	
    uint32_t uart_periph;
    IRQn_Type uart_irqn;
    rcu_periph_enum per_clk;
    rcu_periph_enum tx_gpio_clk;
    rcu_periph_enum rx_gpio_clk;
    uint32_t tx_port;
    uint16_t tx_pin;
    uint32_t rx_port;
    uint16_t rx_pin; 
#ifdef RT_USING_USART_DMA
    IRQn_Type rx_dma_irqn;
    uint32_t rx_dma_periph;
    uint32_t tx_dma_periph;
    dma_channel_enum rx_dma_channel;
    dma_channel_enum tx_dma_channel;
#endif
	
    struct uart_cfg cfg;
	
		/* buffer for reception */
    rt_uint8_t read_index, save_index;
    rt_uint8_t rx_buffer[RT_UART_RX_BUFFER_SIZE];
    rt_uint8_t tx_buffer[RT_UART_TX_BUFFER_SIZE];
};

static void uart_isr(struct gd32_uart *uart);
#ifdef RT_USING_USART_DMA
static void dma_rx_done_isr(struct gd32_uart * uart);
static void dma_tx_done_isr(struct gd32_uart * uart);
#endif

#if defined(RT_USING_USART0)
static struct gd32_uart uart0 =
{
	.uart_periph = USART0,
	.uart_irqn = USART0_IRQn,
	.per_clk = RCU_USART0, 
	.tx_gpio_clk = RCU_GPIOB, 
	.rx_gpio_clk = RCU_GPIOB,       						// periph clock, tx gpio clock, rt gpio clock
	.tx_port = GPIOB, 
	.tx_pin = GPIO_PIN_6, 											// tx port, tx pin
	.rx_port = GPIOB, 
	.rx_pin = GPIO_PIN_7,                     	// rx port, rx pin
#ifdef RT_USING_USART_DMA
    .rx_dma_irqn = DMA0_Channel4_IRQn,
    .rx_dma_periph = DMA0,
    .tx_dma_periph = DMA0,
    .rx_dma_channel = DMA_CH4,
    .tx_dma_channel = DMA_CH3,
#endif
    .cfg = 
	{
		.baud_rate 	= RT_UART_DEFAULT_BAUD_RATE,
		.data_bits 	= RT_UART_DEFAULT_DATA_BITS,
		.stop_bits 	= RT_UART_DEFAULT_STOP_BITS,
		.parity		= RT_UART_DEFAULT_PARITY,
	}
};

void USART0_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&uart0);

    /* leave interrupt */
    rt_interrupt_leave();
}

#ifdef RT_USING_USART_DMA
void DMA0_Channel4_IRQHandler(void)
{
    rt_interrupt_enter();
    
    dma_rx_done_isr(&uart0);
    
    rt_interrupt_leave();
}

void DMA0_Channel3_IRQHandler(void)
{
    rt_interrupt_enter();
    
    dma_tx_done_isr(&uart0);
    
    rt_interrupt_leave();
}
#endif

#endif /* RT_USING_USART0 */

#if defined(RT_USING_USART1)
struct gd32_uart uart1;

void USART1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&uart1);

    /* leave interrupt */
    rt_interrupt_leave();
}

#endif /* RT_USING_UART1 */

#if defined(RT_USING_USART2)
struct gd32_uart uart2;

void USART2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&uart2);

    /* leave interrupt */
    rt_interrupt_leave();
}

#endif /* RT_USING_UART2 */


/**
* @brief UART MSP Initialization
*        This function configures the hardware resources used in this example:
*           - Peripheral's clock enable
*           - Peripheral's GPIO Configuration
*           - NVIC configuration for UART interrupt request enable
* @param uart: UART handle pointer
* @retval None
*/
void gd32_uart_gpio_init(struct gd32_uart *uart)
{
    /* enable USART clock */
    rcu_periph_clock_enable(uart->tx_gpio_clk);
    rcu_periph_clock_enable(uart->rx_gpio_clk);
    rcu_periph_clock_enable(uart->per_clk);

    /* connect port to USARTx_Tx */
    gpio_init(uart->tx_port, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, uart->tx_pin);

    /* connect port to USARTx_Rx */
    gpio_init(uart->rx_port, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, uart->rx_pin);
	
#ifdef RT_USING_USART0_REMAP
		rcu_periph_clock_enable(RCU_AF);
		gpio_pin_remap_config(GPIO_USART0_REMAP, ENABLE);
#endif

    NVIC_SetPriority(uart->uart_irqn, 0);
    NVIC_EnableIRQ(uart->uart_irqn);
}

#ifdef RT_USING_USART_DMA
static void gd32_uart_dma_init(struct gd32_uart *uart)
{
    rcu_periph_clock_enable(RCU_DMA0);
    dma_deinit(uart->rx_dma_periph, uart->rx_dma_channel);
    dma_deinit(uart->tx_dma_periph, uart->tx_dma_channel);
    
    dma_parameter_struct dma_init_struct;
    dma_init_struct.direction = DMA_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_addr = (uint32_t)uart->rx_buffer;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number = RT_UART_RX_BUFFER_SIZE;
    dma_init_struct.periph_addr = (uint32_t)&(USART_DATA(uart->uart_periph));
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(uart->rx_dma_periph, uart->rx_dma_channel, &dma_init_struct);
    
    dma_init_struct.direction = DMA_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_addr = (uint32_t)uart->tx_buffer;
    dma_init_struct.number = 0;
    dma_init(uart->tx_dma_periph, uart->tx_dma_channel, &dma_init_struct);
    
    /* configure DMA mode */
    dma_circulation_enable(uart->rx_dma_periph, uart->rx_dma_channel);
    dma_memory_to_memory_disable(uart->rx_dma_periph, uart->rx_dma_channel);
    dma_circulation_disable(uart->tx_dma_periph, uart->tx_dma_channel);
    dma_memory_to_memory_disable(uart->tx_dma_periph, uart->tx_dma_channel);
    /* configure DMA interrupt */
    dma_interrupt_enable(uart->rx_dma_periph, uart->rx_dma_channel, DMA_INT_FTF);
    dma_interrupt_enable(uart->tx_dma_periph, uart->tx_dma_channel, DMA_INT_FTF);
    
    /* */
    nvic_irq_enable(uart->rx_dma_irqn, 0, 0);
    /* enable DMA channel */
    dma_channel_enable(uart->rx_dma_periph, uart->rx_dma_channel);
}

/**
 * DMA receive done process. This need add to DMA receive done ISR.
 *
 * @param serial serial device
 */
static void dma_rx_done_isr(struct gd32_uart * uart)
{
    if(dma_interrupt_flag_get(uart->rx_dma_periph, uart->rx_dma_channel, DMA_INTF_FTFIF) != RESET)
    {
        uart->save_index = 0;
        
        /* invoke callback */
        if (uart->parent.rx_indicate != RT_NULL)
        {
            rt_size_t length;
            if (uart->read_index > uart->save_index)
                length = RT_UART_RX_BUFFER_SIZE - uart->read_index + uart->save_index;
            else
                length = uart->save_index - uart->read_index;

            uart->parent.rx_indicate(&uart->parent, length);
        }
        
        dma_interrupt_flag_clear(uart->rx_dma_periph, uart->rx_dma_channel, DMA_INTF_FTFIF);
    }
}

/**
 * DMA transmit done process. This need add to DMA transmit done ISR.
 *
 * @param serial serial device
 */
static void dma_tx_done_isr(struct gd32_uart * uart)
{
    if(dma_interrupt_flag_get(uart->tx_dma_periph, uart->tx_dma_channel, DMA_INTF_FTFIF) != RESET)
    {
        dma_interrupt_flag_clear(uart->tx_dma_periph, uart->tx_dma_channel, DMA_INTF_FTFIF);
    }
}

#endif

static rt_err_t rt_uart_init(rt_device_t dev)
{
    struct gd32_uart *uart;
    struct uart_cfg * cfg;
    RT_ASSERT(dev != RT_NULL);
    uart = (struct gd32_uart *)dev;
    cfg = &(uart->cfg);
    
    gd32_uart_gpio_init(uart);
    
    usart_baudrate_set(uart->uart_periph, cfg->baud_rate);

    switch (cfg->data_bits)
    {
        case DATA_BITS_9:
            usart_word_length_set(uart->uart_periph, USART_WL_9BIT);
            break;

        default:
            usart_word_length_set(uart->uart_periph, USART_WL_8BIT);
            break;
    }

    switch (cfg->stop_bits)
    {
        case STOP_BITS_2:
            usart_stop_bit_set(uart->uart_periph, USART_STB_2BIT);
            break;
        default:
            usart_stop_bit_set(uart->uart_periph, USART_STB_1BIT);
            break;
    }

    switch (cfg->parity)
    {
        case PARITY_ODD:
            usart_parity_config(uart->uart_periph, USART_PM_ODD);
            break;
        case PARITY_EVEN:
            usart_parity_config(uart->uart_periph, USART_PM_EVEN);
            break;
        default:
            usart_parity_config(uart->uart_periph, USART_PM_NONE);
            break;
    }
#ifndef RT_USING_USART_DMA
    usart_interrupt_enable(uart->uart_periph, USART_INT_RBNE);
#else
    gd32_uart_dma_init(&uart0);
    usart_dma_receive_config(uart->uart_periph, USART_DENR_ENABLE);
    usart_dma_transmit_config(uart->uart_periph, USART_DENT_ENABLE);
    usart_interrupt_enable(uart->uart_periph, USART_INT_IDLE);
#endif
    usart_receive_config(uart->uart_periph, USART_RECEIVE_ENABLE);
    usart_transmit_config(uart->uart_periph, USART_TRANSMIT_ENABLE);
    usart_enable(uart->uart_periph);

    return RT_EOK;
}

static rt_err_t rt_uart_control(struct rt_device *serial, int cmd, void *arg)
{
    struct gd32_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct gd32_uart *)serial;

    switch (cmd)
    {
        case RT_DEVICE_CTRL_CLR_INT:
            /* disable rx irq */
            NVIC_DisableIRQ(uart->uart_irqn);
            /* disable interrupt */
#ifndef RT_USING_USART_DMA
            usart_interrupt_disable(uart->uart_periph, USART_INT_RBNE);
#else
            nvic_irq_disable(uart->rx_dma_irqn);
            usart_interrupt_disable(uart->uart_periph, USART_INT_IDLE);
#endif

            break;
        case RT_DEVICE_CTRL_SET_INT:
            /* enable rx irq */
            NVIC_EnableIRQ(uart->uart_irqn);
            /* enable interrupt */
#ifndef RT_USING_USART_DMA
            usart_interrupt_enable(uart->uart_periph, USART_INT_RBNE);
#else 
            nvic_irq_enable(uart->rx_dma_irqn, 0, 0);
            usart_interrupt_enable(uart->uart_periph, USART_INT_IDLE);
#endif
            break;
    }

    return RT_EOK;
}

static rt_size_t rt_uart_write(rt_device_t dev, rt_off_t pos, const void* buffer, rt_size_t size)
{
    char *ptr = (char*) buffer;
    struct gd32_uart* uart = (struct gd32_uart *)dev;
    
    RT_ASSERT(dev != RT_NULL);
    RT_ASSERT(buffer != RT_NULL);
    if (dev->open_flag & RT_DEVICE_FLAG_STREAM)
    {
        /* stream mode */
#ifndef RT_USING_USART_DMA
        while (size)
        {
            if (*ptr == '\n')
            {
                while((usart_flag_get(uart->uart_periph, USART_FLAG_TBE) == RESET));
                usart_data_transmit(uart->uart_periph, '\r');
            }
                    
            while((usart_flag_get(uart->uart_periph, USART_FLAG_TBE) == RESET));
            usart_data_transmit(uart->uart_periph, *ptr);
            
            ptr++;
            size--;
        }
#else
        uint32_t count = 0;
        if(size > RT_UART_TX_BUFFER_SIZE)
            size = RT_UART_TX_BUFFER_SIZE;
        while(dma_transfer_number_get(uart->tx_dma_periph, uart->tx_dma_channel) > 0) ;
        while(size)
        {
            if(*ptr == '\n')
            {
                uart->tx_buffer[count] = '\r';
                count++;
            }
            uart->tx_buffer[count] = *ptr;
            count++;
            ptr++;
            size--;
        }
        /* wait for last dma transmission*/
        dma_channel_disable(uart->tx_dma_periph, uart->tx_dma_channel);
        dma_transfer_number_config(uart->tx_dma_periph, uart->tx_dma_channel, count);
        dma_channel_enable(uart->tx_dma_periph, uart->tx_dma_channel);
#endif
    }else
    {
#ifndef RT_USING_USART_DMA
        while (size)
        {
            while((usart_flag_get(uart->uart_periph, USART_FLAG_TBE) == RESET));
            usart_data_transmit(uart->uart_periph, *ptr);
        
            ptr++;
            size--;
        }
#else
        if(size > RT_UART_TX_BUFFER_SIZE)
            size = RT_UART_TX_BUFFER_SIZE;
        rt_memcpy(uart->tx_buffer, ptr, size);
        /* wait for last dma transmission*/
        while(dma_transfer_number_get(uart->tx_dma_periph, uart->tx_dma_channel) > 0) ;
        dma_channel_disable(uart->tx_dma_periph, uart->tx_dma_channel);
        dma_transfer_number_config(uart->tx_dma_periph, uart->tx_dma_channel, size);
        dma_channel_enable(uart->tx_dma_periph, uart->tx_dma_channel);
#endif
    }
		
    return (rt_size_t)ptr - (rt_size_t)buffer;
}

static rt_size_t rt_uart_read(rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size)
{
    struct gd32_uart* uart = (struct gd32_uart *)dev;
    rt_uint8_t *ptr;
    rt_size_t length;
    
    RT_ASSERT(dev != RT_NULL);
    RT_ASSERT(buffer != RT_NULL);
    
    ptr = (rt_uint8_t *) buffer;
    while (size)
    {
        /* interrupt receive */
        rt_base_t level;
			
        /* disable interrupt */
        level = rt_hw_interrupt_disable();
        if (uart->read_index != uart->save_index)
        {
            *ptr = uart->rx_buffer[uart->read_index];

            uart->read_index ++;
            if (uart->read_index >= RT_UART_RX_BUFFER_SIZE)
                uart->read_index = 0;
        }
        else
        {
            /* no data in rx buffer */

            /* enable interrupt */
            rt_hw_interrupt_enable(level);
            break;
        }

        /* enable interrupt */
        rt_hw_interrupt_enable(level);

        ptr ++;
        size --;
    }
    length = (rt_uint32_t)ptr - (rt_uint32_t)buffer;
    return length;
}

/**
 * Uart common interrupt process. This need add to uart ISR.
 *
 * @param serial serial device
 */
static void uart_isr(struct gd32_uart *uart)
{
    rt_ubase_t level;
    /* enter interrupt */
    rt_interrupt_enter();
	
    level = rt_hw_interrupt_disable();
    
    if(usart_interrupt_flag_get(uart->uart_periph, USART_INT_FLAG_RBNE) == SET)
    {
        uart->rx_buffer[uart->save_index] = (rt_uint8_t)usart_data_receive(uart->uart_periph);
        uart->save_index ++;
        if (uart->save_index >= RT_UART_RX_BUFFER_SIZE)
        {
            uart->save_index = 0;
        }
            
        /* invoke callback */
        if (uart->parent.rx_indicate != RT_NULL)
        {
            rt_size_t length;
            if (uart->read_index > uart->save_index)
                length = RT_UART_RX_BUFFER_SIZE - uart->read_index + uart->save_index;
            else
                length = uart->save_index - uart->read_index;

            uart->parent.rx_indicate(&uart->parent, length);
        }
    }
    
#ifdef RT_USING_USART_DMA
    if(usart_interrupt_flag_get(uart->uart_periph, USART_INT_FLAG_IDLE) == SET)
    {
        uart->save_index = RT_UART_RX_BUFFER_SIZE - dma_transfer_number_get(uart->rx_dma_periph, uart->rx_dma_channel);
        /* invoke callback */
        if (uart->parent.rx_indicate != RT_NULL)
        {
            rt_size_t length;
            if (uart->read_index > uart->save_index)
                length = RT_UART_RX_BUFFER_SIZE - uart->read_index + uart->save_index;
            else
                length = uart->save_index - uart->read_index;

            uart->parent.rx_indicate(&uart->parent, length);
        }
        /* clear idle bit */
        USART_STAT(uart->uart_periph);
        USART_DATA(uart->uart_periph);
        
        usart_interrupt_flag_clear(uart->uart_periph, USART_INT_FLAG_IDLE);
    }
#endif
    
    rt_hw_interrupt_enable(level);
		
    /* leave interrupt */
    rt_interrupt_leave();
}

static rt_err_t rt_uart_open(rt_device_t dev, rt_uint16_t oflag)
{
    struct gd32_uart* uart;
    RT_ASSERT(dev != RT_NULL);
    uart = (struct gd32_uart *)dev;

    if (dev->flag & RT_DEVICE_FLAG_INT_RX)
    {
        /* Enable the UART Interrupt */
        NVIC_EnableIRQ(uart->uart_irqn);
#ifdef RT_USING_USART_DMA
        nvic_irq_enable(uart->rx_dma_irqn, 0, 0);
#endif
    }

    return RT_EOK;
}

static rt_err_t rt_uart_close(rt_device_t dev)
{
    struct gd32_uart* uart;
    RT_ASSERT(dev != RT_NULL);
    uart = (struct gd32_uart *)dev;

    if (dev->flag & RT_DEVICE_FLAG_INT_RX)
    {
        /* Disable the UART Interrupt */
        NVIC_DisableIRQ(uart->uart_irqn);
    }
#ifdef RT_USING_USART_DMA
        nvic_irq_disable(uart->rx_dma_irqn);
#endif
    return RT_EOK;
}

int rt_hw_usart_init(void)
{
#ifdef RT_USING_USART0
    {
        struct gd32_uart* uart;

        /* get uart device */
        uart = &uart0;

        /* device initialization */
        uart->parent.type = RT_Device_Class_Char;
        uart->read_index = 0;
        uart->save_index = 0;
        rt_memset(uart->rx_buffer, 0, sizeof(uart->rx_buffer));

        /* device interface */
        uart->parent.init 	    = rt_uart_init;
        uart->parent.open 	    = rt_uart_open;
        uart->parent.close      = rt_uart_close;
        uart->parent.read 	    = rt_uart_read;
        uart->parent.write      = rt_uart_write;
        uart->parent.control    = rt_uart_control;
        uart->parent.user_data  = RT_NULL;

        rt_device_register(&uart->parent, "uart0", RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    }
#endif /* RT_USING_UART1 */    
		return 0;
}
INIT_BOARD_EXPORT(rt_hw_usart_init);
#endif
