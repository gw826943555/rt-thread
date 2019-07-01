/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 *
 * Change Logs:
 * Date           Author            Notes
 * 2017-10-30     ZYH            the first version
 * 2017-11-15     ZYH          update to 3.0.0
 * 2019-04-28     William       port to gd32f103cb
 */
#include "drv_usb.h"
#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"

#define USB_DISCONNECT_PIN                  38      //PA15

#define USBD_DEBUG                          rt_kprintf

/* interrupt flag mask which decide what event should be handled by application */
#define INT_MASK          (CTL_STIE | CTL_WKUPIE | CTL_SPSIE \
                            | CTL_SOFIE | CTL_ESOFIE | CTL_RSTIE)
                            
#define ENDP_BUF_ADDR         (sizeof(usbd_ep_buf_struct) * EP_COUNT)
    
usbd_ep_buf_struct *pbuf_reg = (usbd_ep_buf_struct *)USBD_RAM;

static usbd_core_handle_struct _usb_dev;
static struct udcd _gd_udc;
static struct ep_id _ep_pool[] =
{
    {0x0,  USB_EP_ATTR_CONTROL,     USB_DIR_INOUT,  EP0_MAX_PACKET_SIZE, ID_ASSIGNED  },
    {0x1,  USB_EP_ATTR_BULK,        USB_DIR_IN,     EP1_MAX_PACKET_SIZE, ID_UNASSIGNED},
    {0x1,  USB_EP_ATTR_BULK,        USB_DIR_OUT,    EP1_MAX_PACKET_SIZE, ID_UNASSIGNED},
    {0x2,  USB_EP_ATTR_INT,         USB_DIR_IN,     EP2_MAX_PACKET_SIZE, ID_UNASSIGNED},
    {0x2,  USB_EP_ATTR_INT,         USB_DIR_OUT,    EP2_MAX_PACKET_SIZE, ID_UNASSIGNED},
    {0x3,  USB_EP_ATTR_INT,         USB_DIR_IN,     EP3_MAX_PACKET_SIZE, ID_UNASSIGNED},
    {0x3,  USB_EP_ATTR_INT,         USB_DIR_OUT,    EP3_MAX_PACKET_SIZE, ID_UNASSIGNED},
    {0xFF, USB_EP_ATTR_TYPE_MASK,   USB_DIR_MASK,   0,  ID_ASSIGNED  },
};

void usbd_int_reset(void)
{
    /* configure endpoint 0 buffer */
    pbuf_reg->tx_addr = (uint16_t)ENDP_BUF_ADDR;
    pbuf_reg->rx_addr = (uint16_t)ENDP_BUF_ADDR + EP0_MAX_PACKET_SIZE;
    
    /* configure endpoint 0 Rx count */
    if (EP0_MAX_PACKET_SIZE > 62U) {
        pbuf_reg->rx_count = ((EP0_MAX_PACKET_SIZE << 5U) - 1U) | 0x8000U;
    } else {
        pbuf_reg->rx_count = ((EP0_MAX_PACKET_SIZE + 1U) & ~1U) << 9U;
    }
    
    _usb_dev.in_ep[EP0].maxpacket = EP0_MAX_PACKET_SIZE;
    _usb_dev.out_ep[EP0].maxpacket = EP0_MAX_PACKET_SIZE;
    
    /* set endpoints address */
    for (int i = 0U; i < EP_COUNT; i++)
    {
        _EP_ADDR_SET(i, i);
    }
    
    /* set device address as default address 0 */
    USBD_REG_SET(USBD_DADDR, DADDR_USBEN);
    
    /* clear endpoint 0 register */
    USBD_REG_SET(USBD_EPxCS(EP0), USBD_EPxCS(EP0));

    USBD_REG_SET(USBD_EPxCS(EP0), EP_CONTROL | EPRX_VALID | EPTX_NAK);
    
    _usb_dev.status = USBD_DEFAULT;
    
    rt_usbd_reset_handler(&_gd_udc);
}

void usbd_int_data(void)
{
    uint8_t ep_num = 0;
    
    volatile int16_t int_status = 0;
    volatile int16_t ep_value = 0;
    
    /* wait till interrupts are not pending */
    while (0U != ((int_status = USBD_REG_GET(USBD_INTF)) & (uint16_t)INTF_STIF)) 
    {
        /* get endpoint number and the value of control and state register */
        ep_num = (uint8_t)(int_status & INTF_EPNUM);
        ep_value = USBD_REG_GET(USBD_EPxCS(ep_num));
        
        if (0U == (int_status & INTF_DIR)) 
        {
            if (0U != (ep_value & EPxCS_TX_ST)) 
            {
                /* clear successful transmit interrupt flag */
                USBD_ENDP_TX_STAT_CLEAR(ep_num);
                
                if (ep_num == 0)
                {
                    rt_usbd_ep0_in_handler(&_gd_udc);
                }
                else
                {
                    rt_usbd_ep_in_handler(&_gd_udc, 0x80 | ep_num, _usb_dev.in_ep[EP0].trs_count);
                }
            }
        }else
        {
            uint16_t count = 0U;
            
            if (0U != (ep_value & EPxCS_RX_ST))
            {
                /* clear successful receive interrupt flag */
                USBD_ENDP_RX_STAT_CLEAR(ep_num);
                
                count = (pbuf_reg + ep_num)->rx_count & (uint16_t)EPRCNT_CNT;
                
                if (0U != count)
                {
                    if (0U != (ep_value & EPxCS_SETUP))
                    {
                        /* handle setup packet */
//                        rt_usbd_ep0_setup_handler(&_gd_udc, (struct urequest*)_usb_dev.setup_packet);
                        rt_usbd_ep0_setup_handler(&_gd_udc, RT_NULL);
                    }else
                    {
                        if (ep_num != 0)
                        {
                            rt_usbd_ep_out_handler(&_gd_udc, ep_num, _usb_dev.out_ep[ep_num].trs_count);
                        }
                        else
                        {
                            rt_usbd_ep0_out_handler(&_gd_udc, _usb_dev.out_ep[0].trs_count);
                        }
                    }
                }
            }
        }
    }
}

void USBD_LP_CAN0_RX0_IRQHandler(void)
{
    rt_interrupt_enter();
    
    volatile uint16_t int_flag = 0;
    
    int_flag = USBD_REG_GET(USBD_INTF);
    
    if(int_flag & INTF_STIF)
    {
        USBD_DEBUG("USBD SUCCESS TRANSFER INT\n\r");
        usbd_int_data();
        USBD_REG_SET(USBD_INTF, (uint16_t)CLR_STIF);
    }
    
    if(int_flag &  INTF_WKUPIF)
    {
        USBD_DEBUG("USBD WAKEUP INT\n\r");
        /* clear wakeup interrupt flag in INTF */
        USBD_REG_SET(USBD_INTF, (uint16_t)CLR_WKUPIF);
    }
    
    if(int_flag &  INTF_SPSIF)
    {
        USBD_DEBUG("USBD SUSPEND INT\n\r");
        if(!(USBD_REG_GET(USBD_CTL) & CTL_RSREQ)) {
            /* clear of suspend interrupt flag bit must be done after setting of CTLR_SETSPS */
            USBD_REG_SET(USBD_INTF, (uint16_t)CLR_SPSIF);
        }
    }
    
    if(int_flag &  INTF_SOFIF)
    {
//        USBD_DEBUG("USBD SOF INT\n\r");
        USBD_REG_SET(USBD_INTF, (uint16_t)CLR_SOFIF);
    }
    
    if(int_flag &  INTF_ESOFIF)
    {
//        USBD_DEBUG("USBD ESOF INT\n\r");
        USBD_REG_SET(USBD_INTF, (uint16_t)CLR_ESOFIF);
    }
    
    if(int_flag &  INTF_RSTIF)
    {
        USBD_DEBUG("USBD RESET INT\r\n");
        
        /* clear reset interrupt flag in INTF */
        USBD_REG_SET(USBD_INTF, (uint16_t)CLR_RSTIF);
        
        usbd_int_reset();
    }
    rt_interrupt_leave();
}

static rt_err_t _set_address(rt_uint8_t address)
{
    return RT_EOK;
}

static rt_err_t _set_config(rt_uint8_t address)
{
    return RT_EOK;
}

static rt_err_t _ep_set_stall(rt_uint8_t address)
{
//    HAL_PCD_EP_SetStall(&_stm_pcd, address);
    return RT_EOK;
}

static rt_err_t _ep_clear_stall(rt_uint8_t address)
{
//    HAL_PCD_EP_ClrStall(&_stm_pcd, address);
    return RT_EOK;
}

static rt_err_t _ep_enable(uep_t ep)
{
    RT_ASSERT(ep != RT_NULL);
    RT_ASSERT(ep->ep_desc != RT_NULL);
//    HAL_PCD_EP_Open(&_stm_pcd, ep->ep_desc->bEndpointAddress,
//                    ep->ep_desc->wMaxPacketSize, ep->ep_desc->bmAttributes);
    return RT_EOK;
}

static rt_err_t _ep_disable(uep_t ep)
{
    RT_ASSERT(ep != RT_NULL);
    RT_ASSERT(ep->ep_desc != RT_NULL);
//    HAL_PCD_EP_Close(&_stm_pcd, ep->ep_desc->bEndpointAddress);
    return RT_EOK;
}

static rt_size_t _ep_read_prepare(rt_uint8_t address, void *buffer, rt_size_t size)
{
//    HAL_PCD_EP_Receive(&_stm_pcd, address, buffer, size);
    return size;
}

static rt_size_t _ep_read(rt_uint8_t address, void *buffer)
{
    rt_size_t size = 0;
    uint32_t *src = 0;
    uint16_t *dest = 0;
    RT_ASSERT(buffer != RT_NULL);
    
    size = (pbuf_reg + address)->rx_count & (uint16_t)EPRCNT_CNT;
    src = (uint32_t *)((uint32_t)((pbuf_reg + address)->rx_addr * 2U + USBD_RAM));
    dest = (uint16_t *)buffer;
    
    for (int n = 0U; n < (size + 1U) / 2U; n++) 
    {
        *dest++ = (uint16_t)*src++;
    }
    
    return size;
}

static rt_size_t _ep_write(rt_uint8_t address, void *buffer, rt_size_t size)
{
    uint32_t* dest = 0;
    uint16_t* src = 0;
    int8_t ep_num = 0;
    
    dest = (uint32_t *)((uint32_t)((pbuf_reg + address)->tx_addr * 2U + USBD_RAM));
    src = (uint16_t*)buffer;
    ep_num = address & 0x7F;
    
    for(int n=0; n<(size+1)/2U; n++)
    {
        *dest++ = *src++;
    }
    
    (pbuf_reg + ep_num)->tx_count = (uint16_t)size;
    
    /* enable endpoint to transmit */
    USBD_ENDP_TX_STATUS_SET(ep_num, EPTX_VALID);
    
    rt_kprintf("EP%d write %d bytes\r\n", ep_num, size);
    return size;
}

static rt_err_t _ep0_send_status(void)
{
//    HAL_PCD_EP_Transmit(&_stm_pcd, 0x00, NULL, 0);
    return RT_EOK;
}

static rt_err_t _suspend(void)
{
    return RT_EOK;
}

static rt_err_t _wakeup(void)
{
    return RT_EOK;
}

static rt_err_t _init(rt_device_t device)
{
    /* enable USB APB1 clock */
    rcu_periph_clock_enable(RCU_USBD);
    
    /* just reset the CLOSE bit */
    USBD_REG_SET(USBD_CTL, CTL_SETRST);
    
    /* clear SETRST bit in USBD_CTL register */
    USBD_REG_SET(USBD_CTL, 0U);
    
    /* clear all pending interrupts */
    USBD_REG_SET(USBD_INTF, 0U);
    
    /* set allocation buffer address */
    USBD_REG_SET(USBD_BADDR, BUFFER_ADDRESS & 0xFFF8U);
    
    /* enable all interrupts mask bits */
    USBD_REG_SET(USBD_CTL, INT_MASK);
    
    /* enable the USB low priority interrupt */
    nvic_irq_enable(USBD_LP_CAN0_RX0_IRQn, 1, 0);
    
    rt_pin_mode(USB_DISCONNECT_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(USB_DISCONNECT_PIN, PIN_HIGH);
    
    return RT_EOK;
}

const static struct udcd_ops _udc_ops =
{
    _set_address,
    _set_config,
    _ep_set_stall,
    _ep_clear_stall,
    _ep_enable,
    _ep_disable,
    _ep_read_prepare,
    _ep_read,
    _ep_write,
    _ep0_send_status,
    _suspend,
    _wakeup,
};

int stm_usbd_register(void)
{
    rt_memset((void *)&_gd_udc, 0, sizeof(struct udcd));
    _gd_udc.parent.type = RT_Device_Class_USBDevice;
    _gd_udc.parent.init = _init;
    _gd_udc.parent.user_data = RT_NULL;
    _gd_udc.ops = &_udc_ops;
    /* Register endpoint infomation */
    _gd_udc.ep_pool = _ep_pool;
    _gd_udc.ep0.id = &_ep_pool[0];
    rt_device_register((rt_device_t)&_gd_udc, "usbd", 0);
    rt_usb_device_init();
    return 0;
}
INIT_DEVICE_EXPORT(stm_usbd_register);
