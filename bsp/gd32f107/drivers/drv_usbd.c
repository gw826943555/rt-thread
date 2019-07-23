#include <rtthread.h>

#ifdef RT_USING_USB_DEVICE
#include "drv_usbd.h"
#include <rtdevice.h>
#include <board.h>
#include "usb_regs.h"

static struct udcd _gd_udc;
usb_dev_struct _gd_usb;
static struct ep_id _ep_pool[] =
{
    {0x0,  USB_EP_ATTR_CONTROL,     USB_DIR_INOUT,  64, ID_ASSIGNED  },
    {0x1,  USB_EP_ATTR_BULK,        USB_DIR_IN,     64, ID_UNASSIGNED},
    {0x1,  USB_EP_ATTR_BULK,        USB_DIR_OUT,    64, ID_UNASSIGNED},
    {0x2,  USB_EP_ATTR_INT,         USB_DIR_IN,     32, ID_UNASSIGNED},
    {0x2,  USB_EP_ATTR_INT,         USB_DIR_OUT,    64, ID_UNASSIGNED},
    {0x3,  USB_EP_ATTR_BULK,        USB_DIR_IN,     32, ID_UNASSIGNED},
    {0x3,  USB_EP_ATTR_BULK,        USB_DIR_OUT,    64, ID_UNASSIGNED},
    {0xFF, USB_EP_ATTR_TYPE_MASK,   USB_DIR_MASK,   0,  ID_ASSIGNED  },
};

static void gd_usb_isr(void);
static void gd_usb_ep_open(uint8_t addr, uint16_t max_packet, uint8_t type);

void USBFS_IRQHandler(void)
{
    rt_interrupt_enter();
    gd_usb_isr();
    /* leave interrupt */
    rt_interrupt_leave();

}

static rt_err_t _ep_set_stall(rt_uint8_t address)
{
    uint8_t ep_num = address & 0x7FU;
    __IO uint32_t devepctl = 0U;

    if (address >> 7U) {
        devepctl = USB_DIEPxCTL((uint16_t)ep_num);

        /* set the endpoint disable bit */
        if (devepctl & DEPCTL_EPEN) {
            devepctl |= DEPCTL_EPD;
        }

        /* set the endpoint stall bit */
        devepctl |= DEPCTL_STALL;

        USB_DIEPxCTL((uint16_t)ep_num) = devepctl;
    } else {
        /* set the endpoint stall bit */
        USB_DOEPxCTL((uint16_t)ep_num) |= DEPCTL_STALL;
    }
    
    if((address & 0x7FU) == 0U)
    {
        /* Prepare the EP0 to start the first control setup */
        USB_DOEP0LEN  = 0;
        USB_DOEP0LEN |= DOEP0LEN_PCNT;
        USB_DOEP0LEN |= (3*8);
        USB_DOEP0LEN |= DOEP0LEN_STPCNT;
    }
    
    return RT_EOK;
}

static rt_err_t _ep_clear_stall(rt_uint8_t address)
{
    usb_ep_struct *ep;
    uint8_t ep_num = address & 0x7FU;
    __IO uint32_t devepctl = 0U;

    if(address >> 7U)
    {
        ep = &_gd_usb.in_ep[ep_num];

        devepctl = USB_DIEPxCTL((uint16_t)ep_num);

        /* clear the in endpoint stall bits */
        devepctl &= ~DEPCTL_STALL;

        if ((USB_EP_ATTR_INT == ep->endp_type) || (USB_EP_ATTR_BULK == ep->endp_type)) {
            devepctl |= DEPCTL_SEVNFRM;
        }

        USB_DIEPxCTL((uint16_t)ep_num) = devepctl;
    } 
    else 
    {
        ep = &_gd_usb.out_ep[ep_num];

        devepctl = USB_DOEPxCTL((uint16_t)ep_num);

        /* clear the out endpoint stall bits */
        devepctl &= ~DEPCTL_STALL;

        if ((USB_EP_ATTR_INT == ep->endp_type) || (USB_EP_ATTR_BULK == ep->endp_type)) {
            devepctl |= DEPCTL_SEVNFRM;
        }

        USB_DOEPxCTL((uint16_t)ep_num) = devepctl;
    }
    return RT_EOK;
}

static rt_err_t _set_address(rt_uint8_t address)
{
    USB_DCFG &= ~(DCFG_DAR);
    USB_DCFG |= ((uint32_t)address << 4U);
    return RT_EOK;
}

static rt_err_t _set_config(rt_uint8_t address)
{
    return RT_EOK;
}

static rt_err_t _ep_enable(uep_t ep)
{
    RT_ASSERT(ep != RT_NULL);
    RT_ASSERT(ep->ep_desc != RT_NULL);

    gd_usb_ep_open(ep->ep_desc->bEndpointAddress, ep->ep_desc->wMaxPacketSize, ep->ep_desc->type);
    
    return RT_EOK;
}

static rt_err_t _ep_disable(uep_t ep)
{
    RT_ASSERT(ep != RT_NULL);
    RT_ASSERT(ep->ep_desc != RT_NULL);

    uint32_t devepinten = 0U;
    uint8_t ep_num = ep->id->addr & 0x7FU;

    if (ep->id->addr >> 7U) {
        devepinten |= 1U << ep_num;

        USB_DIEPxCTL((uint16_t)ep_num) &= ~DEPCTL_EPACT;
    } else {
        devepinten |= (1U << ep_num) << 16U;

        USB_DOEPxCTL((uint16_t)ep_num) &= ~DEPCTL_EPACT;
    }

    /* disable the interrupts for this endpoint */
    USB_DAEPINTEN &= ~devepinten;
    
    return RT_EOK;
}

static rt_size_t _ep_read(rt_uint8_t address, void *buffer)
{
    rt_size_t size = 0;
    RT_ASSERT(buffer != RT_NULL);
    return size;
}

static rt_size_t _ep_read_prepare(rt_uint8_t address, void *buffer, rt_size_t size)
{
    usb_ep_struct *ep;
    uint8_t ep_num = address & 0x7FU;

    ep = &_gd_usb.out_ep[ep_num];

    /* setup and start the Xfer */
    ep->xfer_buff = buffer;
    ep->xfer_len = size;
    ep->xfer_count = 0U;
    
    if((address & 0x7FU) == 0U)
    {
        /* Zero Length Packet? */
        if(ep->xfer_len == 0)
        {
            USB_DOEP0LEN &= ~DIEP0LEN_PCNT;
            USB_DOEP0LEN |= (1U << 19);
            USB_DOEP0LEN &= ~(DIEP0LEN_TLEN);
        }
        else
        {
            /* Program the transfer size and packet count
              * as follows: xfersize = N * maxpacket +
              * short_packet pktcnt = N + (short_packet
              * exist ? 1 : 0)
              */
            USB_DOEP0LEN &= ~DIEP0LEN_PCNT;
            USB_DOEP0LEN &= ~(DIEP0LEN_TLEN);
            
            if(ep->xfer_len > ep->endp_mps)
            {
                ep->xfer_len = ep->endp_mps;
            }
            USB_DOEP0LEN |= (1U << 19);
            USB_DOEP0LEN |= (DIEP0LEN_TLEN & ep->xfer_len);
        }
        /* Enable the Rx FIFO Empty Interrupt for this EP */
        if (ep->xfer_len > 0)
        {
            USB_DIEPFEINTEN |= 1 << (ep_num);
        }
        /* EP enable, IN data in FIFO */
        USB_DOEP0CTL |= (DEP0CTL_CNAK | DEP0CTL_EPEN);
    }
    else
    {
        /* Zero Length Packet? */
        if(ep->xfer_len == 0)
        {
            USB_DOEPxLEN(ep_num) &= ~DEPLEN_PCNT;
            USB_DOEPxLEN(ep_num) |= (DEPLEN_PCNT & (1U << 19));
            USB_DOEPxLEN(ep_num) &= ~(DEPLEN_TLEN);
        }
        else
        {
            /* Program the transfer size and packet count
              * as follows: xfersize = N * maxpacket +
              * short_packet pktcnt = N + (short_packet
              * exist ? 1 : 0)
              */
            USB_DOEPxLEN(ep_num) &= ~DEPLEN_PCNT;
            USB_DOEPxLEN(ep_num) &= ~(DEPLEN_TLEN);
            USB_DOEPxLEN(ep_num) |= (DEPLEN_PCNT & (((ep->xfer_len + ep->endp_mps - 1) / ep->endp_mps) << 19));
            USB_DOEPxLEN(ep_num) |= (DEPLEN_TLEN & ep->xfer_len);
            
            if(USB_EP_ATTR_ISOC == ep->endp_type)
            {
                USB_DOEPxLEN(ep_num) &= ~(DIEPLEN_MCNT);
                USB_DOEPxLEN(ep_num) |= (DIEPLEN_MCNT & (1<<29));
            }
            
            if(USB_EP_ATTR_ISOC != ep->endp_type)
            {
                if(ep->xfer_len > 0)
                {
                    USB_DOEPINTEN |= 1 << ep_num;
                }
            }
            
            if(USB_EP_ATTR_ISOC == ep->endp_type)
            {
                if((USB_DSTAT & (1 << 8)) == 0)
                {
                    USB_DOEPxCTL(ep_num) |= DEPCTL_SODDFRM;
                }
                else
                {
                    USB_DOEPxCTL(ep_num) |= DEPCTL_SEVNFRM;
                }
            }
            /* EP enable, IN data in FIFO */
            USB_DOEPxCTL(ep_num) |= (DEPCTL_CNAK | DEPCTL_EPEN);
            
            if(USB_EP_ATTR_ISOC == ep->endp_type)
            {
                uint32_t count32b = (ep->xfer_len + 3) / 4;
                uint8_t *src = ep->xfer_buff;
                __IO uint32_t *fifo = USB_FIFO(ep_num);
                
                rt_kprintf("ep_%02x: ", address);
                for(int n=0; n<count32b; n++, src+=4)
                {
                    rt_kprintf("0x%08X ", *((__packed uint32_t *)src));
                    *fifo = *((__packed uint32_t *)src);
                }
                rt_kprintf("\r\n");
            }
        }
    }
    
    return size;
}

static rt_size_t _ep_write(rt_uint8_t address, void *buffer, rt_size_t size)
{
    usb_ep_struct *ep;
    uint8_t ep_num = address & 0x7FU;
    __IO uint32_t devepctl = 0U;
    __IO uint32_t deveplen = 0U;

    ep = &_gd_usb.in_ep[ep_num];

    /* setup and start the transfer */
    ep->xfer_buff = buffer;
    ep->xfer_len = size;
    ep->xfer_count = 0U;

    if((ep_num & 0x7FU) == 0U)
    {
        USB_DIEP0LEN &= ~(DIEP0LEN_PCNT);
        USB_DIEP0LEN &= ~(DIEP0LEN_TLEN);
        
        /* Zero Length Packet? */
        if(ep->xfer_len == 0)
        {
            USB_DIEP0LEN &= ~(DIEP0LEN_PCNT);
            USB_DIEP0LEN |= (DIEP0LEN_PCNT & (1 << 19));
            USB_DIEP0LEN &= ~(DIEP0LEN_TLEN);
        }
        else
        {
            /* Program the transfer size and packet count
            * as follows: xfersize = N * maxpacket +
            * short_packet pktcnt = N + (short_packet
            * exist ? 1 : 0)
            */
            USB_DIEP0LEN &= ~(DIEP0LEN_PCNT);
            USB_DIEP0LEN &= ~(DIEP0LEN_TLEN);
            
            if(ep->xfer_len > ep->endp_mps)
            {
                ep->xfer_len = ep->endp_mps;
            }
            USB_DIEP0LEN |= (DIEP0LEN_PCNT & (1 << 19));
            USB_DIEP0LEN |= (DIEP0LEN_TLEN & ep->xfer_len);
        }
        /* Enable the Tx FIFO Empty Interrupt for this EP */
        if (ep->xfer_len > 0)
        {
            USB_DIEPFEINTEN |= 1 << (ep_num);
        }
        
        /* EP enable */
        USB_DIEP0CTL |= (DEP0CTL_CNAK | DEP0CTL_EPEN);
    }
    else
    {
        USB_DIEPxLEN(ep_num) &= ~(DEPLEN_PCNT);
        USB_DIEPxLEN(ep_num) &= ~(DEPLEN_TLEN);
        
        if(ep->xfer_len == 0)
        {
            USB_DIEPxLEN(ep_num) |= (DEPLEN_TLEN & (ep->endp_mps));
            USB_DIEPxLEN(ep_num) |= (DEPLEN_PCNT & (1 << 19));
        }
        else
        {
            uint16_t pktcnt = (ep->xfer_len + ep->endp_mps - 1) / ep->endp_mps;
            USB_DIEPxLEN(ep_num) |= (DEPLEN_TLEN & (ep->endp_mps * pktcnt));
            USB_DIEPxLEN(ep_num) |= (DEPLEN_PCNT & (pktcnt << 19));
        }
        
        if(USB_EP_ATTR_ISOC == ep->endp_type)
        {
            if((USB_DSTAT & (1 << 8)) == 0)
            {
                USB_DIEPxCTL(ep_num) |= DEPCTL_SODDFRM;
            }
            else
            {
                USB_DIEPxCTL(ep_num) |= DEPCTL_SEVNFRM;
            }
        }
        /* EP enable */
        USB_DIEPxCTL(ep_num) |= (DEPCTL_CNAK | DEPCTL_EPEN);
    }
    
    return size;
}

static rt_err_t _ep0_send_status(void)
{
    _ep_write(0x00, NULL, 0);
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
    uint32_t count = 0U;

    /* usbfs clock configuration */
    {
        uint32_t system_clock = rcu_clock_freq_get(CK_SYS);
        uint32_t usbfs_prescaler;
        
        if (system_clock == 48000000) {
            usbfs_prescaler = RCU_CKUSB_CKPLL_DIV1;
        } else if (system_clock == 72000000) {
            usbfs_prescaler = RCU_CKUSB_CKPLL_DIV1_5;
        } else if (system_clock == 96000000) {
            usbfs_prescaler = RCU_CKUSB_CKPLL_DIV2;
        } else if (system_clock == 120000000) {
            usbfs_prescaler = RCU_CKUSB_CKPLL_DIV2_5;
        }  else {
            return RT_ERROR;
            /*  reserved  */
        }
        rcu_usb_clock_config(usbfs_prescaler);
        rcu_periph_clock_enable(RCU_USBFS);
    }
    
    /* usbfs device mode configuration */
    USB_GLOBAL_INT_DISABLE();
    
    {
        /* enable core soft reset */
        USB_GRSTCTL |= GRSTCTL_CSRST;
        /* wait for the core to be soft reset */
        do {
            if (++count > 200000U) {
                break;
            }
        } while (1U == (USB_GRSTCTL & GRSTCTL_CSRST));
        /* active the transceiver and enable vbus sensing */
        USB_GCCFG |= GCCFG_PWRON | GCCFG_VBUSACEN | GCCFG_VBUSBCEN;
        /* set tx fifo empty level to half empty mode */
        USB_GAHBCS &= ~GAHBCS_TXFTH | TXFIFO_EMPTY_HALF;
#ifdef VBUS_SENSING_ENABLED
        USB_GCCFG |= GCCFG_VBUSIG;
#endif /* VBUS_SENSING_DISABLED */
#ifdef USBD_SOFOEN
        USB_GCCFG |= GCCFG_SOFOEN;
#endif
    }
    
    /* force device mode*/
    {
        USB_GUSBCS &= ~GUSBCS_FHM;
        USB_GUSBCS |= GUSBCS_FDM;
    }
    
    /* Init endpoints structures */
    for(int n=0; n<USBFS_MAX_DEV_EPCOUNT; n++)
    {
        _gd_usb.in_ep[n].endp_mps = 0U;
        _gd_usb.in_ep[n].endp_type = USB_EP_ATTR_CONTROL;
        _gd_usb.in_ep[n].xfer_buff = 0;
        _gd_usb.in_ep[n].xfer_len = 0;
        
        _gd_usb.out_ep[n].endp_mps = 0U;
        _gd_usb.out_ep[n].endp_type = USB_EP_ATTR_CONTROL;
        _gd_usb.out_ep[n].xfer_buff = 0;
        _gd_usb.out_ep[n].xfer_len = 0;
    }
    
    /* set device disconnect */
    USB_SOFT_DISCONNECT_ENABLE();
    
    /* init device */
    {
        __IO uint32_t devinepintf = USB_DIEP0TFLEN;
        uint32_t ram_offset = 0U;
        /* restart the phy clock (maybe don't need to...) */
        USB_PWRCLKCTL = 0U;
        /* config periodic frmae interval to default */
        USB_DCFG &= ~DCFG_EOPFT;
        USB_DCFG |= FRAME_INTERVAL_80;
        /* set full speed PHY */
        USB_DCFG &= ~DCFG_DS;
        USB_DCFG |= USB_SPEED_INP_FULL;
        
        /* set fifo size */
        USB_GRFLEN = RX_FIFO_FS_SIZE;
        ram_offset += RX_FIFO_FS_SIZE;
        USB_DIEP0TFLEN = (TX0_FIFO_FS_SIZE << 16U) | ram_offset;
        ram_offset += TX0_FIFO_FS_SIZE;
        USB_DIEPxTFLEN(1) = (TX1_FIFO_FS_SIZE << 16U) | ram_offset;
        ram_offset += TX1_FIFO_FS_SIZE;
        USB_DIEPxTFLEN(2) = (TX2_FIFO_FS_SIZE << 16U) | ram_offset;
        ram_offset += TX2_FIFO_FS_SIZE;
        USB_DIEPxTFLEN(3) = (TX3_FIFO_FS_SIZE << 16U) | ram_offset;
        
        {
            uint32_t count = 0U;
            /* flush all tx fifos */
            USB_GRSTCTL = ((uint32_t)0x10 << 6U) | GRSTCTL_TXFF;
            /* wait for tx fifo flush bit is set */
            do {
                if (++count > 200000U) {
                    break;
                }
            } while (USB_GRSTCTL & GRSTCTL_TXFF);
        }
        
        {
            uint32_t count = 0U;
            /* flush entire rx fifo */
            USB_GRSTCTL = GRSTCTL_RXFF;
            /* wait for rx fifo flush bit is set */
            do {
                if (++count > 200000U) {
                    break;
                }
            } while (USB_GRSTCTL & GRSTCTL_RXFF);
        }
        /* clear all pending device interrupts */
        USB_DIEPINTEN = 0U;
        USB_DOEPINTEN = 0U;
        USB_DAEPINT = 0xFFFFFFFFU;
        USB_DAEPINTEN = 0U;
        
        /* configure all in/out endpoints */
        for ( int i = 0U; i < USBFS_MAX_DEV_EPCOUNT; i++) 
        {
            if (USB_DIEPxCTL(i) & DEPCTL_EPEN) {
                USB_DIEPxCTL(i) |= DEPCTL_EPD | DEPCTL_SNAK;
            } else {
                USB_DIEPxCTL(i) = 0U;
            }

            if (USB_DOEPxCTL(i) & DEPCTL_EPEN) {
                USB_DOEPxCTL(i) |= DEPCTL_EPD | DEPCTL_SNAK;
            } else {
                USB_DOEPxCTL(i) = 0U;
            }

            /* set in/out endpoint transfer length to 0 */
            USB_DIEPxLEN(i) = 0U;
            USB_DOEPxLEN(i) = 0U;

            /* clear all pending in/out endpoints interrupts */
            USB_DIEPxINTF(i) = 0xFFU;
            USB_DOEPxINTF(i) = 0xFFU;
        }
        
        {
            uint32_t int_mask = 0U;
            /* disable all interrupts */
            USB_GINTEN = 0U;
            /* clear any pending interrupts */
            USB_GINTF = 0xBFFFFFFFU;
            /* enable the common interrupts */
            {
                /* clear any pending USB interrupts */
                USB_GOTGINTF = 0xFFFFFFFFU;
                USB_GINTF = 0xBFFFFFFF;
                /* enable the usb wakeup and suspend interrupts */
                USB_GINTEN = GINTEN_WKUPIE | GINTEN_SPIE;
            }
            int_mask = GINTEN_RXFNEIE;
            /* enable device_mode-related interrupts */
            int_mask |= GINTEN_SPIE | GINTEN_RSTIE | GINTEN_ENUMFIE \
                       | GINTEN_IEPIE | GINTEN_OEPIE | GINTEN_WKUPIE | GINTEN_ISOONCIE \
                       | GINTEN_ISOINCIE;
#ifdef VBUS_SENSING_ENABLED
            int_mask |= GINTEN_SESIE | GINTEN_OTGIE;
#endif /* VBUS_SENSING_ENABLED */

            USB_GINTEN &= ~int_mask;
            USB_GINTEN |= int_mask;
        }
    }
    
    /* set device Connect */
    USB_SOFT_DISCONNECT_DISABLE();
    /* enable USB global interrupt */
    USB_GLOBAL_INT_ENABLE();
    /* enable usbfs interrupt*/
    nvic_irq_enable((uint8_t)USBFS_IRQn, 2U, 0U);
    
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


static void gd_usb_ep_open(uint8_t addr, uint16_t max_packet, uint8_t type)
{
    uint32_t devepinten = 0U;
    uint32_t devepctl = 0U;

    usb_ep_struct *_ep;
    uint8_t  ep_num  = addr & 0x7FU;
    uint8_t  ep_type = type & USB_EP_ATTR_TYPE_MASK;
    uint16_t ep_mps  = max_packet;
    usb_dir_enum ep_dir;

    if (addr >> 7U) 
    {
        _ep = &_gd_usb.in_ep[ep_num];

        devepinten |= 1U << ep_num;
        devepctl = USB_DIEPxCTL((uint16_t)ep_num);

        ep_dir = USB_TX;
    } 
    else 
    {
        _ep = &_gd_usb.out_ep[ep_num];

        devepinten |= (1U << ep_num) << 16U;
        devepctl = USB_DOEPxCTL((uint16_t)ep_num);

        ep_dir = USB_RX;
    }

    /* if the endpoint is not active, need change the endpoint control register */
    if (!(devepctl & DEPCTL_EPACT)) 
    {
        devepctl &= ~DEPCTL_MPL;
        devepctl |= ep_mps;

        devepctl &= ~DEPCTL_EPTYPE;
        devepctl |= (uint32_t)ep_type << 18U;

        if (USB_TX == ep_dir) {
            devepctl &= ~DIEPCTL_TXFNUM;
            devepctl |= (uint32_t)ep_num << 22U;
        }

        devepctl |= DEPCTL_SD0PID;
        devepctl |= DEPCTL_EPACT;
    }

    if (USB_TX == ep_dir) {
        USB_DIEPxCTL((uint16_t)ep_num) = devepctl;
    } else if (USB_RX == ep_dir) {
        USB_DOEPxCTL((uint16_t)ep_num) = devepctl;
    } else {
        /* no operation */
    }

    _ep->endp_mps = ep_mps;
    _ep->endp_type = ep_type;

    /* enable the interrupts for this endpoint */
    USB_DAEPINTEN |= devepinten;
}   

static void gd_usb_reset_cb(void)
{
    uint32_t count = 0;

    /* clear the remote wakeup signaling */
    USB_DCTL &= ~DCTL_RWKUP;

    /* flush the fifo */
    USB_GRSTCTL = ((uint32_t)0x10 << 6U) | GRSTCTL_TXFF;

    /* wait for tx fifo flush bit is set */
    do 
    {
        if (++count > 200000U) {
            break;
        }
    } while (USB_GRSTCTL & GRSTCTL_TXFF);

    for (int i = 0U; i < USBFS_MAX_DEV_EPCOUNT; i++) 
    {
        USB_DIEPxINTF((uint16_t)i) = 0xFFU;
        USB_DOEPxINTF((uint16_t)i) = 0xFFU;
    }

    /* clear all pending device endpoint interrupts */
    USB_DAEPINT = 0xFFFFFFFFU;

    /* enable endpoint 0 interrupts */
    USB_DAEPINTEN &= ~DAEPINTEN_OEPIE;
    USB_DAEPINTEN &= ~DAEPINTEN_IEPIE;
    USB_DAEPINTEN |= 0x10001U;

    /* enable out endpoint interrupts */
    USB_DOEPINTEN |= (DOEPINTEN_STPFEN | DOEPINTEN_TFEN | DOEPINTEN_EPDISEN);

    /* enable in endpoint interrupts */
    USB_DIEPINTEN |= (DIEPINTEN_TFEN | DIEPINTEN_CITOEN | DIEPINTEN_EPDISEN);

    /* reset device address */
    USB_DCFG &= ~DCFG_DAR;

    /* configure endpoint 0 to receive setup packets */
    USB_DOEPxLEN(0U) = 0;
    USB_DOEPxLEN(0U) |= (DOEP0LEN_PCNT & (1 << 19));
    USB_DOEPxLEN(0U) |= (3*8);
    USB_DOEPxLEN(0U) |= DOEP0LEN_STPCNT;
    
    /* clear usb reset interrupt */
    USB_GINTF = GINTF_RST;
}


static void gd_usb_ep_out_cb(void)
{
    uint8_t endp_num = 0U;
    uint32_t endp_intr = 0U;
    __IO uint32_t out_endp_intr = 0U;
    /* read in the device interrupt bits */
    USB_DAOEP_INTR_READ(endp_intr);
    while (endp_intr) 
    {
        if (endp_intr & 0x1U) 
        {
            USB_DOEP_INTR_READ(out_endp_intr, (uint16_t)endp_num);

            /* transfer complete interrupt */
            if (out_endp_intr & DOEPINTF_TF) 
            {
                USB_DOEPxINTF((uint16_t)endp_num) = DOEPINTF_TF;

                /* data receive is completed */
                if(0U == endp_num)
                {
                    if((_gd_usb.set_up[0] == 0x00) && (_gd_usb.set_up[1] == 0x05))
                    {
                        __nop();
                    }
                    rt_usbd_ep0_out_handler(&_gd_udc, _gd_usb.out_ep[endp_num].xfer_count);
                }
                else
                {
                    rt_usbd_ep_out_handler(&_gd_udc, endp_num, _gd_usb.out_ep[endp_num].xfer_count);
                }
            }

            /* endpoint disable interrupt */
            if (out_endp_intr & DOEPINTF_EPDIS) {
                USB_DOEPxINTF((uint16_t)endp_num) = DOEPINTF_EPDIS;
            }

            /* setup phase finished interrupt (just for control endpoints) */
            if (out_endp_intr & DOEPINTF_STPF) 
            {
                /* setup phase is completed */
                rt_kprintf("set_up:");
                for(int i=0; i<8; i++)
                {
                    rt_kprintf("0x%02x ", _gd_usb.set_up[i]);
                }
                rt_kprintf("\r\n");
                rt_usbd_ep0_setup_handler(&_gd_udc, (struct urequest*)_gd_usb.set_up);

                USB_DOEPxINTF((uint16_t)endp_num) = DOEPINTF_STPF;
                
                __nop();
            }

            /* back to back setup packets received */
            if (out_endp_intr & DOEPINTF_BTBSTP) 
            {
                USB_DOEPxINTF((uint16_t)endp_num) = DOEPINTF_BTBSTP;
            }
        }

        endp_num ++;
        endp_intr >>= 1;
    }
}


static void gd_usb_ep_in_cb(void)
{
    uint8_t endp_num = 0U;
    uint32_t endp_intr = 0U;

    __IO uint32_t in_endp_intr = 0U;

    /* get all in endpoints which have interrupts */
    USB_DAIEP_INTR_READ(endp_intr);

    while (endp_intr) 
    {
        if (endp_intr & 0x1U) 
        {
            USB_DIEP_INTR_READ(in_endp_intr, (uint16_t)endp_num);

            if (in_endp_intr & DIEPINTF_TF) 
            {
                /* disable the fifo empty interrupt for the endpoint */
                USB_DIEPFEINTEN &= ~(0x1U << endp_num);

                USB_DIEPxINTF((uint16_t)endp_num) = DIEPINTF_TF;

                /* data transmittion is completed */
                if (endp_num == 0)
                {
                    rt_usbd_ep0_in_handler(&_gd_udc);
                }
                else
                {
                    rt_usbd_ep_in_handler(&_gd_udc, 0x80 | endp_num, _gd_usb.in_ep[endp_num].xfer_count);
                }
            }

            if (in_endp_intr & DIEPINTF_CITO) 
            {
                USB_DIEPxINTF((uint16_t)endp_num) = DIEPINTF_CITO;
            }

            if (in_endp_intr & DIEPINTF_IEPNE) 
            {
                USB_DIEPxINTF((uint16_t)endp_num) = DIEPINTF_IEPNE;
            }

            if (in_endp_intr & DIEPINTF_EPDIS) 
            {
                USB_DIEPxINTF((uint16_t)endp_num) = DIEPINTF_EPDIS;
            }

            if (in_endp_intr & DIEPINTF_TXFE) 
            {
                uint32_t len = 0U, word_len = 0U;
                usb_ep_struct *ep;

                ep = &_gd_usb.in_ep[endp_num];
                len = ep->xfer_len - ep->xfer_count;

                if (len > ep->endp_mps) 
                {
                    len = ep->endp_mps;
                }

                word_len = (len + 3U) / 4U;

                while (((USB_DIEPxTFSTAT((uint16_t)endp_num) & DIEPTFSTAT_IEPTFS) > word_len) &&
                        (ep->xfer_count < ep->xfer_len) && (ep->xfer_len != 0)) 
                {
                    /* write the FIFO */
                    len = ep->xfer_len - ep->xfer_count;

                    if (len > ep->endp_mps) 
                    {
                        len = ep->endp_mps;
                    }

                    word_len = (len + 3U) / 4U;

                    __IO uint32_t *fifo = USB_FIFO(endp_num);
                    uint8_t *src = ep->xfer_buff;
                    rt_kprintf("ep_in%d: ", endp_num);
                    for (int i = 0U; i < word_len; i++) 
                    {
                        rt_kprintf("0x%08X ", *((__packed uint32_t *)src));
                        *fifo = *((__packed uint32_t *)src);
                        src += 4U;
                    }
                    rt_kprintf("\r\n");
                    
                    ep->xfer_buff += len;
                    ep->xfer_count += len;
                    
                    if(ep->xfer_len == ep->xfer_count)
                    {
                        USB_DIEPFEINTEN &= ~(0x01U <<  endp_num);
                    }
                }
                USB_DIEPxINTF((uint16_t)endp_num) = DIEPINTF_TXFE;
            }
        }

        endp_num ++;
        endp_intr >>= 1;
    }
}


static void gd_usb_recv_fifo_none_empty_cb(void)
{
    usb_ep_struct *ep;
    uint8_t data_pid = 0U, endp_num = 0U;
    uint32_t bcount = 0U, packet_num = 0U;

    /* get the status from the top of the fifo (must be read to a variable) */
    __IO uint32_t rx_status = USB_GRSTATP;

    /* disable the rx fifo non-empty interrupt */
    USB_GINTEN &= ~GINTEN_RXFNEIE;

    endp_num = (uint8_t)(rx_status & GRSTATRP_EPNUM);
    bcount = (rx_status & GRSTATRP_BCOUNT) >> 4U;
    data_pid = (uint8_t)((rx_status & GRSTATRP_DPID) >> 15U);

    /* ensure no-DMA mode can work */
    packet_num = USB_DOEPxLEN((uint16_t)endp_num) & DEPLEN_PCNT;
    if ((1U == endp_num) && (0U == packet_num)) {
        uint32_t devepctl = USB_DOEPxCTL((uint16_t)endp_num);

        devepctl |= DEPCTL_SNAK;
        devepctl &= ~DEPCTL_EPEN;
        devepctl &= ~DEPCTL_EPD;

        USB_DOEPxCTL((uint16_t)endp_num) = devepctl;
    }

    ep = &_gd_usb.out_ep[endp_num];

    switch ((rx_status & GRSTATRP_RPCKST) >> 17U) 
    {
        case RXSTAT_GOUT_NAK:
            break;
        case RXSTAT_DATA_UPDT:
            if (bcount > 0U) 
            {
                uint32_t i = 0U;
                uint32_t count32b = (bcount + 3U) / 4U;

                __IO uint32_t *fifo = USB_FIFO(0U);
                uint8_t *dest = ep->xfer_buff;

                rt_kprintf("ep_out%d: ", endp_num);
                for (i = 0U; i < count32b; i++) 
                {
                    *(__packed uint32_t *)dest = *fifo;
                    rt_kprintf("0x%08X ", *(__packed uint32_t *)dest);
                    
                    dest += 4U;
                }
                rt_kprintf("\r\n");
                ep->xfer_buff += bcount;
                ep->xfer_count += bcount;
            }
            break;
        case RXSTAT_XFER_COMP:
            break;
        case RXSTAT_SETUP_COMP:
            break;
        case RXSTAT_SETUP_UPDT:
            *(uint32_t *)0x5000081CU |= 0x00020000U;
            if ((0U == endp_num) && (8U == bcount) && (DPID_DATA0 == data_pid)) 
            {
                /* copy the setup packet received in fifo into the setup buffer in ram */
                uint8_t *dest = (uint8_t *)&_gd_usb.set_up;
                uint32_t i = 0U;
                uint32_t count32b = (8 + 3U) / 4U;

                __IO uint32_t *fifo = USB_FIFO(0U);
                
                rt_kprintf("ep_out%d: ", endp_num);
                for (i = 0U; i < count32b; i++) {
                    *(__packed uint32_t *)dest = *fifo;
                    rt_kprintf("0x%08X ", *(__packed uint32_t *)dest);
                    dest += 4U;
                }
                rt_kprintf("\r\n");
                
                ep->xfer_count += bcount;
            }
            break;
        default:
            break;
    }

    /* enable the rx fifo non-empty interrupt */
    USB_GINTEN |= GINTEN_RXFNEIE;
}


static void gd_usb_emuration_done_cb(void)
{
    uint8_t enum_speed = (uint8_t)((USB_DSTAT & DSTAT_ES) >> 1U);

    /* set the max packet size of devie in endpoint based on the enumeration speed */
    USB_DIEPxCTL(0U) &= ~ DEP0CTL_MPL;
    if(enum_speed == 0x01U)
    {
        USB_DIEPxCTL(0U) |= EP0MPL_8;
    }

    /* clear global in NAK */
    USB_DCTL &= ~DCTL_CGINAK;
    USB_DCTL |= DCTL_CGINAK;

    /* set USB turn-around time based on device speed and PHY interface */
    USB_GUSBCS &= ~GUSBCS_UTT;
    USB_GUSBCS |= 0x05U << 10;
    
    gd_usb_ep_open(0x00, 0x40, USB_EP_ATTR_CONTROL);
    gd_usb_ep_open(0x80, 0x40, USB_EP_ATTR_CONTROL);
    
    rt_usbd_reset_handler(&_gd_udc);
    
    /* clear interrupt */
    USB_GINTF = GINTF_ENUMF;
}

static void gd_usb_isr(void)
{
    uint32_t int_status = 0U, gintf = USB_GINTF, ginten = USB_GINTEN;
    
        /* ensure the core is in device mode */
    if (0 == USB_CURRENT_MODE_GET()) 
    {
        int_status = gintf & ginten;

        /* there are no interrupts, avoid spurious interrupt */
        if (!int_status) 
        {
            return ;
        }

        /* OUT endpoints interrupts */
        if (int_status & GINTF_OEPIF) 
        {
            gd_usb_ep_out_cb();
        }

        /* IN endpoints interrupts */
        if (int_status & GINTF_IEPIF) 
        {
            gd_usb_ep_in_cb();
        }

        /* mode mismatch interrupt */
        if (int_status & GINTF_MFIF) 
        {
            /* clear interrupt */
            USB_GINTF = GINTF_MFIF;
        }

        /* early suspend interrupt */
        if (int_status & GINTF_ESP) 
        {
            USB_GINTEN &= ~GINTEN_ESPIE;
            USB_GINTF = GINTF_ESP;
        }

        /* suspend interrupt */
        if (int_status & GINTF_SP) 
        {
            if(USB_DSTAT & DSTAT_SPST)
            {
                
            }
            /* clear interrupt */
            USB_GINTF = GINTF_SP;
        }

        /* wakeup interrupt */
        if (int_status & GINTF_WKUPIF) 
        {
            /* Clear the Remote Wake-up signalling */
            USB_DCTL &= ~DCTL_RWKUP;
            /* clear interrupt */
            USB_GINTF = GINTF_WKUPIF;
        }

        /* start of frame interrupt */
        if (int_status & GINTF_SOF) 
        {
            rt_usbd_sof_handler(&_gd_udc);
            USB_GINTF = GINTF_SOF;
        }

        /* reveive fifo not empty interrupt */
        if (int_status & GINTF_RXFNEIF) 
        {
            gd_usb_recv_fifo_none_empty_cb();
        }

        /* USB reset interrupt */
        if (int_status & GINTF_RST) 
        {
            gd_usb_reset_cb();
        }

        /* enumeration has been finished interrupt */
        if (int_status & GINTF_ENUMF) 
        {
            gd_usb_emuration_done_cb();
        }

        /* incomplete synchronization in transfer interrupt*/
        if (int_status & GINTF_ISOINCIF) {
            /* clear interrupt */
            USB_GINTF = GINTF_ISOINCIF;
        }

        /* incomplete synchronization out transfer interrupt*/
        if (int_status & GINTF_ISOONCIF) {
            /* clear interrupt */
            USB_GINTF = GINTF_ISOONCIF;
        }

#ifdef VBUS_SENSING_ENABLED

        /* session request interrupt */
        if (int_status & GINTF_SESIF) {
            retval |= usbd_intf_sessionrequest(pudev);
        }

        /* OTG mode interrupt */
        if (int_status & GINTF_OTGIF) {
            retval |= usbd_intf_otg(pudev);
        }
#endif /* VBUS_SENSING_ENABLED */
    }
}

int gd_usbd_register(void)
{
    rt_memset((void*)&_gd_udc, 0, sizeof(struct udcd));
    _gd_udc.parent.type = RT_Device_Class_USBDevice;
    _gd_udc.parent.init = _init;
//    _gd_udc.parent.user_data = &_stm_pcd;
    _gd_udc.ops = &_udc_ops;
    /* Register endpoint infomation */
    _gd_udc.ep_pool = _ep_pool;
    _gd_udc.ep0.id = &_ep_pool[0];
    rt_device_register((rt_device_t)&_gd_udc, "usbd", 0);
    rt_usb_device_init();
    return RT_EOK;
}
//INIT_DEVICE_EXPORT(gd_usbd_register);





#endif


