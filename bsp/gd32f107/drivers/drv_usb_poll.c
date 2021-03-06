#include <rtthread.h>
#ifdef RT_USING_USB_DEVICE
#include "drv_usb_poll.h"
#include <rtdevice.h>
#include <board.h>
#include "usb_regs.h"

#define DRV_USB_DEBUG       1
#if DRV_USB_DEBUG
#define USB_LOG             rt_kprintf         
#else
#define USB_LOG(...)
#endif

#define USB_INT             0

static struct udcd _gd_udc;
gd_usb_dev _gd_usbd;


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

static void gd_usb_flush_txfifo(uint8_t addr)
{
    uint32_t count = 0U;

    USB_GRSTCTL = ((uint32_t)addr << 6U) | GRSTCTL_TXFF;

    /* wait for tx fifo flush bit is set */
    do {
        if (++count > 200000U) {
            break;
        }
    } while (USB_GRSTCTL & GRSTCTL_TXFF);
}

static void gd_usb_flush_rxfifo(void)
{
    uint32_t count = 0U;

    USB_GRSTCTL = GRSTCTL_RXFF;

    /* wait for rx fifo flush bit is set */
    do {
        if (++count > 200000U) {
            break;
        }
    } while (USB_GRSTCTL & GRSTCTL_RXFF);
}

void *gd_usb_fifo_read (uint8_t *dest, uint16_t len)
{
    uint32_t i = 0U;
    uint32_t count32b = (len + 3U) / 4U;

    __IO uint32_t *fifo = USB_FIFO(0U);

    for (i = 0U; i < count32b; i++) {
        *(__packed uint32_t *)dest = *fifo;
        
        dest += 4U;
    }

    return ((void *)dest);
}

static void gd_usb_write_fifo(gd_usb_dev *dev, uint8_t ep_num)
{
    uint32_t len = 0U, word_len = 0U, fifo_empty_mask = 0U;
    usb_ep_struct *ep;

    ep = &dev->in_ep[ep_num];
    len = ep->xfer_len - ep->xfer_count;

    if (len > ep->endp_mps) 
    {
        len = ep->endp_mps;
    }

    word_len = (len + 3U) / 4U;

    while (((USB_DIEPxTFSTAT((uint16_t)ep_num) & DIEPTFSTAT_IEPTFS) > word_len) &&
            (ep->xfer_count < ep->xfer_len)) 
    {
        /* write the FIFO */
        len = ep->xfer_len - ep->xfer_count;

        if (len > ep->endp_mps) 
        {
            len = ep->endp_mps;
        }

        word_len = (len + 3U) / 4U;
        __IO uint32_t* fifo = USB_FIFO(ep_num);

#if DRV_USB_DEBUG
        USB_LOG("EP%d IN:" , ep_num);
        for(uint32_t index=0; index<word_len*4; index++)
        {
            USB_LOG("0x%02x ", ep->xfer_buff[index]);
        }
        USB_LOG("\r\n");
#endif
        
        for(uint32_t index=0; index<word_len; index++)
        {
            *fifo = *((__packed uint32_t *)ep->xfer_buff);
            ep->xfer_buff += 4;
        }

        ep->xfer_count += len;
        
        if(ep->xfer_len == ep->xfer_count) {
            fifo_empty_mask = 0x1U << ep_num;
            USB_DIEPFEINTEN &= ~fifo_empty_mask;
        }
    }
}

static void gd_usb_ep_out_handler(void)
{
    /* OUT endpoints interrupts */
    
    uint8_t endp_num = 0U;
    uint32_t endp_intr = 0U;
    __IO uint32_t out_endp_intr = 0U;
    
    /* read in the device interrupt bits */
    USB_DAOEP_INTR_READ(endp_intr);
    
    while(endp_intr)
    {
        if (endp_intr & 0x1U) 
        {
//            USB_DOEP_INTR_READ(out_endp_intr, (uint16_t)endp_num);
            out_endp_intr = USB_DOEPxINTF(endp_num);
            USB_LOG("EP%d out handle\r\n", endp_num);
            
            USB_LOG("USB_DOEP%dINTF:%X\r\n", endp_num, out_endp_intr);
            /* transfer complete interrupt */
            if(out_endp_intr & DOEPINTF_TF)
            {
                USB_DOEPxINTF((uint16_t)endp_num) = DOEPINTF_TF;
                if (endp_num != 0)
                {
                    rt_usbd_ep_out_handler(&_gd_udc, endp_num, _gd_usbd.out_ep[endp_num].xfer_count);
                }
                else
                {
                    rt_usbd_ep0_out_handler(&_gd_udc, _gd_usbd.out_ep[0].xfer_count);
                }
            }
            
            /* endpoint disable interrupt */
            if (out_endp_intr & DOEPINTF_EPDIS) 
            {
                USB_DOEPxINTF((uint16_t)endp_num) = DOEPINTF_EPDIS;
            }
            
            /* setup phase finished interrupt (just for control endpoints) */
            if (out_endp_intr & DOEPINTF_STPF) 
            {
                /* setup phase is completed */
                USB_LOG("set up handle\r\n");
//                rt_usbd_ep0_setup_handler(&_gd_udc, (struct urequest *)_gd_usbd.set_up);
                USB_DOEPxINTF((uint16_t)endp_num) = DOEPINTF_STPF;
            }
            /* back to back setup packets received */
            if (out_endp_intr & DOEPINTF_BTBSTP) 
            {
                USB_DOEPxINTF((uint16_t)endp_num) = DOEPINTF_BTBSTP;
            }
            
            USB_LOG("USB_DOEP%dINTF:%X\r\n", endp_num, USB_DOEPxINTF(endp_num));
        }
        endp_num ++;
        endp_intr >>= 1;
    }
}

static void gd_usb_ep_in_handler(void)
{
    /* IN endpoints interrupts */

    uint8_t endp_num = 0U;
    uint32_t endp_intr = 0U;

    __IO uint32_t in_endp_intr = 0U;

    /* get all in endpoints which have interrupts */
    USB_DAIEP_INTR_READ(endp_intr);
    
    while (endp_intr) 
    {
        if(endp_intr & 0x1U)
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
                    rt_usbd_ep_in_handler(&_gd_udc, 0x80 | endp_num, _gd_usbd.in_ep[endp_num].xfer_count);
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
                gd_usb_write_fifo(&_gd_usbd, endp_num);
                USB_DIEPxINTF((uint16_t)endp_num) = DIEPINTF_TXFE;
            }
        }
        endp_num ++;
        endp_intr >>= 1;
    }
}

static void gd_usb_recv_fifo_not_empty_handler(void)
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
    if ((1U == endp_num) && (0U == packet_num)) 
    {
        uint32_t devepctl = USB_DOEPxCTL((uint16_t)endp_num);

        devepctl |= DEPCTL_SNAK;
        devepctl &= ~DEPCTL_EPEN;
        devepctl &= ~DEPCTL_EPD;

        USB_DOEPxCTL((uint16_t)endp_num) = devepctl;
    }

    ep = &_gd_usbd.out_ep[endp_num];

    switch ((rx_status & GRSTATRP_RPCKST) >> 17U) {
        case RXSTAT_GOUT_NAK:
            break;
        case RXSTAT_DATA_UPDT:
            if (bcount > 0U) 
            {
                gd_usb_fifo_read(ep->xfer_buff, (uint16_t)bcount);
                ep->xfer_buff += bcount;
                ep->xfer_count += bcount;
                
#if DRV_USB_DEBUG
                USB_LOG("EP%d out:", endp_num);
                for(uint32_t index=0; index<bcount; index++)
                {
                    USB_LOG("0x%02x ", *(uint8_t*)(ep->xfer_buff - bcount + index));
                }
                USB_LOG("\r\n");
#endif
                
            }
            break;
        case RXSTAT_XFER_COMP:
            USB_LOG("EP%d out done\r\n", endp_num);
            break;
        case RXSTAT_SETUP_COMP:
            USB_LOG("setup recv done\r\n");
            rt_usbd_ep0_setup_handler(&_gd_udc, (struct urequest *)_gd_usbd.set_up);
            break;
        case RXSTAT_SETUP_UPDT:
//            *(uint32_t *)0x5000081CU |= 0x00020000U;
//            USB_DAEPINTEN |= 0x00020000U;
            if ((0U == endp_num) && (8U == bcount) && (DPID_DATA0 == data_pid)) 
            {
                /* copy the setup packet received in fifo into the setup buffer in ram */
                gd_usb_fifo_read(_gd_usbd.set_up, 8U);
                ep->xfer_count += bcount;
                
#if DRV_USB_DEBUG
                USB_LOG("setup:");
                for(uint32_t index=0; index<bcount; index++)
                {
                    USB_LOG("0x%02x ", _gd_usbd.set_up[index]);
                }
                USB_LOG("\r\n");
#endif
                
            }
            break;
        default:
            break;
    }

    /* enable the rx fifo non-empty interrupt */
    USB_GINTEN |= GINTEN_RXFNEIE;
}

static void gd_usb_reset_handler(void)
{
    usb_ep_struct *ep;
    
    /* clear the remote wakeup signaling */
    USB_DCTL &= ~DCTL_RWKUP;
    gd_usb_flush_txfifo(0);
    
    for (uint32_t i = 0U; i < 4; i++) 
    {
        USB_DIEPxINTF((uint16_t)i) = 0xFFU;
        USB_DOEPxINTF((uint16_t)i) = 0xFFU;
    }
    
    /* clear all pending device endpoint interrupts */
    USB_DAEPINT = 0xFFFFFFFFU;

    /* enable endpoint 0 interrupts */
    USB_DAEPINTEN &= ~DAEPINTEN_OEPIE;
    USB_DAEPINTEN &= ~DAEPINTEN_IEPIE;
    USB_DAEPINTEN = (1U << 16) | 1U;
    
    /* enable out endpoint interrupts */
    USB_DOEPINTEN = DOEPINTEN_STPFEN | DOEPINTEN_TFEN | DOEPINTEN_EPDISEN;

    /* enable in endpoint interrupts */
    USB_DIEPINTEN = DIEPINTEN_TFEN | DIEPINTEN_CITOEN | DIEPINTEN_EPDISEN;

    /* reset device address */
    USB_DCFG &= ~DCFG_DAR;
    USB_DCFG |= 0U << 4U;
    
    /* configure endpoint 0 to receive setup packets */
    /* set out endpoint 0 receive length to 24 bytes */
    USB_DOEPxLEN(0U) &= ~DOEP0LEN_TLEN;
    USB_DOEPxLEN(0U) |= 8U * 3U;

    /* set out endpoint 0 receive length to 1 packet */
    USB_DOEPxLEN(0U) &= ~DOEP0LEN_PCNT;
    USB_DOEPxLEN(0U) |= 1U << 19;

    /* set setup packet count to 3 */
    USB_DOEPxLEN(0U) &= ~DOEP0LEN_STPCNT;
    USB_DOEPxLEN(0U) |= 3U << 29;
    
    USB_GINTF = GINTF_RST;
    
    /* open EP0 IN */
    ep = &_gd_usbd.in_ep[0];
    
    USB_DIEPxCTL(0U) &= ~DEP0CTL_MPL;
    USB_DIEPxCTL(0U) &= ~DEPCTL_EPTYPE;
    USB_DIEPxCTL(0U) &= ~DIEPCTL_TXFNUM;

    if (!(USB_DIEPxCTL(0U) & DEP0CTL_EPACT)) 
    {
        USB_DIEPxCTL(0U) |= EP0_MAX_PACKET_SIZE;
        USB_DIEPxCTL(0U) |= (USB_EP_ATTR_CONTROL << 18U);
        USB_DIEPxCTL(0U) |= DEP0CTL_EPACT;
    }
    
    ep->endp_mps = EP0_MAX_PACKET_SIZE;
    ep->endp_type = USB_EP_ATTR_CONTROL;
    
    /* open EP0 OUT */
    ep = &_gd_usbd.out_ep[0];
    
    USB_DOEPxCTL(0U) &= ~DEP0CTL_MPL;
    USB_DOEPxCTL(0U) &= ~DEPCTL_EPTYPE;

    if (!(USB_DOEPxCTL(0U) & DEP0CTL_EPACT)) 
    {
        USB_DOEPxCTL(0U) |= EP0_MAX_PACKET_SIZE;
        USB_DOEPxCTL(0U) |= (USB_EP_ATTR_CONTROL << 18U);
        USB_DOEPxCTL(0U) |= DEP0CTL_EPACT;
    }
    
    ep->endp_mps = EP0_MAX_PACKET_SIZE;
    ep->endp_type = USB_EP_ATTR_CONTROL;
    
    rt_usbd_reset_handler(&_gd_udc);
}

static void gd_usb_emuration_finish_handler(void)
{
    /* set the max packet size of devie in endpoint based on the enumeration speed */
    USB_DIEPxCTL(0U) |= DEP0CTL_MPL_64;
    
    /* clear global in NAK */
    USB_DCTL &= ~DCTL_CGINAK;
    USB_DCTL |= DCTL_CGINAK;
    
    /* set USB turn-around time based on device speed and PHY interface */
    USB_GUSBCS &= ~GUSBCS_UTT;
    USB_GUSBCS |= 0x05U << 10;
    
    USB_GINTF = GINTF_ENUMF;
}

static void gd_usb_poll(void)
{
    /* Read interrupt status register. */
    uint32_t intsts = USB_GINTF;
    
    /* there are no interrupts, avoid spurious interrupt */
    if (!intsts) 
    {
        return ;
    }
    
    if(intsts & GINTF_OEPIF)
    {
        gd_usb_ep_out_handler();
    }
    
    if(intsts & GINTF_IEPIF)
    {
        gd_usb_ep_in_handler();
    }
    
    /* mode mismatch interrupt */
    if (intsts & GINTF_MFIF) 
    {
        /* clear interrupt */
        USB_GINTF = GINTF_MFIF;
    }
    
    /* early suspend interrupt */
    if (intsts & GINTF_ESP) 
    {
        USB_GINTEN &= ~GINTEN_ESPIE;
        USB_GINTF = GINTF_ESP;
    }
    
    /* suspend interrupt */
    if (intsts & GINTF_SP) 
    {
        USB_GINTF = GINTF_SP;
    }
    
    /* wakeup interrupt */
    if (intsts & GINTF_WKUPIF) 
    {
        USB_GINTF = GINTF_WKUPIF;
    }
    
    /* start of frame interrupt */
    if (intsts & GINTF_SOF) 
    {
        rt_usbd_sof_handler(&_gd_udc);
        USB_GINTF = GINTF_SOF;
    }
    
    /* reveive fifo not empty interrupt */
    if (intsts & GINTF_RXFNEIF) 
    {
        gd_usb_recv_fifo_not_empty_handler();
    }
    
    /* USB reset interrupt */
    if (intsts & GINTF_RST) 
    {
        gd_usb_reset_handler();
    }
    
    /* enumeration has been finished interrupt */
    if (intsts & GINTF_ENUMF) 
    {
        gd_usb_emuration_finish_handler();
    }
    
    /* incomplete synchronization in transfer interrupt*/
    if (intsts & GINTF_ISOINCIF) 
    {
        USB_GINTF |= GINTF_ISOINCIF;
    }
    
    /* incomplete synchronization out transfer interrupt*/
    if (intsts & GINTF_ISOONCIF) 
    {
        USB_GINTF |= GINTF_ISOONCIF;
    }
}

#if USB_INT
void  USBFS_IRQHandler (void)
{
    rt_interrupt_enter();
    gd_usb_poll();
    rt_interrupt_leave();
}
#endif

static rt_err_t _ep_set_stall(rt_uint8_t address)
{
    uint8_t ep_num = address & 0x7FU;
    __IO uint32_t devepctl = 0U;
    
    USB_LOG("ep%d set stall\r\n", ep_num);

    if (address >> 7U) 
    {
        devepctl = USB_DIEPxCTL((uint16_t)ep_num);

        /* set the endpoint disable bit */
        if (devepctl & DEPCTL_EPEN) 
        {
            devepctl |= DEPCTL_EPD;
        }

        /* set the endpoint stall bit */
        devepctl |= DEPCTL_STALL;

        USB_DIEPxCTL((uint16_t)ep_num) = devepctl;
    } 
    else 
    {
        /* set the endpoint stall bit */
        USB_DOEPxCTL((uint16_t)ep_num) |= DEPCTL_STALL;
    }
    
    if((address & 0x7FU) == 0)
    {
        __IO uint32_t ep0len = 0U;

        /* set out endpoint 0 receive length to 24 bytes */
        ep0len &= ~DOEP0LEN_TLEN;
        ep0len |= 8U * 3U;

        /* set out endpoint 0 receive length to 1 packet */
        ep0len &= ~DOEP0LEN_PCNT;
        ep0len |= 1U << 19;

        /* set setup packet count to 3 */
        ep0len &= ~DOEP0LEN_STPCNT;
        ep0len |= 3U << 29;

        USB_DOEPxLEN(0U) = ep0len;
    }
    
    return RT_EOK;
}

static rt_err_t _ep_clear_stall(rt_uint8_t address)
{
    usb_ep_struct *ep;
    uint8_t ep_num = address & 0x7FU;
    __IO uint32_t devepctl = 0U;
    
    USB_LOG("ep%d clear stall\r\n", ep_num);

    if(address >> 7U)
    {
        ep = &_gd_usbd.in_ep[ep_num];

        devepctl = USB_DIEPxCTL((uint16_t)ep_num);

        /* clear the in endpoint stall bits */
        devepctl &= ~DEPCTL_STALL;

        if ((USB_EP_ATTR_INT == ep->endp_type) || (USB_EP_ATTR_BULK == ep->endp_type)) 
        {
            devepctl |= DEPCTL_SEVNFRM;
        }

        USB_DIEPxCTL((uint16_t)ep_num) = devepctl;
    } 
    else 
    {
        ep = &_gd_usbd.out_ep[ep_num];

        devepctl = USB_DOEPxCTL((uint16_t)ep_num);

        /* clear the out endpoint stall bits */
        devepctl &= ~DEPCTL_STALL;

        if ((USB_EP_ATTR_INT == ep->endp_type) || (USB_EP_ATTR_BULK == ep->endp_type)) 
        {
            devepctl |= DEPCTL_SEVNFRM;
        }

        USB_DOEPxCTL((uint16_t)ep_num) = devepctl;
    }
    
    return RT_EOK;
}

static rt_err_t _set_address(rt_uint8_t address)
{
    USB_DCFG = (USB_DCFG & ~DCFG_DAR) | (address << 4);
    return RT_EOK;
}

static rt_err_t _set_config(rt_uint8_t address)
{
    return RT_EOK;
}

static rt_err_t _ep_enable(uep_t ep_t)
{
    RT_ASSERT(ep_t != RT_NULL);
    RT_ASSERT(ep_t->ep_desc != RT_NULL);
    
    uint32_t devepinten = 0U;
    uint32_t devepctl = 0U;
    usb_ep_struct *ep;
    uint8_t ep_dir;

    uint8_t ep_num = ep_t->ep_desc->bEndpointAddress & 0x7FU;
    uint8_t ep_type = ep_t->ep_desc->bmAttributes & USB_EP_ATTR_TYPE_MASK;
    uint16_t ep_mps = ep_t->ep_desc->wMaxPacketSize;
    
    USB_LOG("ep%02x open\r\n", ep_t->ep_desc->bEndpointAddress);
    
    if (ep_t->ep_desc->bEndpointAddress >> 7U) 
    {
        ep = &_gd_usbd.in_ep[ep_num];

        devepinten |= 1U << ep_num;
        devepctl = USB_DIEPxCTL((uint16_t)ep_num);

        ep_dir = USB_DIR_IN;
    } 
    else 
    {
        ep = &_gd_usbd.out_ep[ep_num];

        devepinten |= (1U << ep_num) << 16U;
        devepctl = USB_DOEPxCTL((uint16_t)ep_num);

        ep_dir = USB_DIR_OUT;
    }

    /* if the endpoint is not active, need change the endpoint control register */
    if (!(devepctl & DEPCTL_EPACT)) 
    {
        devepctl &= ~DEPCTL_MPL;
        devepctl |= ep_t->ep_desc->wMaxPacketSize;

        devepctl &= ~DEPCTL_EPTYPE;
        devepctl |= (uint32_t)ep_type << 18U;

        if (USB_DIR_IN == ep_dir) 
        {
            devepctl &= ~DIEPCTL_TXFNUM;
            devepctl |= (uint32_t)ep_num << 22U;
        }

        devepctl |= DEPCTL_SD0PID;
        devepctl |= DEPCTL_EPACT;
    }

    if (USB_DIR_IN == ep_dir) 
    {
        USB_DIEPxCTL((uint16_t)ep_num) = devepctl;
    } 
    else if (USB_DIR_OUT == ep_dir) 
    {
        USB_DOEPxCTL((uint16_t)ep_num) = devepctl;
    } else {
        /* no operation */
    }

    ep->endp_mps = ep_mps;
    ep->endp_type = ep_type;

    /* enable the interrupts for this endpoint */
    USB_DAEPINTEN |= devepinten;
    return RT_EOK;
}

static rt_err_t _ep_disable(uep_t ep)
{
    uint32_t devepinten = 0U;
    uint8_t ep_num = ep->ep_desc->bEndpointAddress & 0x7FU;
    
    USB_LOG("ep%02x close\r\n", ep->ep_desc->bEndpointAddress);
    
    if(ep->ep_desc->bEndpointAddress >> 7U)
    {
        devepinten |= 1U << ep_num;

        USB_DIEPxCTL((uint16_t)ep_num) &= ~DEPCTL_EPACT;
    }
    else
    {
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
    uint32_t devepctl = 0U, devepxlen = 0U;

    ep = &_gd_usbd.out_ep[ep_num];

    /* setup and start the Xfer */
    ep->xfer_buff = buffer;
    ep->xfer_len = size;
    ep->xfer_count = 0U;

    devepctl = USB_DOEPxCTL((uint16_t)ep_num);
    devepxlen = USB_DOEPxLEN((uint16_t)ep_num);

    devepxlen &= ~DEPLEN_TLEN;
    devepxlen &= ~DEPLEN_PCNT;

    /* zero length packet */
    if (0U == ep->xfer_len) 
    {
        /* set the transfer length to max packet size */
        devepxlen |= ep->endp_mps;

        /* set the transfer packet count to 1 */
        devepxlen |= 1U << 19U;
    } 
    else 
    {
        if (0U == ep_num) 
        {
            /* set the transfer length to max packet size */
            devepxlen |= ep->endp_mps;

            /* set the transfer packet count to 1 */
            devepxlen |= 1U << 19U;
        } 
        else 
        {
            /* configure the transfer size and packet count as follows:
             * pktcnt = N
             * xfersize = N * maxpacket
             */
            devepxlen |= ((ep->xfer_len + ep->endp_mps - 1U) / ep->endp_mps) << 19U;
            devepxlen |= ((devepxlen & DEPLEN_PCNT) >> 19U) * ep->endp_mps;
        }
    }

    USB_DOEPxLEN((uint16_t)ep_num) = devepxlen;

    if (USB_EP_ATTR_ISOC == ep->endp_type) 
    {
        if (ep->endp_frame) 
        {
            devepctl |= DEPCTL_SODDFRM;
        } 
        else 
        {
            devepctl |= DEPCTL_SEVNFRM;
        }
    }

    /* enable the endpoint and clear the NAK */
    devepctl |= DEPCTL_EPEN | DEPCTL_CNAK;

    USB_DOEPxCTL((uint16_t)ep_num) = devepctl;
    return size;
}

static rt_size_t _ep_write(rt_uint8_t address, void *buffer, rt_size_t size)
{
    usb_ep_struct *ep;
    uint8_t ep_num = address & 0x7FU;
    __IO uint32_t devepctl = 0U;
    __IO uint32_t deveplen = 0U;

    ep = &_gd_usbd.in_ep[ep_num];

    /* setup and start the transfer */
    ep->xfer_buff = buffer;
    ep->xfer_len = size;
    ep->xfer_count = 0U;

    devepctl = USB_DIEPxCTL((uint16_t)ep_num);
    deveplen = USB_DIEPxLEN((uint16_t)ep_num);

    /* clear transfer length to 0 */
    deveplen &= ~DEPLEN_TLEN;

    /* clear transfer packet to 0 */
    deveplen &= ~DEPLEN_PCNT;

    /* zero length packet */
    if (0U == ep->xfer_len) 
    {
        /* set transfer packet count to 1 */
        deveplen |= 1U << 19U;
    } 
    else 
    {
        if (0U == ep_num) 
        {
            if (ep->xfer_len > ep->endp_mps) 
            {
                ep->xfer_len = ep->endp_mps;
            }

            deveplen |= 1U << 19U;
        } else 
        {
            deveplen |= ((ep->xfer_len - 1U + ep->endp_mps) / ep->endp_mps) << 19U;
        }

        /* configure the transfer size and packet count as follows: 
         * xfersize = N * maxpacket + short_packet 
         * pktcnt = N + (short_packet exist ? 1 : 0)
         */
        deveplen |= ep->xfer_len;

        if (USB_EP_ATTR_ISOC == ep->endp_type) 
        {
            deveplen |= DIEPLEN_MCNT & (1U << 29U);
        }
    }

    USB_DIEPxLEN((uint16_t)ep_num) = deveplen;

    if (USB_EP_ATTR_ISOC == ep->endp_type) {
        if (0U == (((USB_DSTAT & DSTAT_FNRSOF) >> 8U) & 0x1U)) {
            devepctl |= DEPCTL_SODDFRM;
        } else {
            devepctl |= DEPCTL_SEVNFRM;
        }
    }

    /* enable the endpoint and clear the NAK */
    devepctl |= DEPCTL_EPEN | DEPCTL_CNAK;

    USB_DIEPxCTL((uint16_t)ep_num) = devepctl;

    if (USB_EP_ATTR_ISOC != ep->endp_type) 
    {
        /* enable the Tx FIFO empty interrupt for this endpoint */
        if (ep->xfer_len > 0U) {
            USB_DIEPFEINTEN |= 1U << ep_num;
        }
    } 
    else 
    {
        uint32_t count32b = ((uint16_t)ep->xfer_len + 3U) / 4U;
        __IO uint32_t *fifo = USB_FIFO(ep_num);
        uint8_t *src = ep->xfer_buff;
        for (uint16_t index = 0U; index < count32b; index++) 
        {
            *fifo = *((__packed uint32_t *)src);

            src += 4U;
        }
        
#if DRV_USB_DEBUG
                USB_LOG("EP%d in:", ep_num);
                for(uint32_t index=0; index<ep->xfer_len; index++)
                {
                    USB_LOG("0x%02x ", ep->xfer_buff[index]);
                }
                USB_LOG("\r\n");
#endif
                
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

/** Initialize the USB device controller hardware of the GD32. */
static rt_err_t _init(rt_device_t device)
{
//    SystemCoreClockUpdate();
//    switch(SystemCoreClock)
//    {
//        case 48000000: rcu_usb_clock_config(RCU_CKUSB_CKPLL_DIV1); break;
//        case 72000000: rcu_usb_clock_config(RCU_CKUSB_CKPLL_DIV1_5); break;
//        case 96000000: rcu_usb_clock_config(RCU_CKUSB_CKPLL_DIV2); break;
//        case 120000000: rcu_usb_clock_config(RCU_CKUSB_CKPLL_DIV2_5); break;
//        default: return RT_ERROR;
//    }
    rcu_periph_clock_enable(RCU_USBFS);
    
    /* delay for correct openration */
    uint32_t count = 0;
    do
    {
        count++;
    }while(count < 20000000);
    
    /* Do core reset */
    USB_GRSTCTL |= GRSTCTL_CSRST;
    while(USB_GRSTCTL &= GRSTCTL_CSRST);
    
    /* active the transceiver and enable vbus sensing */
    USB_GCCFG |= GCCFG_PWRON | GCCFG_VBUSACEN | GCCFG_VBUSBCEN;
    
    /* set tx fifo empty level to half empty mode */
    USB_GAHBCS &= ~GAHBCS_TXFTH | TXFIFO_EMPTY_HALF;
    
    /* Explicitly disable DP pullup */
    USB_SOFT_DISCONNECT_ENABLE();
    
    /* Force peripheral only mode. */
    USB_GUSBCS &= ~GUSBCS_FHM;
    USB_GUSBCS |= GUSBCS_FDM;
    
    /* restart the phy clock (maybe don't need to...) */
    USB_PWRCLKCTL = 0U;
    
    /* config periodic frmae interval to default */
    USB_DCFG &= ~DCFG_EOPFT;
    USB_DCFG |= FRAME_INTERVAL_80;
   
    /* set full speed PHY */
    USB_DCFG &= ~DCFG_DS;
    USB_DCFG |= USB_SPEED_INP_FULL;
    
    /* set rx fifo size */
    USB_GRFLEN &= ~GRFLEN_RXFD;
    USB_GRFLEN |= (uint32_t)RX_FIFO_FS_SIZE;
    
    /* set endpoint 0 tx fifo length and RAM address */
    USB_DIEP0TFLEN &= ~DIEP0TFLEN_IEP0TXFD;
    USB_DIEP0TFLEN |= (uint32_t)TX0_FIFO_FS_SIZE << 16;
    USB_DIEP0TFLEN &= ~DIEP0TFLEN_IEP0TXRSAR;
    USB_DIEP0TFLEN |= (uint32_t)RX_FIFO_FS_SIZE;
    
    uint32_t ram_address = (uint32_t)RX_FIFO_FS_SIZE;
    /* USB endpoint Tx FIFO size */
    uint16_t USBFS_TX_FIFO_SIZE[4] = 
    {
        (uint16_t)TX0_FIFO_FS_SIZE,
        (uint16_t)TX1_FIFO_FS_SIZE,
        (uint16_t)TX2_FIFO_FS_SIZE,
        (uint16_t)TX3_FIFO_FS_SIZE
    };

    /* set endpoint 1 to 3's tx fifo length and RAM address */
    for (int i = 1U; i < 4; i++) 
    {
        ram_address += USBFS_TX_FIFO_SIZE[i - 1U];

        USB_DIEPxTFLEN(i) &= ~DIEPTFLEN_IEPTXFD;
        USB_DIEPxTFLEN(i) |= (uint32_t)USBFS_TX_FIFO_SIZE[i] << 16U;
        USB_DIEPxTFLEN(i) &= ~DIEPTFLEN_IEPTXRSAR;
        USB_DIEPxTFLEN(i) |= ram_address;
    }
    
    /* flush all tx fifos */
    gd_usb_flush_txfifo(0x10U);
    
    /* flush entire rx fifo */
    gd_usb_flush_rxfifo();
    
    /* clear all pending device interrupts */
    USB_DIEPINTEN = 0U;
    USB_DOEPINTEN = 0U;
    USB_DAEPINT = 0xFFFFFFFFU;
    USB_DAEPINTEN = 0U;
    
    /* configure all in/out endpoints */
    for (uint8_t i = 0U; i < 4; i++) 
    {
        if (USB_DIEPxCTL(i) & DEPCTL_EPEN) 
        {
            USB_DIEPxCTL(i) |= DEPCTL_EPD | DEPCTL_SNAK;
        } 
        else 
        {
            USB_DIEPxCTL(i) = 0U;
        }

        if (USB_DOEPxCTL(i) & DEPCTL_EPEN) 
        {
            USB_DOEPxCTL(i) |= DEPCTL_EPD | DEPCTL_SNAK;
        } 
        else 
        {
            USB_DOEPxCTL(i) = 0U;
        }

        /* set in/out endpoint transfer length to 0 */
        USB_DIEPxLEN(i) = 0U;
        USB_DOEPxLEN(i) = 0U;

        /* clear all pending in/out endpoints interrupts */
        USB_DIEPxINTF(i) = 0xFFU;
        USB_DOEPxINTF(i) = 0xFFU;
    }
    
    uint32_t int_mask = 0U;

    /* disable all interrupts */
    USB_GINTEN = 0U;

    /* clear any pending interrupts */
    USB_GINTF = 0xBFFFFFFFU;

    /* enable the usb wakeup and suspend interrupts */
    USB_GINTEN = GINTEN_WKUPIE | GINTEN_SPIE;

    int_mask = GINTEN_RXFNEIE;

    /* enable device_mode-related interrupts */
    int_mask |= GINTEN_SPIE | GINTEN_RSTIE | GINTEN_ENUMFIE \
               | GINTEN_IEPIE | GINTEN_OEPIE | GINTEN_SOFIE | GINTEN_ISOONCIE \
               | GINTEN_ISOINCIE;
    
    USB_GINTEN &= ~int_mask;
    USB_GINTEN |= int_mask;
    
    /* set device Connect */
    USB_SOFT_DISCONNECT_DISABLE();
    
#if USB_INT
    nvic_irq_enable((uint8_t)USBFS_IRQn, 2U, 0U);
    /* enable USB global interrupt */
    USB_GLOBAL_INT_ENABLE();
#endif
    
    return RT_EOK;
}

int gd_usbd_register(void)
{
    rt_memset((void*)&_gd_udc, 0, sizeof(struct udcd));
    _gd_udc.parent.type = RT_Device_Class_USBDevice;
    _gd_udc.parent.init = _init;
    _gd_udc.parent.user_data = &_gd_usbd;
    _gd_udc.ops = &_udc_ops;
    /* Register endpoint infomation */
    _gd_udc.ep_pool = _ep_pool;
    _gd_udc.ep0.id = &_ep_pool[0];
    rt_device_register((rt_device_t)&_gd_udc, "usbd", 0);

#if USB_INT
    rt_usb_device_init();
#else
    if(rt_thread_idle_sethook(gd_usb_poll) == RT_EOK)
    {
        rt_usb_device_init();
        return RT_EOK;
    }
#endif
    
    return RT_ERROR;
}
INIT_DEVICE_EXPORT(gd_usbd_register);

#if DRV_USB_DEBUG
rt_bool_t str2hex(char* str, uint32_t* hex)
{
    if((str[0] == '0') && ((str[1] == 'x') || (str[1] == 'X')))
    {
        uint8_t len = rt_strlen(str) -2;
        for(uint8_t index=0; index<len; index++)
        {
            uint8_t bit4 = str[index + 2];
            
            if((bit4 >= '0') && (bit4 <= '9'))
            {
                bit4 -= '0';
            }
            else if((bit4 >= 'A') && (bit4 <= 'F'))
            {
                bit4 = bit4 - 'A' + 10;
            }
            else if((bit4 >= 'a') && (bit4 <= 'f'))
            {
                bit4 = bit4 - 'a' + 10;
            }
            else
            {
                return RT_FALSE;
            }
            
            *hex <<= 4;
            *hex += bit4;
        }
        return RT_TRUE;
    }
    return RT_FALSE;
}

void usb_dump_reg(int argc, char **argu)
{
    if(argc != 2)
    {
        rt_kprintf("wrong parameter, e.g.: usb_dump_reg 0x0a1f\r\n");
        return ;
    }
    else
    {
        uint32_t reg = 0;
        if(str2hex(argu[1], &reg) == RT_TRUE)
        {
            rt_kprintf("0x%08x:", reg + USBFS_BASE);
            rt_kprintf("0x%08x\r\n", *(uint32_t *)(reg + USBFS_BASE));
        }
        else
        {
            rt_kprintf("wrong parameter, e.g.: usb_dump_reg 0x0a1f\r\n");
        }
    }
}

MSH_CMD_EXPORT(usb_dump_reg, dump usb regs);
#endif

#endif

