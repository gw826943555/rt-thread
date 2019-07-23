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
    uint32_t fifo;
    /* set IN endpoint NAK */
	USB_DIEPxCTL(addr) |= DEPCTL_SNAK;
	/* wait for core to respond */
	while (!(USB_DIEPxINTF(addr) & DIEPINTF_IEPNE)) 
    {
		/* idle */
	}
	/* get fifo for this endpoint */
	fifo = (USB_DIEPxCTL(addr) & DIEPCTL_TXFNUM) >> 22;
	/* wait for core to idle */
	;
	/* flush tx fifo */
	USB_GRSTCTL = (fifo << 6) | GRSTCTL_TXFF;
	/* reset packet counter */
	USB_DIEPxLEN(addr) = 0;
	while ((USB_GRSTCTL & GRSTCTL_TXFF)) 
    {
		/* idle */
	}
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
            USB_DOEP_INTR_READ(out_endp_intr, (uint16_t)endp_num);
            
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
                rt_usbd_ep0_setup_handler(&_gd_udc, (struct urequest *)_gd_usbd.set_up);
                USB_DOEPxINTF((uint16_t)endp_num) = DOEPINTF_STPF;
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
            if (bcount > 0U) {
                __IO uint32_t *fifo = USB_FIFO(0U);
                uint32_t count32b = (bcount +3U) /4U;
                for(uint32_t index = 0; index < count32b; index++)
                {
                    *(__packed uint32_t *)ep->xfer_buff = *fifo;
                    ep->xfer_buff += 4;
                }
                ep->xfer_count += bcount;
                
#if DRV_USB_DEBUG
                USB_LOG("EP%d out:", endp_num);
                for(uint32_t index=0; index<bcount; index++)
                {
                    USB_LOG("0x%02x ", *(uint8_t*)(ep->xfer_buff - count32b*4 + index));
                }
                USB_LOG("\r\n");
#endif
                
            }
            break;
        case RXSTAT_XFER_COMP:
            break;
        case RXSTAT_SETUP_COMP:
            break;
        case RXSTAT_SETUP_UPDT:
            *(uint32_t *)0x5000081CU |= 0x00020000U;
            if ((0U == endp_num) && (8U == bcount) && (DPID_DATA0 == data_pid)) {
                /* copy the setup packet received in fifo into the setup buffer in ram */
                __IO uint32_t *fifo = USB_FIFO(0U);
                uint8_t *dest = _gd_usbd.set_up;
                for(uint32_t index = 0; index < 2; index++)
                {
                    *(__packed uint32_t *)dest = *fifo;
                    dest += 4;
                }
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
//        rt_usbd_sof_handler(&_gd_udc);
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

static rt_err_t _ep_set_stall(rt_uint8_t address)
{
    uint8_t ep_num = address & 0x7FU;
    __IO uint32_t devepctl = 0U;
    
    USB_LOG("ep%d set stall\r\n", ep_num);

    if (address >> 7U) 
    {
        devepctl = USB_DIEPxCTL((uint16_t)ep_num);

        /* set the endpoint disable bit */
        if (devepctl & DEPCTL_EPEN) {
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
    if (0U == ep->xfer_len) {
        /* set transfer packet count to 1 */
        deveplen |= 1U << 19U;
    } else {
        if (0U == ep_num) {
            if (ep->xfer_len > ep->endp_mps) {
                ep->xfer_len = ep->endp_mps;
            }

            deveplen |= 1U << 19U;
        } else {
            deveplen |= ((ep->xfer_len - 1U + ep->endp_mps) / ep->endp_mps) << 19U;
        }

        /* configure the transfer size and packet count as follows: 
         * xfersize = N * maxpacket + short_packet 
         * pktcnt = N + (short_packet exist ? 1 : 0)
         */
        deveplen |= ep->xfer_len;

        if (USB_EP_ATTR_ISOC == ep->endp_type) {
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
    
    /* Explicitly enable DP pullup */
    USB_DCTL &= ~DCTL_SD;
    
    /* Force peripheral only mode. */
    USB_GUSBCS |= (GUSBCS_FDM | GUSBCS_UTT);
    
    USB_GINTF = GINTF_MFIF;
    
    /* Full speed device. */
    USB_DCFG |= DCFG_DS;
    
    /* Restart the PHY clock. */
    USB_PWRCLKCTL = 0;
    
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
    
    /* Unmask interrupts for TX and RX. */
    USB_GAHBCS |= GAHBCS_GINTEN;
    USB_GINTEN = GINTEN_ENUMFIE |
                GINTEN_RXFNEIE  |
                GINTEN_IEPIE |
                GINTEN_SPIE |
                GINTEN_WKUPIE |
                GINTEN_OEPIE |
                GINTEN_RSTIE;
    USB_DAEPINTEN = 0xf000f;
    USB_DIEPINTEN = DIEPINTEN_TFEN;
    
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
    
    if(rt_thread_idle_sethook(gd_usb_poll) == RT_EOK)
    {
        rt_usb_device_init();
        return RT_EOK;
    }
    
    return RT_ERROR;
}
INIT_DEVICE_EXPORT(gd_usbd_register);



#endif

