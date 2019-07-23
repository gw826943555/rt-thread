#ifndef __DRV_USBD_H__
#define __DRV_USBD_H__

#include "gd32f10x.h"
#include "drivers/usb_common.h"
#define USBFS_MAX_DEV_EPCOUNT                   4U          /* USBFS device endpoint count */

#define USB_MAX_EP0_SIZE                        64U        /* endpoint 0 max packet size */

/* 32bit words*/
#define RX_FIFO_FS_SIZE                         0x80
#define TX0_FIFO_FS_SIZE                        0x40
#define TX1_FIFO_FS_SIZE                        0x40
#define TX2_FIFO_FS_SIZE                        0x20
#define TX3_FIFO_FS_SIZE                        0x20
#define FIFO_FS_MAX_SIZE                        0x140

#if (RX_FIFO_FS_SIZE  + \
     TX0_FIFO_FS_SIZE + \
     TX1_FIFO_FS_SIZE + \
     TX2_FIFO_FS_SIZE + \
     TX3_FIFO_FS_SIZE   \
     ) > FIFO_FS_MAX_SIZE
     
#error FIFO OVERFLOW!!!

#endif

#define RXSTAT_GOUT_NAK                         1U          /* global out NAK (triggers an interrupt) */
#define RXSTAT_DATA_UPDT                        2U          /* out data packet received */
#define RXSTAT_XFER_COMP                        3U          /* out transfer completed (triggers an interrupt) */
#define RXSTAT_SETUP_COMP                       4U          /* setup transaction completed (triggers an interrupt) */
#define RXSTAT_SETUP_UPDT                       6U          /* setup data packet received */

#define DPID_DATA0                              0U          /* device endpoint data PID is DATA0 */
#define DPID_DATA1                              2U          /* device endpoint data PID is DATA1 */
#define DPID_DATA2                              1U          /* device endpoint data PID is DATA2 */
#define DPID_MDATA                              3U          /* device endpoint data PID is MDATA */

/* USB endpoint in device mode */
typedef struct
{
    uint8_t         endp_type;                             /* USB endpoint type */
    uint8_t         endp_frame;                            /* USB endpoint frame */
    uint32_t        endp_mps;                              /* USB endpoint max packet size */

    /* Transaction level variables */
    uint8_t        *xfer_buff;                             /* USB transfer buffer */
    uint32_t        xfer_len;                              /* Current transfer length */
    uint32_t        xfer_count;                            /* Partial transfer length in case of multi packet transfer */
}usb_ep_struct;

typedef struct
{
    usb_ep_struct   in_ep[USBFS_MAX_DEV_EPCOUNT];            /* USB in endpoint */
    usb_ep_struct   out_ep[USBFS_MAX_DEV_EPCOUNT];           /* USB out endpoint */
    
    uint8_t set_up[8*3];
}usb_dev_struct;

/* USB transfer direction */
typedef enum
{
    USB_RX = 0,                                            /* receive direction type value */
    USB_TX                                                 /* transmit direction type value */
}usb_dir_enum;

#endif

