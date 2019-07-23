#ifndef __DRV_USB_POLL_H__
#define __DRV_USB_POLL_H__
#include "gd32f10x.h"
#include "drivers/usb_common.h"

#define EP_MAX_PACKET_SIZE                      64

#define RX_FIFO_FS_SIZE                         128
#define TX0_FIFO_FS_SIZE                        64
#define TX1_FIFO_FS_SIZE                        64
#define TX2_FIFO_FS_SIZE                        32
#define TX3_FIFO_FS_SIZE                        32

#define DEP0CTL_MPL_64                          (0x0 << 0)
#define DEP0CTL_MPL_32                          (0x1 << 0)
#define DEP0CTL_MPL_16                          (0x2 << 0)
#define DEP0CTL_MPL_8                           (0x3 << 0)

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
//    uint8_t         endp_frame;                            /* USB endpoint frame */
    uint32_t        endp_mps;                              /* USB endpoint max packet size */

    /* Transaction level variables */
    uint8_t        *xfer_buff;                             /* USB transfer buffer */
    uint32_t        xfer_len;                              /* USB transfer length */
    uint32_t        xfer_count;                            /* USB transfer count */
}usb_ep_struct;

typedef struct
{
    uint8_t         set_up[3*8];
    
    usb_ep_struct   in_ep[4];
    usb_ep_struct   out_ep[4];
}gd_usb_dev;

#endif

