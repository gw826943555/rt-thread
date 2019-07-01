#ifndef __DRV_USB_H__
#define __DRV_USB_H__

#include <rtthread.h>

#include "usbd_regs.h"

#define EP_COUNT                4

#define EP0_MAX_PACKET_SIZE     8
#define EP1_MAX_PACKET_SIZE     64
#define EP2_MAX_PACKET_SIZE     64
#define EP3_MAX_PACKET_SIZE     64
//#define EP4_MAX_PACKET_SIZE     8
//#define EP5_MAX_PACKET_SIZE     8
//#define EP6_MAX_PACKET_SIZE     8
//#define EP7_MAX_PACKET_SIZE     8

typedef struct
{
    uint16_t tx_addr;    /*!< transmission address */
    uint16_t reserved0;
    uint16_t tx_count;   /*!< transmission count */
    uint16_t reserved1;
    uint16_t rx_addr;    /*!< reception address */
    uint16_t reserved2;
    uint16_t rx_count;   /*!< reception count */
    uint16_t reserved3;
}usbd_ep_buf_struct;

typedef struct
{
    /* basic parameters */
    uint8_t        stall;               /*!< endpoint stall status */
    uint32_t       maxpacket;           /*!< the maxpacket of the endpoint */

    /* transaction level parameters */
    uint8_t       *trs_buf;             /*!< transaction buffer address */
    uint32_t       trs_len;             /*!< transaction buffer length */
    uint32_t       trs_count;           /*!< transaction data counts */
}usb_ep_struct;

/* USB device status */
typedef enum
{
    USBD_UNCONNECTED = 0, /*!< USB device unconnected status */
    USBD_DEFAULT,         /*!< USB device default status */
    USBD_ADDRESSED,       /*!< USB device addressed status */
    USBD_CONFIGURED,      /*!< USB device configured status */
    USBD_SUSPENDED,       /*!< USB device suspended status */
    USBD_CONNECTED        /*!< USB device connected status */
}usbd_run_status_enum;

/* USB core driver struct */
typedef struct
{
    /* basic parameters */
    uint8_t  config_num;                /*!< the number of the USB device configuration */
    __IO uint8_t  status;               /*!< USB device status */
    uint8_t  prev_status;               /*!< the previous USB device status */
    uint8_t  remote_wakeup;             /*!< the flag that point out the device whether support the remote wakeup function */

    /* the parameters which needs in control transfer */
    uint8_t  setup_packet[8];           /*!< the buffer used to store the setup packet */
    uint32_t ctl_count;                 /*!< the datas length of control transfer request */

    /* device endpoints */
    usb_ep_struct in_ep[EP_COUNT];      /*!< the in direction endpoints */
    usb_ep_struct out_ep[EP_COUNT];     /*!< the out direction endpoints */

#ifdef LPM_ENABLED
    uint8_t *bos_desc;                  /*!< BOS descriptor */
#endif /* LPM_ENABLED */

    uint8_t *dev_desc;                  /*!< device descriptor */
    uint8_t *config_desc;               /*!< configuration descriptor */
    void* const *strings;               /*!< configuration strings */

}usbd_core_handle_struct;


#endif

