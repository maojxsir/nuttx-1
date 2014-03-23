/****************************************************************************
 * drivers/usbdev/USBTMC.c
 *
 *   Copyright (C) 2008-2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This logic emulates the Prolific USBTMC serial/USB converter
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include <queue.h>
#include <debug.h>
#include <pthread.h>

#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

#include <ficl.h>
#include "usbtmc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Number of requests in the write queue */

#ifndef CONFIG_USBTMC_NWRREQS
#  define CONFIG_USBTMC_NWRREQS 4
#endif

/* Number of requests in the read queue */

#ifndef CONFIG_USBTMC_NRDREQS
#  define CONFIG_USBTMC_NRDREQS 4
#endif

/* Logical endpoint numbers / max packet sizes */

#ifndef CONFIG_USBTMC_EPINTIN
#  warning "EPINTIN not defined in the configuration"
#  define CONFIG_USBTMC_EPINTIN 1
#endif

#ifndef CONFIG_USBTMC_EPBULKOUT
#  warning "EPBULKOUT not defined in the configuration"
#  define CONFIG_USBTMC_EPBULKOUT 2
#endif

#ifndef CONFIG_USBTMC_EPBULKIN
#  warning "EPBULKIN not defined in the configuration"
#  define CONFIG_USBTMC_EPBULKIN 3
#endif

/* Packet and request buffer sizes */

#ifndef CONFIG_USBTMC_EP0MAXPACKET
#  define CONFIG_USBTMC_EP0MAXPACKET 64
#endif

#ifndef CONFIG_USBTMC_BUFFCOUNT
 #  define CONFIG_USBTMC_BUFFCOUNT 4
#endif

/* Ideally, the BULKOUT request size should *not* be the same size as the
 * maxpacket size.  That is because IN transfers of exactly the maxpacket
 * size will be followed by a NULL packet.  The BULKOUT request buffer
 * size, on the other hand, is always the same as the maxpacket size.
 */

#define USBTMC_EP_MXPACKET         (64)

#ifndef CONFIG_USBTMC_BULKIN_REQLEN
#  define CONFIG_USBTMC_BULKIN_REQLEN   USBTMC_EP_MXPACKET
#endif

/* Vendor and product IDs and strings */

#ifndef CONFIG_USBTMC_VENDORID
#  define CONFIG_USBTMC_VENDORID  0x0483    // Use STMicroelectronics VID
#endif

#ifndef CONFIG_USBTMC_PRODUCTID
#  define CONFIG_USBTMC_PRODUCTID 0x6868   
#endif

static uint16_t str_vendor[] = {0x5c0f,0x7518,0x5de5,0x4f5c,0x5ba4};
#define CONFIG_USBTMC_VENDORSTR  str_vendor

static uint16_t str_product[] = {0x5c0f,0x7518,0x7535,0x6e90};
#define CONFIG_USBTMC_PRODUCTSTR str_product

static uint16_t str_serial[] = {0x0000};
#undef CONFIG_USBTMC_SERIALSTR
#define CONFIG_USBTMC_SERIALSTR str_serial

static uint16_t str_config[] = {0x6d4b,0x8bd5,0x6d4b,0x91cf,0x4eea,0x5668};
#undef CONFIG_USBTMC_CONFIGSTR
#define CONFIG_USBTMC_CONFIGSTR str_config

/* USB Controller */

#ifdef CONFIG_USBDEV_SELFPOWERED
#  define USBTMC_SELFPOWERED USB_CONFIG_ATTR_SELFPOWER
#else
#  define USBTMC_SELFPOWERED (0)
#endif

#ifdef CONFIG_USBDEV_REMOTEWAKEUP
#  define USBTMC_REMOTEWAKEUP USB_CONFIG_ATTR_WAKEUP
#else
#  define USBTMC_REMOTEWAKEUP (0)
#endif

#ifndef CONFIG_USBDEV_MAXPOWER
#  define CONFIG_USBDEV_MAXPOWER 100
#endif

/* Descriptors ****************************************************************/

/* These settings are not modifiable via the NuttX configuration */

#define USBTMC_VERSIONNO           (0x0202) /* Device version number */
#define USBTMC_CONFIGIDNONE        (0)      /* Config ID means to return to address mode */
#define USBTMC_CONFIGID            (1)      /* The only supported configuration ID */
#define USBTMC_NCONFIGS            (1)      /* Number of configurations supported */
#define USBTMC_INTERFACEID         (0)
#define USBTMC_ALTINTERFACEID      (0)
#define USBTMC_NINTERFACES         (1)      /* Number of interfaces in the configuration */
#define USBTMC_NENDPOINTS          (3)      /* Number of endpoints in the interface  */

/* Endpoint configuration */

#define USBTMC_EPINTIN_ADDR        (USB_DIR_IN|CONFIG_USBTMC_EPINTIN)
#define USBTMC_EPINTIN_ATTR        (USB_EP_ATTR_XFER_INT)
#define USBTMC_EPINTIN_MXPACKET    (10)

#define USBTMC_EPOUTBULK_ADDR      (CONFIG_USBTMC_EPBULKOUT)
#define USBTMC_EPOUTBULK_ATTR      (USB_EP_ATTR_XFER_BULK)

#define USBTMC_EPINBULK_ADDR       (USB_DIR_IN|CONFIG_USBTMC_EPBULKIN)
#define USBTMC_EPINBULK_ATTR       (USB_EP_ATTR_XFER_BULK)

/* String language */

 #define USBTMC_STR_LANGUAGE        (0x0804)  /* chinese  */

/* Descriptor strings */

#define USBTMC_MANUFACTURERSTRID   (1)
#define USBTMC_PRODUCTSTRID        (2)
#define USBTMC_SERIALSTRID         (3)
#define USBTMC_CONFIGSTRID         (4)

/* Buffer big enough for any of our descriptors */
#define USBTMC_MXDESCLEN           (64)

/* Vender specific control requests *******************************************/

// TODO:Add USBTMC command

/* Misc Macros ****************************************************************/

/* min/max macros */

#ifndef min
#  define min(a,b) ((a)<(b)?(a):(b))
#endif

#ifndef max
#  define max(a,b) ((a)>(b)?(a):(b))
#endif

#define USBTMC_FICL_CMD_SIZE (256)

/* Trace values *************************************************************/

#define USBTMC_CLASSAPI_SETUP       TRACE_EVENT(TRACE_CLASSAPI_ID, USBTMC_TRACECLASSAPI_SETUP)
#define USBTMC_CLASSAPI_SHUTDOWN    TRACE_EVENT(TRACE_CLASSAPI_ID, USBTMC_TRACECLASSAPI_SHUTDOWN)
#define USBTMC_CLASSAPI_ATTACH      TRACE_EVENT(TRACE_CLASSAPI_ID, USBTMC_TRACECLASSAPI_ATTACH)
#define USBTMC_CLASSAPI_DETACH      TRACE_EVENT(TRACE_CLASSAPI_ID, USBTMC_TRACECLASSAPI_DETACH)
#define USBTMC_CLASSAPI_IOCTL       TRACE_EVENT(TRACE_CLASSAPI_ID, USBTMC_TRACECLASSAPI_IOCTL)
#define USBTMC_CLASSAPI_RECEIVE     TRACE_EVENT(TRACE_CLASSAPI_ID, USBTMC_TRACECLASSAPI_RECEIVE)
#define USBTMC_CLASSAPI_RXINT       TRACE_EVENT(TRACE_CLASSAPI_ID, USBTMC_TRACECLASSAPI_RXINT)
#define USBTMC_CLASSAPI_RXAVAILABLE TRACE_EVENT(TRACE_CLASSAPI_ID, USBTMC_TRACECLASSAPI_RXAVAILABLE)
#define USBTMC_CLASSAPI_SEND        TRACE_EVENT(TRACE_CLASSAPI_ID, USBTMC_TRACECLASSAPI_SEND)
#define USBTMC_CLASSAPI_TXINT       TRACE_EVENT(TRACE_CLASSAPI_ID, USBTMC_TRACECLASSAPI_TXINT)
#define USBTMC_CLASSAPI_TXREADY     TRACE_EVENT(TRACE_CLASSAPI_ID, USBTMC_TRACECLASSAPI_TXREADY)
#define USBTMC_CLASSAPI_TXEMPTY     TRACE_EVENT(TRACE_CLASSAPI_ID, USBTMC_TRACECLASSAPI_TXEMPTY)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Container to support a list of requests */

struct usbtmc_req_s
{
    FAR struct usbtmc_req_s *flink;     /* Implements a singly linked list */
    FAR struct usbdev_req_s *req;       /* The contained request */
};
struct usbtmc_fifo_s
{
  uint8_t head;
  uint8_t tail;
  uint8_t used;
  pthread_mutex_t mutex;
  char buff[CONFIG_USBTMC_BUFFCOUNT][USBTMC_EP_MXPACKET];
};
/* This structure describes the internal state of the driver */

struct usbtmc_dev_s
{
    FAR struct usbdev_s     *usbdev;    /* usbdev driver pointer */
    
    uint8_t config;                     /* Configuration number */
    uint8_t nwrq;                       /* Number of queue write requests (in reqlist)*/
    uint8_t nrdq;                       /* Number of queue read requests (in epbulkout) */
    int16_t rxhead;                     /* Working head; used when rx int disabled */
    bool bTransferBegin;
    uint32_t currTransSize;             /* Current Transfer Size*/
    uint32_t currRecvSize;
    uint32_t currSendSize;
    uint8_t  currTag;
    ficlVm *ficl_vm;                    /* Ficl Virtual Machine*/
    ficlSystem *ficl_system;            /* Ficl */
    sem_t read_sem;                     /* indicate decode thread to decode usbtmc message*/
    sem_t write_sem;                    /* indicate decode thread to send data to host*/
    pthread_t decodethread;
    FAR struct usbdev_ep_s  *epintin;   /* Interrupt IN endpoint structure */
    FAR struct usbdev_ep_s  *epbulkin;  /* Bulk IN endpoint structure */
    FAR struct usbdev_ep_s  *epbulkout; /* Bulk OUT endpoint structure */
    FAR struct usbdev_req_s *ctrlreq;   /* Control request */
    struct sq_queue_s        reqlist;   /* List of write request containers */
    
    /* Pre-allocated write request containers.  The write requests will
     * be linked in a free list (reqlist), and used to send requests to
     * EPBULKIN; Read requests will be queued in the EBULKOUT.
     */
    
    struct usbtmc_req_s wrreqs[CONFIG_USBTMC_NWRREQS];
    struct usbtmc_req_s rdreqs[CONFIG_USBTMC_NWRREQS];
    
    /* I/O buffers */
    
    struct usbtmc_fifo_s rxbuffer;
    struct usbtmc_fifo_s txbuffer;
};

/* The internal version of the class driver */

struct usbtmc_driver_s
{
    struct usbdevclass_driver_s drvr;
    FAR struct usbtmc_dev_s     *dev;
};

/* This is what is allocated */

struct usbtmc_alloc_s
{
    struct usbtmc_dev_s    dev;
    struct usbtmc_driver_s drvr;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Transfer helpers *********************************************************/

static uint16_t usbclass_fillrequest(FAR struct usbtmc_dev_s *priv,
                                     uint8_t *reqbuf, uint16_t reqlen);
static int     usbclass_sndpacket(FAR struct usbtmc_dev_s *priv);
static inline int usbclass_recvpacket(FAR struct usbtmc_dev_s *priv,
                                      uint8_t *reqbuf, uint16_t reqlen);

/* Request helpers *********************************************************/

static struct  usbdev_req_s *usbclass_allocreq(FAR struct usbdev_ep_s *ep,
                                               uint16_t len);
static void    usbclass_freereq(FAR struct usbdev_ep_s *ep,
                                FAR struct usbdev_req_s *req);

/* Configuration ***********************************************************/

static int     usbclass_mkstrdesc(uint8_t id, struct usb_strdesc_s *strdesc);
#ifdef CONFIG_USBDEV_DUALSPEED
static void    usbclass_mkepbulkdesc(const struct usb_epdesc_s *indesc,
                                     uint16_t mxpacket, struct usb_epdesc_s *outdesc);
static int16_t usbclass_mkcfgdesc(uint8_t *buf, uint8_t speed, uint8_t type);
#else
static int16_t usbclass_mkcfgdesc(uint8_t *buf);
#endif
static void    usbclass_resetconfig(FAR struct usbtmc_dev_s *priv);
static int     usbclass_setconfig(FAR struct usbtmc_dev_s *priv,
                                  uint8_t config);

/* Completion event handlers ***********************************************/

static void    usbclass_ep0incomplete(FAR struct usbdev_ep_s *ep,
                                      FAR struct usbdev_req_s *req);
static void    usbclass_rdcomplete(FAR struct usbdev_ep_s *ep,
                                   FAR struct usbdev_req_s *req);
static void    usbclass_wrcomplete(FAR struct usbdev_ep_s *ep,
                                   FAR struct usbdev_req_s *req);

/* USB class device ********************************************************/

static int     usbclass_bind(FAR struct usbdevclass_driver_s *driver,
                             FAR struct usbdev_s *dev);
static void    usbclass_unbind(FAR struct usbdevclass_driver_s *driver,
                               FAR struct usbdev_s *dev);
static int     usbclass_setup(FAR struct usbdevclass_driver_s *driver,
                              FAR struct usbdev_s *dev,
                              FAR const struct usb_ctrlreq_s *ctrl, FAR uint8_t *dataout,
                              size_t outlen);
static void    usbclass_disconnect(FAR struct usbdevclass_driver_s *driver,
                                   FAR struct usbdev_s *dev);
#if 0
static void    usbclass_suspend(FAR struct usbdevclass_driver_s *driver,
                                FAR struct usbdev_s *dev);
static void    usbclass_resume(FAR struct usbdevclass_driver_s *driver,
                               FAR struct usbdev_s *dev);
#endif

void  usbtmc_ficlTextOut(ficlCallback *callback, char *message);
void *usbtmc_decode_thread(void *arg);

extern uint8_t g_adc1_dmabuff[];
extern uint8_t g_DAC1DMABuffer[];

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/* USB class device ********************************************************/

static const struct usbdevclass_driverops_s g_driverops =
{
    usbclass_bind,        /* bind */
    usbclass_unbind,      /* unbind */
    usbclass_setup,       /* setup */
    usbclass_disconnect,  /* disconnect */
    #if 0
    usbclass_suspend,     /* suspend */
    usbclass_resume,      /* resume */
    #else
    NULL,                 /* suspend */
    NULL,                 /* resume */
    #endif
};

/* USB descriptor templates these will be copied and modified **************/

static const struct usb_devdesc_s g_devdesc =
{
    USB_SIZEOF_DEVDESC,                           /* len */
    USB_DESC_TYPE_DEVICE,                         /* type */
    {LSBYTE(0x0200), MSBYTE(0x0200)},             /* usb */
    0,                                            /* classid */
    0,                                            /* subclass */
    0,                                            /* protocol */
    CONFIG_USBTMC_EP0MAXPACKET,                   /* maxpacketsize */
    { LSBYTE(CONFIG_USBTMC_VENDORID),             /* vendor */
        MSBYTE(CONFIG_USBTMC_VENDORID) },
        { LSBYTE(CONFIG_USBTMC_PRODUCTID),            /* product */
            MSBYTE(CONFIG_USBTMC_PRODUCTID) },
            { LSBYTE(USBTMC_VERSIONNO),                   /* device */
                MSBYTE(USBTMC_VERSIONNO) },
                USBTMC_MANUFACTURERSTRID,                     /* imfgr */
                USBTMC_PRODUCTSTRID,                          /* iproduct */
                USBTMC_SERIALSTRID,                           /* serno */
                USBTMC_NCONFIGS                               /* nconfigs */
};

static const struct usb_cfgdesc_s g_cfgdesc =
{
    USB_SIZEOF_CFGDESC,                           /* len */
    USB_DESC_TYPE_CONFIG,                         /* type */
    {0, 0},                                       /* totallen -- to be provided */
    USBTMC_NINTERFACES,                           /* ninterfaces */
    USBTMC_CONFIGID,                              /* cfgvalue */
    USBTMC_CONFIGSTRID,                           /* icfg */
    USB_CONFIG_ATTR_ONE |                         /* attr */
    USBTMC_SELFPOWERED |
    USBTMC_REMOTEWAKEUP,
    (CONFIG_USBDEV_MAXPOWER + 1) / 2              /* mxpower */
};

static const struct usb_ifdesc_s g_ifdesc =
{
    USB_SIZEOF_IFDESC,                            /* len */
    USB_DESC_TYPE_INTERFACE,                      /* type */
    0,                                            /* ifno */
    0,                                            /* alt */
    USBTMC_NENDPOINTS,                            /* neps */
    USB_CLASS_APP_SPEC,                           /* classid */
    3,                                            /* subclass */
    0,                                            /* protocol */
    USBTMC_CONFIGSTRID                            /* iif */
};

static const struct usb_epdesc_s g_epintindesc =
{
    USB_SIZEOF_EPDESC,                            /* len */
    USB_DESC_TYPE_ENDPOINT,                       /* type */
    USBTMC_EPINTIN_ADDR,                          /* addr */
    USBTMC_EPINTIN_ATTR,                          /* attr */
    { LSBYTE(USBTMC_EPINTIN_MXPACKET),            /* maxpacket */
        MSBYTE(USBTMC_EPINTIN_MXPACKET) },
        1                                             /* interval */
};

static const struct usb_epdesc_s g_epbulkoutdesc =
{
    USB_SIZEOF_EPDESC,                            /* len */
    USB_DESC_TYPE_ENDPOINT,                       /* type */
    USBTMC_EPOUTBULK_ADDR,                        /* addr */
    USBTMC_EPOUTBULK_ATTR,                        /* attr */
    { LSBYTE(USBTMC_EP_MXPACKET), MSBYTE(USBTMC_EP_MXPACKET) },/* maxpacket -- might change to 512 */
    0                                             /* interval */
};

static const struct usb_epdesc_s g_epbulkindesc =
{
    USB_SIZEOF_EPDESC,                            /* len */
    USB_DESC_TYPE_ENDPOINT,                       /* type */
    USBTMC_EPINBULK_ADDR,                         /* addr */
    USBTMC_EPINBULK_ATTR,                         /* attr */
    { LSBYTE(USBTMC_EP_MXPACKET), MSBYTE(USBTMC_EP_MXPACKET) },    /* maxpacket -- might change to 512 */
    0                                             /* interval */
};

#ifdef CONFIG_USBDEV_DUALSPEED
static const struct usb_qualdesc_s g_qualdesc =
{
    USB_SIZEOF_QUALDESC,                          /* len */
    USB_DESC_TYPE_DEVICEQUALIFIER,                /* type */
    {LSBYTE(0x0200), MSBYTE(0x0200) },            /* USB */
    0,                                            /* classid */
    0,                                            /* subclass */
    0,                                            /* protocol */
    CONFIG_USBTMC_EP0MAXPACKET,                   /* mxpacketsize */
    USBTMC_NCONFIGS,                              /* nconfigs */
    0,                                            /* reserved */
};
#endif
static struct usbtmc_capability_s g_cap = {
  .status = STATUS_SUCCESS,
  .bcd_ver = 0x0100,
  .interface_cap = 0,
  .device_cap = 0,
};
/****************************************************************************
 * Private Functions
 ****************************************************************************/

/************************************************************************************
 * Name: usbclass_fillrequest
 *
 * Description:
 *   If there is data to send it is copied to the given buffer.  Called either
 *   to initiate the first write operation, or from the completion interrupt handler
 *   service consecutive write operations.
 *
 * NOTE: The USB serial driver does not use the serial drivers uart_xmitchars()
 *   API.  That logic is essentially duplicated here because unlike UART hardware,
 *   we need to be able to handle writes not byte-by-byte, but packet-by-packet.
 *   Unfortunately, that decision also exposes some internals of the serial driver
 *   in the following.
 *
 ************************************************************************************/

static uint16_t usbclass_fillrequest(FAR struct usbtmc_dev_s *priv, uint8_t *reqbuf,
                                     uint16_t reqlen)
{
  if (reqlen % 4) {
    usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_INVALIDARG),0);
    return -EINVAL;
  }

  if (priv->txbuffer.used == 0) {
    return 0;
  }

  memcpy(reqbuf,&priv->txbuffer.buff[priv->txbuffer.head],reqlen);
  pthread_mutex_lock(&priv->txbuffer.mutex);
  priv->txbuffer.used--;
  priv->txbuffer.head = (priv->txbuffer.head+1) % CONFIG_USBTMC_BUFFCOUNT;
  pthread_mutex_unlock(&priv->txbuffer.mutex);
  sem_post(&priv->write_sem);
  return reqlen;
}

/************************************************************************************
 * Name: usbclass_sndpacket
 *
 * Description:
 *   This function obtains write requests, transfers the TX data into the request,
 *   and submits the requests to the USB controller.  This continues untils either
 *   (1) there are no further packets available, or (2) thre is not further data
 *   to send.
 *
 ************************************************************************************/

static int usbclass_sndpacket(FAR struct usbtmc_dev_s *priv)
{

  FAR struct usbdev_ep_s *ep;
  FAR struct usbdev_req_s *req;
  FAR struct usbtmc_req_s *reqcontainer;
  uint16_t reqlen;
  irqstate_t flags;
  int len;
  int ret = OK;

#ifdef CONFIG_DEBUG
  if (priv == NULL)
    {
      usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_INVALIDARG), 0);
      return -ENODEV;
    }
#endif

  flags = irqsave();

  /* Use our IN endpoint for the transfer */

  ep = priv->epbulkin;

  /* Loop until either (1) we run out or write requests, or (2) usbclass_fillrequest()
   * is unable to fill the request with data (i.e., until there is no more data
   * to be sent).
   */

  /* Get the maximum number of bytes that will fit into one bulk IN request */

  //reqlen = max(CONFIG_PL2303_BULKIN_REQLEN, ep->maxpacket);
  reqlen = ep->maxpacket;

  while (!sq_empty(&priv->reqlist))
    {
      /* Peek at the request in the container at the head of the list */

      reqcontainer = (struct usbtmc_req_s *)sq_peek(&priv->reqlist);
      req          = reqcontainer->req;

      /* Fill the request with serial TX data */

      len = usbclass_fillrequest(priv, req->buf, reqlen);
      if (len > 0)
        {
          /* Remove the empty container from the request list */

          (void)sq_remfirst(&priv->reqlist);
          priv->nwrq--;

          /* Then submit the request to the endpoint */

          req->len     = len;
          req->priv    = reqcontainer;
          req->flags   = USBDEV_REQFLAGS_NULLPKT;
          ret          = EP_SUBMIT(ep, req);
          if (ret != OK)
            {
              usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_SUBMITFAIL), (uint16_t)-ret);
              break;
            }
        }
      else
        {
          break;
        }
    }

  irqrestore(flags);
  return ret;
}

/************************************************************************************
 * Name: usbclass_recvpacket
 *
 * Description:
 *   A normal completion event was received by the read completion handler at the
 *   interrupt level (with interrupts disabled).  This function handles the USB packet
 *   and provides the received data to the uart RX buffer.
 *
 * Assumptions:
 *   Called from the USB interrupt handler with interrupts disabled.
 *
 ************************************************************************************/

static inline int usbclass_recvpacket(FAR struct usbtmc_dev_s *priv,
                                      uint8_t *reqbuf, uint16_t reqlen)
{

  if(reqlen % 4 || reqlen > USBTMC_EP_MXPACKET) {
    usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_INVALIDARG),EINVAL);
    return -EINVAL;
  }
  if (priv->rxbuffer.used == CONFIG_USBTMC_BUFFCOUNT) {
    usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_RXOVERRUN),0);
    return -EPERM;
  }
  

  memcpy(&priv->rxbuffer.buff[priv->rxbuffer.tail], reqbuf, reqlen);
  pthread_mutex_lock(&priv->rxbuffer.mutex);
  priv->rxbuffer.used++;
  priv->rxbuffer.tail = (priv->rxbuffer.tail+1) % CONFIG_USBTMC_BUFFCOUNT;
  pthread_mutex_unlock(&priv->rxbuffer.mutex);
  sem_post(&priv->read_sem);

  return 0;
}

/****************************************************************************
 * Name: usbclass_allocreq
 *
 * Description:
 *   Allocate a request instance along with its buffer
 *
 ****************************************************************************/

static struct usbdev_req_s *usbclass_allocreq(FAR struct usbdev_ep_s *ep,
                                              uint16_t len)
{
    FAR struct usbdev_req_s *req;
    
    req = EP_ALLOCREQ(ep);
    if (req != NULL)
    {
        req->len = len;
        req->buf = EP_ALLOCBUFFER(ep, len);
        if (!req->buf)
        {
            EP_FREEREQ(ep, req);
            req = NULL;
        }
    }
    return req;
}

/****************************************************************************
 * Name: usbclass_freereq
 *
 * Description:
 *   Free a request instance along with its buffer
 *
 ****************************************************************************/

static void usbclass_freereq(FAR struct usbdev_ep_s *ep,
                             FAR struct usbdev_req_s *req)
{
    if (ep != NULL && req != NULL)
    {
        if (req->buf != NULL)
        {
            EP_FREEBUFFER(ep, req->buf);
        }
        EP_FREEREQ(ep, req);
    }
}

/****************************************************************************
 * Name: usbclass_mkstrdesc
 *
 * Description:
 *   Construct a string descriptor
 *
 ****************************************************************************/

static int usbclass_mkstrdesc(uint8_t id, struct usb_strdesc_s *strdesc)
{
    const char *str;
    int len;
    int ndata;
    int i;
    
    switch (id)
    {
        case 0:
        {
            /* Descriptor 0 is the language id */
            
            strdesc->len     = 4;
            strdesc->type    = USB_DESC_TYPE_STRING;
            strdesc->data[0] = LSBYTE(USBTMC_STR_LANGUAGE);
            strdesc->data[1] = MSBYTE(USBTMC_STR_LANGUAGE);
            return 4;
        }
        
        case USBTMC_MANUFACTURERSTRID:
            str = CONFIG_USBTMC_VENDORSTR;
            break;
            
        case USBTMC_PRODUCTSTRID:
            str = CONFIG_USBTMC_PRODUCTSTR;
            break;
            
        case USBTMC_SERIALSTRID:
            str = CONFIG_USBTMC_SERIALSTR;
            break;
            
        case USBTMC_CONFIGSTRID:
            str = CONFIG_USBTMC_CONFIGSTR;
            break;
            
        default:
            return -EINVAL;
    }
    
    /* The string is utf16-le.  The poor man's utf-8 to utf16-le
     * conversion below will only handle 7-bit en-us ascii
     */
    //TODO, USE UTF16 chinese desc
#if 0
    len = strlen(str);
    for (i = 0, ndata = 0; i < len; i++, ndata += 2)
    {
        strdesc->data[ndata]   = str[i];
        strdesc->data[ndata+1] = 0;
    }
#endif
    
    strdesc->len  = sizeof(str);
    strdesc->type = USB_DESC_TYPE_STRING;
    return strdesc->len;
}

/****************************************************************************
 * Name: usbclass_mkepbulkdesc
 *
 * Description:
 *   Construct the endpoint descriptor
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DUALSPEED
static inline void usbclass_mkepbulkdesc(const FAR struct usb_epdesc_s *indesc,
                                         uint16_t mxpacket,
                                         FAR struct usb_epdesc_s *outdesc)
{
    /* Copy the canned descriptor */
    
    memcpy(outdesc, indesc, USB_SIZEOF_EPDESC);
    
    /* Then add the correct max packet size */
    
    outdesc->mxpacketsize[0] = LSBYTE(mxpacket);
    outdesc->mxpacketsize[1] = MSBYTE(mxpacket);
}
#endif

/****************************************************************************
 * Name: usbclass_mkcfgdesc
 *
 * Description:
 *   Construct the configuration descriptor
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DUALSPEED
static int16_t usbclass_mkcfgdesc(uint8_t *buf, uint8_t speed, uint8_t type)
#else
static int16_t usbclass_mkcfgdesc(uint8_t *buf)
#endif
{
    FAR struct usb_cfgdesc_s *cfgdesc = (struct usb_cfgdesc_s*)buf;
    #ifdef CONFIG_USBDEV_DUALSPEED
    bool hispeed = (speed == USB_SPEED_HIGH);
    uint16_t bulkmxpacket;
    #endif
    uint16_t totallen;
    
    /* This is the total length of the configuration (not necessarily the
     * size that we will be sending now.
     */
    
    totallen = USB_SIZEOF_CFGDESC + USB_SIZEOF_IFDESC + USBTMC_NENDPOINTS * USB_SIZEOF_EPDESC;
    
    /* Configuration descriptor -- Copy the canned descriptor and fill in the
     * type (we'll also need to update the size below
     */
    
    memcpy(cfgdesc, &g_cfgdesc, USB_SIZEOF_CFGDESC);
    buf += USB_SIZEOF_CFGDESC;
    
    /*  Copy the canned interface descriptor */
    
    memcpy(buf, &g_ifdesc, USB_SIZEOF_IFDESC);
    buf += USB_SIZEOF_IFDESC;
    
    /* Make the three endpoint configurations.  First, check for switches
     * between high and full speed
     */
    
    #ifdef CONFIG_USBDEV_DUALSPEED
    if (type == USB_DESC_TYPE_OTHERSPEEDCONFIG)
    {
        hispeed = !hispeed;
    }
    #endif
    
    memcpy(buf, &g_epintindesc, USB_SIZEOF_EPDESC);
    buf += USB_SIZEOF_EPDESC;
    
    #ifdef CONFIG_USBDEV_DUALSPEED
    if (hispeed)
    {
        bulkmxpacket = 512;
    }
    else
    {
        bulkmxpacket = 64;
    }
    
    usbclass_mkepbulkdesc(&g_epbulkoutdesc, bulkmxpacket, (struct usb_epdesc_s*)buf);
    buf += USB_SIZEOF_EPDESC;
    usbclass_mkepbulkdesc(&g_epbulkindesc, bulkmxpacket, (struct usb_epdesc_s*)buf);
    #else
    memcpy(buf, &g_epbulkoutdesc, USB_SIZEOF_EPDESC);
    buf += USB_SIZEOF_EPDESC;
    memcpy(buf, &g_epbulkindesc, USB_SIZEOF_EPDESC);
    #endif
    
    /* Finally, fill in the total size of the configuration descriptor */
    
    cfgdesc->totallen[0] = LSBYTE(totallen);
    cfgdesc->totallen[1] = MSBYTE(totallen);
    return totallen;
}

/****************************************************************************
 * Name: usbclass_resetconfig
 *
 * Description:
 *   Mark the device as not configured and disable all endpoints.
 *
 ****************************************************************************/

static void usbclass_resetconfig(FAR struct usbtmc_dev_s *priv)
{
    /* Are we configured? */
    
    if (priv->config != USBTMC_CONFIGIDNONE)
    {
        /* Yes.. but not anymore */
        
        priv->config = USBTMC_CONFIGIDNONE;
        
        /* Inform the "upper half" driver that there is no (functional) USB
         * connection.
         */

        
        /* Disable endpoints.  This should force completion of all pending
         * transfers.
         */
        
        EP_DISABLE(priv->epintin);
        EP_DISABLE(priv->epbulkin);
        EP_DISABLE(priv->epbulkout);
    }
}

/****************************************************************************
 * Name: usbclass_setconfig
 *
 * Description:
 *   Set the device configuration by allocating and configuring endpoints and
 *   by allocating and queue read and write requests.
 *
 ****************************************************************************/

static int usbclass_setconfig(FAR struct usbtmc_dev_s *priv, uint8_t config)
{
    FAR struct usbdev_req_s *req;
    #ifdef CONFIG_USBDEV_DUALSPEED
    struct usb_epdesc_s epdesc;
    uint16_t bulkmxpacket;
    #endif
    int i;
    int ret = 0;
    
    #if CONFIG_DEBUG
    if (priv == NULL)
    {
        usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_INVALIDARG), 0);
        return -EIO;
    }
    #endif
    
    if (config == priv->config)
    {
        /* Already configured -- Do nothing */
        
        usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_ALREADYCONFIGURED), 0);
        return 0;
    }
    
    /* Discard the previous configuration data */
    
    usbclass_resetconfig(priv);
    
    /* Was this a request to simply discard the current configuration? */
    
    if (config == USBTMC_CONFIGIDNONE)
    {
        usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_CONFIGNONE), 0);
        return 0;
    }
    
    /* We only accept one configuration */
    
    if (config != USBTMC_CONFIGID)
    {
        usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_CONFIGIDBAD), 0);
        return -EINVAL;
    }
    
    /* Configure the IN interrupt endpoint */
    
    ret = EP_CONFIGURE(priv->epintin, &g_epintindesc, false);
    if (ret < 0)
    {
        usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_EPINTINCONFIGFAIL), 0);
        goto errout;
    }
    priv->epintin->priv = priv;
    
    /* Configure the IN bulk endpoint */
    
    #ifdef CONFIG_USBDEV_DUALSPEED
    if (priv->usbdev->speed == USB_SPEED_HIGH)
    {
        bulkmxpacket = 512;
    }
    else
    {
        bulkmxpacket = 64;
    }
    
    usbclass_mkepbulkdesc(&g_epbulkindesc, bulkmxpacket, &epdesc);
    ret = EP_CONFIGURE(priv->epbulkin, &epdesc, false);
    #else
    ret = EP_CONFIGURE(priv->epbulkin, &g_epbulkindesc, false);
    #endif
    if (ret < 0)
    {
        usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_EPBULKINCONFIGFAIL), 0);
        goto errout;
    }
    
    priv->epbulkin->priv = priv;
    
    /* Configure the OUT bulk endpoint */
    
    #ifdef CONFIG_USBDEV_DUALSPEED
    usbclass_mkepbulkdesc(&g_epbulkoutdesc, bulkmxpacket, &epdesc);
    ret = EP_CONFIGURE(priv->epbulkout, &epdesc, true);
    #else
    ret = EP_CONFIGURE(priv->epbulkout, &g_epbulkoutdesc, true);
    #endif
    if (ret < 0)
    {
        usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_EPBULKOUTCONFIGFAIL), 0);
        goto errout;
    }
    
    priv->epbulkout->priv = priv;
    
    /* Queue read requests in the bulk OUT endpoint */
    
    DEBUGASSERT(priv->nrdq == 0);
    for (i = 0; i < CONFIG_USBTMC_NRDREQS; i++)
    {
        req           = priv->rdreqs[i].req;
        req->callback = usbclass_rdcomplete;
        ret           = EP_SUBMIT(priv->epbulkout, req);
        if (ret != OK)
        {
            usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_RDSUBMIT), (uint16_t)-ret);
            goto errout;
        }
        
        priv->nrdq++;
    }
    
    /* We are successfully configured */
    
    priv->config = config;
    
    /* Inform the "upper half" driver that we are "open for business" */
    
    return OK;
    
    errout:
    usbclass_resetconfig(priv);
    return ret;
}

/****************************************************************************
 * Name: usbclass_ep0incomplete
 *
 * Description:
 *   Handle completion of EP0 control operations
 *
 ****************************************************************************/

static void usbclass_ep0incomplete(FAR struct usbdev_ep_s *ep,
                                   FAR struct usbdev_req_s *req)
{
    if (req->result || req->xfrd != req->len)
    {
        usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_REQRESULT), (uint16_t)-req->result);
    }
}

/****************************************************************************
 * Name: usbclass_rdcomplete
 *
 * Description:
 *   Handle completion of read request on the bulk OUT endpoint.  This
 *   is handled like the receipt of serial data on the "UART"
 *
 ****************************************************************************/

static void usbclass_rdcomplete(FAR struct usbdev_ep_s *ep,
                                FAR struct usbdev_req_s *req)
{
    FAR struct usbtmc_dev_s *priv;
    irqstate_t flags;
    int ret;
    
    /* Sanity check */
    
    #ifdef CONFIG_DEBUG
    if (!ep || !ep->priv || !req)
    {
        usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_INVALIDARG), 0);
        return;
    }
    #endif
    
    /* Extract references to private data */
    
    priv = (FAR struct usbtmc_dev_s*)ep->priv;
    
    /* Process the received data unless this is some unusual condition */
    
    flags = irqsave();
    switch (req->result)
    {
        case 0: /* Normal completion */
            usbtrace(TRACE_CLASSRDCOMPLETE, priv->nrdq);
            usbclass_recvpacket(priv, req->buf, req->xfrd);
            break;
            
        case -ESHUTDOWN: /* Disconnection */
            usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_RDSHUTDOWN), 0);
            priv->nrdq--;
            irqrestore(flags);
            return;
            
        default: /* Some other error occurred */
            usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_RDUNEXPECTED), (uint16_t)-req->result);
            break;
    };
    
    /* Requeue the read request */
    
    req->len = ep->maxpacket;
    ret      = EP_SUBMIT(ep, req);
    if (ret != OK)
    {
        usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_RDSUBMIT), (uint16_t)-req->result);
    }
    irqrestore(flags);
}

/****************************************************************************
 * Name: usbclass_wrcomplete
 *
 * Description:
 *   Handle completion of write request.  This function probably executes
 *   in the context of an interrupt handler.
 *
 ****************************************************************************/

static void usbclass_wrcomplete(FAR struct usbdev_ep_s *ep,
                                FAR struct usbdev_req_s *req)
{
    FAR struct usbtmc_dev_s *priv;
    FAR struct usbtmc_req_s *reqcontainer;
    irqstate_t flags;
    
    /* Sanity check */
    
    #ifdef CONFIG_DEBUG
    if (!ep || !ep->priv || !req || !req->priv)
    {
        usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_INVALIDARG), 0);
        return;
    }
    #endif
    
    /* Extract references to our private data */
    
    priv         = (FAR struct usbtmc_dev_s *)ep->priv;
    reqcontainer = (FAR struct usbtmc_req_s *)req->priv;
    
    /* Return the write request to the free list */
    
    flags = irqsave();
    sq_addlast((sq_entry_t*)reqcontainer, &priv->reqlist);
    priv->nwrq++;
    irqrestore(flags);
    
    /* Send the next packet unless this was some unusual termination
     * condition
     */
    
    switch (req->result)
    {
        case OK: /* Normal completion */
            usbtrace(TRACE_CLASSWRCOMPLETE, priv->nwrq);
            usbclass_sndpacket(priv);
            break;
            
        case -ESHUTDOWN: /* Disconnection */
            usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_WRSHUTDOWN), priv->nwrq);
            break;
            
        default: /* Some other error occurred */
            usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_WRUNEXPECTED), (uint16_t)-req->result);
            break;
    }
}

/****************************************************************************
 * USB Class Driver Methods
 ****************************************************************************/

/****************************************************************************
 * Name: usbclass_bind
 *
 * Description:
 *   Invoked when the driver is bound to a USB device driver
 *
 ****************************************************************************/

static int usbclass_bind(FAR struct usbdevclass_driver_s *driver,
                         FAR struct usbdev_s *dev)
{
    FAR struct usbtmc_dev_s *priv = ((FAR struct usbtmc_driver_s*)driver)->dev;
    FAR struct usbtmc_req_s *reqcontainer;
    irqstate_t flags;
    uint16_t reqlen;
    int ret;
    int i,type;
    pthread_mutexattr_t mattr;
    ficlSystemInformation fsi;
    
    usbtrace(TRACE_CLASSBIND, 0);
    
    /* Bind the structures */
    
    priv->usbdev   = dev;
    
    /* Save the reference to our private data structure in EP0 so that it
     * can be recovered in ep0 completion events (Unless we are part of
     * a composite device and, in that case, the composite device owns
     * EP0).
     */
    
    dev->ep0->priv = priv;
    
    /* Preallocate control request */
    
    priv->ctrlreq = usbclass_allocreq(dev->ep0, USBTMC_MXDESCLEN);
    if (priv->ctrlreq == NULL)
    {
        usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_ALLOCCTRLREQ), 0);
        ret = -ENOMEM;
        goto errout;
    }
    priv->ctrlreq->callback = usbclass_ep0incomplete;
    
    /* Pre-allocate all endpoints... the endpoints will not be functional
     * until the SET CONFIGURATION request is processed in usbclass_setconfig.
     * This is done here because there may be calls to kmalloc and the SET
     * CONFIGURATION processing probably occurrs within interrupt handling
     * logic where kmalloc calls will fail.
     */
    
    /* Pre-allocate the IN interrupt endpoint */
    
    priv->epintin = DEV_ALLOCEP(dev, USBTMC_EPINTIN_ADDR, true, USB_EP_ATTR_XFER_INT);
    if (!priv->epintin)
    {
        usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_EPINTINALLOCFAIL), 0);
        ret = -ENODEV;
        goto errout;
    }
    priv->epintin->priv = priv;
    
    /* Pre-allocate the IN bulk endpoint */
    
    priv->epbulkin = DEV_ALLOCEP(dev, USBTMC_EPINBULK_ADDR, true, USB_EP_ATTR_XFER_BULK);
    if (!priv->epbulkin)
    {
        usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_EPBULKINALLOCFAIL), 0);
        ret = -ENODEV;
        goto errout;
    }
    priv->epbulkin->priv = priv;
    
    /* Pre-allocate the OUT bulk endpoint */
    
    priv->epbulkout = DEV_ALLOCEP(dev, USBTMC_EPOUTBULK_ADDR, false, USB_EP_ATTR_XFER_BULK);
    if (!priv->epbulkout)
    {
        usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_EPBULKOUTALLOCFAIL), 0);
        ret = -ENODEV;
        goto errout;
    }
    priv->epbulkout->priv = priv;
    
    /* Pre-allocate read requests.  The buffer size is one full packet. */
    
    #ifdef CONFIG_USBDEV_DUALSPEED
    reqlen = 512;
    #else
    reqlen = 64;
    #endif
    
    for (i = 0; i < CONFIG_USBTMC_NRDREQS; i++)
    {
        reqcontainer      = &priv->rdreqs[i];
        reqcontainer->req = usbclass_allocreq(priv->epbulkout, reqlen);
        if (reqcontainer->req == NULL)
        {
            usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_RDALLOCREQ), -ENOMEM);
            ret = -ENOMEM;
            goto errout;
        }
        
        reqcontainer->req->priv     = reqcontainer;
        reqcontainer->req->callback = usbclass_rdcomplete;
    }
    
    /* Pre-allocate write request containers and put in a free list.
     * The buffer size should be larger than a full packet.  Otherwise,
     * we will send a bogus null packet at the end of each packet.
     *
     * Pick the larger of the max packet size and the configured request
     * size.
     */
    
    #ifdef CONFIG_USBDEV_DUALSPEED
    reqlen = 512;
    #else
    reqlen = 64;
    #endif
    
    if (CONFIG_USBTMC_BULKIN_REQLEN > reqlen)
    {
        reqlen = CONFIG_CDCACM_BULKIN_REQLEN;
    }
    
    for (i = 0; i < CONFIG_USBTMC_NWRREQS; i++)
    {
        reqcontainer      = &priv->wrreqs[i];
        reqcontainer->req = usbclass_allocreq(priv->epbulkin, reqlen);
        if (reqcontainer->req == NULL)
        {
            usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_WRALLOCREQ), -ENOMEM);
            ret = -ENOMEM;
            goto errout;
        }
        
        reqcontainer->req->priv     = reqcontainer;
        reqcontainer->req->callback = usbclass_wrcomplete;
        
        flags = irqsave();
        sq_addlast((sq_entry_t*)reqcontainer, &priv->reqlist);
        priv->nwrq++;     /* Count of write requests available */
        irqrestore(flags);
    }
    
    ficlSystemInformationInitialize(&fsi);
    fsi.textOut = usbtmc_ficlTextOut;
    fsi.errorOut = usbtmc_ficlTextOut;
    fsi.context = priv;
    priv->ficl_system = ficlSystemCreate(&fsi);
    ficlSystemCompileExtras(priv->ficl_system);
    priv->ficl_vm = ficlSystemCreateVm(priv->ficl_system);
    
    sem_init(&priv->read_sem,0,0);
    sem_init(&priv->write_sem,0,CONFIG_USBTMC_BUFFCOUNT);
#if 1
    //init pthread mutex for rxbuffer and txbuffer
    pthread_mutexattr_init(&mattr);
    ret = pthread_mutexattr_settype(&mattr, PTHREAD_MUTEX_RECURSIVE);
    if (ret != OK) {
      usbtrace(TRACE_CLASSBIND,ret);
      goto errout;
    }
    ret = pthread_mutexattr_gettype(&mattr, &type);
    if (ret != 0)
    {
      printf("ERROR pthread_mutexattr_gettype failed, ret=%d\n", ret);
    }
    if (type != PTHREAD_MUTEX_RECURSIVE)
    {
      printf("ERROR pthread_mutexattr_gettype return type=%d\n", type);
    }
    pthread_mutex_init(&priv->rxbuffer.mutex, &mattr);
    pthread_mutex_init(&priv->txbuffer.mutex, &mattr);
#endif
    // Create a pthread to do usbtmc messaged decode
    ret = pthread_create(&priv->decodethread,NULL,usbtmc_decode_thread,priv);
    if (ret != OK) {
      usbtrace(TRACE_CLASSBIND,errno);
      ret = -errno;
      goto errout;
    }

    /* Report if we are selfpowered */
    
    #ifdef CONFIG_USBDEV_SELFPOWERED
    DEV_SETSELFPOWERED(dev);
    #endif
    
    /* And pull-up the data line for the soft connect function */
    
    DEV_CONNECT(dev);
    
    return OK;
    
errout:
    usbclass_unbind(driver, dev);
    return ret;
}

/****************************************************************************
 * Name: usbclass_unbind
 *
 * Description:
 *    Invoked when the driver is unbound from a USB device driver
 *
 ****************************************************************************/

static void usbclass_unbind(FAR struct usbdevclass_driver_s *driver,
                            FAR struct usbdev_s *dev)
{
    FAR struct usbtmc_dev_s *priv;
    FAR struct usbtmc_req_s *reqcontainer;
    irqstate_t flags;
    int i;
    
    usbtrace(TRACE_CLASSUNBIND, 0);
    
    #ifdef CONFIG_DEBUG
    if (!driver || !dev || !dev->ep0)
    {
        usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_INVALIDARG), 0);
        return;
    }
    #endif
    
    /* Extract reference to private data */
    
    priv = ((FAR struct usbtmc_driver_s*)driver)->dev;
    
    #ifdef CONFIG_DEBUG
    if (!priv)
    {
        usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_EP0NOTBOUND), 0);
        return;
    }
    #endif
    
    /* Make sure that we are not already unbound */
    
    if (priv != NULL)
    {
        /* Make sure that the endpoints have been unconfigured.  If
         * we were terminated gracefully, then the configuration should
         * already have been reset.  If not, then calling usbclass_resetconfig
         * should cause the endpoints to immediately terminate all
         * transfers and return the requests to us (with result == -ESHUTDOWN)
         */
        
        usbclass_resetconfig(priv);
        up_mdelay(50);
        
        /* Free the interrupt IN endpoint */
        
        if (priv->epintin)
        {
            DEV_FREEEP(dev, priv->epintin);
            priv->epintin = NULL;
        }
        
        /* Free the bulk IN endpoint */
        
        if (priv->epbulkin)
        {
            DEV_FREEEP(dev, priv->epbulkin);
            priv->epbulkin = NULL;
        }
        
        /* Free the pre-allocated control request */
        
        if (priv->ctrlreq != NULL)
        {
            usbclass_freereq(dev->ep0, priv->ctrlreq);
            priv->ctrlreq = NULL;
        }
        
        /* Free pre-allocated read requests (which should all have
         * been returned to the free list at this time -- we don't check)
         */
        
        DEBUGASSERT(priv->nrdq == 0);
        for (i = 0; i < CONFIG_USBTMC_NRDREQS; i++)
        {
            reqcontainer = &priv->rdreqs[i];
            if (reqcontainer->req)
            {
                usbclass_freereq(priv->epbulkout, reqcontainer->req);
                reqcontainer->req = NULL;
            }
        }
        
        /* Free the bulk OUT endpoint */
        
        if (priv->epbulkout)
        {
            DEV_FREEEP(dev, priv->epbulkout);
            priv->epbulkout = NULL;
        }
        
        /* Free write requests that are not in use (which should be all
         * of them
         */
        
        flags = irqsave();
        DEBUGASSERT(priv->nwrq == CONFIG_USBTMC_NWRREQS);
        while (!sq_empty(&priv->reqlist))
        {
            reqcontainer = (struct usbtmc_req_s *)sq_remfirst(&priv->reqlist);
            if (reqcontainer->req != NULL)
            {
                usbclass_freereq(priv->epbulkin, reqcontainer->req);
                priv->nwrq--;     /* Number of write requests queued */
            }
        }
        DEBUGASSERT(priv->nwrq == 0);
        irqrestore(flags);
    }
    
    pthread_cancel(priv->decodethread);
    pthread_mutex_destroy(&priv->rxbuffer.mutex);
    pthread_mutex_destroy(&priv->txbuffer.mutex);
    sem_destroy(&priv->read_sem);
    sem_destroy(&priv->write_sem);
    // Whether need to clear all txbuffer and rxbuffer?
}

/****************************************************************************
 * Name: usbclass_setup
 *
 * Description:
 *   Invoked for ep0 control requests.  This function probably executes
 *   in the context of an interrupt handler.
 *
 ****************************************************************************/

static int usbclass_setup(FAR struct usbdevclass_driver_s *driver,
                          FAR struct usbdev_s *dev,
                          FAR const struct usb_ctrlreq_s *ctrl,
                          FAR uint8_t *dataout, size_t outlen)
{
    FAR struct usbtmc_dev_s *priv;
    FAR struct usbdev_req_s *ctrlreq;
    uint16_t value;
    uint16_t index;
    uint16_t len;
    int ret = -EOPNOTSUPP;
    
    #ifdef CONFIG_DEBUG
    if (!driver || !dev || !dev->ep0 || !ctrl)
    {
        usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_INVALIDARG), 0);
        return -EIO;
    }
    #endif
    
    /* Extract reference to private data */
    
    usbtrace(TRACE_CLASSSETUP, ctrl->req);
    
    priv = ((FAR struct usbtmc_driver_s*)driver)->dev;
    
    #ifdef CONFIG_DEBUG
    if (!priv || !priv->ctrlreq)
    {
        usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_EP0NOTBOUND), 0);
        return -ENODEV;
    }
    #endif
    ctrlreq = priv->ctrlreq;
    
    /* Extract the little-endian 16-bit values to host order */
    
    value = GETUINT16(ctrl->value);
    index = GETUINT16(ctrl->index);
    len   = GETUINT16(ctrl->len);
    
    
    switch (ctrl->type & USB_REQ_TYPE_MASK)
    {
        /***********************************************************************
         * Standard Requests
         ***********************************************************************/
        
        case USB_REQ_TYPE_STANDARD:
        {
            switch (ctrl->req)
            {
                case USB_REQ_GETDESCRIPTOR:
                {
                    /* The value field specifies the descriptor type in the MS byte and the
                     * descriptor index in the LS byte (order is little endian)
                     */
                    
                    switch (ctrl->value[1])
                    {
                        case USB_DESC_TYPE_DEVICE:
                        {
                            ret = USB_SIZEOF_DEVDESC;
                            memcpy(ctrlreq->buf, &g_devdesc, ret);
                        }
                        break;
                        
                        #ifdef CONFIG_USBDEV_DUALSPEED
                        case USB_DESC_TYPE_DEVICEQUALIFIER:
                        {
                            ret = USB_SIZEOF_QUALDESC;
                            memcpy(ctrlreq->buf, &g_qualdesc, ret);
                        }
                        break;
                        
                        case USB_DESC_TYPE_OTHERSPEEDCONFIG:
                            #endif /* CONFIG_USBDEV_DUALSPEED */
                            
                        case USB_DESC_TYPE_CONFIG:
                        {
                            #ifdef CONFIG_USBDEV_DUALSPEED
                            ret = usbclass_mkcfgdesc(ctrlreq->buf, dev->speed, ctrl->req);
                            #else
                            ret = usbclass_mkcfgdesc(ctrlreq->buf);
                            #endif
                        }
                        break;
                        
                        case USB_DESC_TYPE_STRING:
                        {
                            /* index == language code. */
                            
                            ret = usbclass_mkstrdesc(ctrl->value[0], (struct usb_strdesc_s *)ctrlreq->buf);
                        }
                        break;
                        
                        default:
                        {
                            usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_GETUNKNOWNDESC), value);
                        }
                        break;
                    }
                }
                break;
                
                case USB_REQ_SETCONFIGURATION:
                {
                  if (ctrl->type == 0)
                  {
                    ret = usbclass_setconfig(priv, value);
                  }
                }
                break;

                case USB_REQ_GETCONFIGURATION:
                {
                  if (ctrl->type == USB_DIR_IN)
                  {
                    *(uint8_t*)ctrlreq->buf = priv->config;
                    ret = 1;
                  }
                }
                break;

                case USB_REQ_SETINTERFACE:
                {
                  if (ctrl->type == USB_REQ_RECIPIENT_INTERFACE)
                  {
                    if (priv->config == USBTMC_CONFIGID &&
                      index == USBTMC_INTERFACEID &&
                      value == USBTMC_ALTINTERFACEID)
                    {
                      usbclass_resetconfig(priv);
                      usbclass_setconfig(priv, priv->config);
                      ret = 0;
                    }
                  }
                }
                break;

                case USB_REQ_GETINTERFACE:
                {
                  if (ctrl->type == (USB_DIR_IN|USB_REQ_RECIPIENT_INTERFACE) &&
                    priv->config == USBTMC_CONFIGIDNONE)
                  {
                    if (index != USBTMC_INTERFACEID)
                    {
                      ret = -EDOM;
                    }
                    else
                    {
                      *(uint8_t*) ctrlreq->buf = USBTMC_ALTINTERFACEID;
                      ret = 1;
                    }
                  }
                }
                break;
                case USB_REQ_CLEARFEATURE:
                /* Handle Clear Feature specified in usbtmc specs*/
                {
                  if(value == USB_FEATURE_ENDPOINTHALT && len == 0) {
                    if (index == USBTMC_EPOUTBULK_ADDR) {
                      /* The device, after receiving CLEAR_FEATURE request, must interpret 
                      the first part of the next Bulk-OUT transaction as a new USBTMC Bulk-OUT Header*/
                      // TODO add logic
                    } else if (index == USBTMC_EPINBULK_ADDR) {
                      /* The device, after receiving CLEAR_FEATURE request, msut not queue any bulk-in data 
                      until it receives a USBTMC command message that expects a reponse*/
                      //TODO add logic
                    }
                  }
                }
                break;
                default:
                usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_UNSUPPORTEDSTDREQ), ctrl->req);
                break;
              }
            }
            break;
        
        /***********************************************************************
         * USBTMC Class-Specific Requests
         ***********************************************************************/
        case USB_REQ_TYPE_CLASS:
        {
          switch (ctrl->req) {
            case INITIATE_ABORT_BULK_OUT:
            case INITIATE_ABORT_BULK_IN:
            {
              if (ctrl->type & USB_REQ_RECIPIENT_MASK == USB_REQ_RECIPIENT_ENDPOINT ) {
                if (len != 2) {
                  ret = 0;
                  usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_INVALIDARG),INITIATE_ABORT_BULK_OUT);
                }
                if (value == priv->currTag) {
                  ctrlreq->buf[0] = STATUS_SUCCESS;
                } else {
                  ctrlreq->buf[0] = STATUS_TRANSFER_NOT_IN_PROGRESS;
                }
                ctrlreq->buf[1] = priv->currTag;
                ret = 2;
              }
            }
            break;

            case CHECK_ABORT_BULK_OUT_STATUS:
            case CHECK_ABORT_BULK_IN_STATUS:
            {
              if (ctrl->type & USB_REQ_RECIPIENT_MASK == USB_REQ_RECIPIENT_ENDPOINT ) {
                pthread_cancel(priv->decodethread);
                sem_destroy(&priv->read_sem);
                sem_destroy(&priv->write_sem);
                priv->bTransferBegin = false;
                priv->currTag = 0;
                priv->currTransSize = 0;
                if (ctrl->req == CHECK_ABORT_BULK_OUT_STATUS) {
                  memset(&priv->rxbuffer.buff,0,USBTMC_EP_MXPACKET*CONFIG_USBTMC_BUFFCOUNT);
                } else {
                  memset(&priv->txbuffer.buff,0,USBTMC_EP_MXPACKET*CONFIG_USBTMC_BUFFCOUNT);
                }
                priv->rxbuffer.head = 0;
                priv->rxbuffer.tail = 0;
                priv->rxbuffer.used = 0;
                ctrlreq->buf[0] = STATUS_SUCCESS;
                ctrlreq->buf[1] = 0;
                ctrlreq->buf[2] = 0;
                ctrlreq->buf[3] = 0;
                if (ctrl->req == CHECK_ABORT_BULK_OUT_STATUS) {
                  *(uint32_t*)ctrlreq->buf[4] = priv->currRecvSize;
                }else {
                  *(uint32_t*)ctrlreq->buf[4] = priv->currSendSize;
                }
                priv->currRecvSize = 0;
                sem_init(&priv->read_sem,0,0);
                sem_init(&priv->write_sem,0,0);
                pthread_create(&priv->decodethread,NULL,usbtmc_decode_thread,priv);
                ret = 8;
              }
            }
            break;

            case INITIATE_CLEAR:
            {
              if (ctrl->type & USB_REQ_RECIPIENT_MASK == USB_REQ_RECIPIENT_INTERFACE ) {
                // not supported now
              }
            }
            break;
            
            case CHECK_CLEAR_STATUS:
            {
              if (ctrl->type & USB_REQ_RECIPIENT_MASK == USB_REQ_RECIPIENT_INTERFACE ) {
                // not supported now
              }
            }
            break;

            case GET_CAPABILITIES:
            {
              if (ctrl->type & USB_REQ_RECIPIENT_MASK == USB_REQ_RECIPIENT_INTERFACE ) {
                if (len != 0x18) {
                  ret = 0;
                  usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_INVALIDARG),GET_CAPABILITIES);
                }
                ret = 0x18;
                memcpy(ctrlreq->buf, &g_cap, ret); 
              }
            }
            break;

            case INDICATIOR_PULSE:
            {
              if (ctrl->type & USB_REQ_RECIPIENT_MASK == USB_REQ_RECIPIENT_INTERFACE ) {
                // TODO ADD Logic
              }
            }
            break;

            default:
              usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_UNSUPPORTEDCLASSREQ),ctrl->req);
              break;
          }
        }                      
        default:
          usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_UNSUPPORTEDTYPE), ctrl->type);
          break;
    }
    
    /* Respond to the setup command if data was returned.  On an error return
     * value (ret < 0), the USB driver will stall.
     */
    
    if (ret >= 0)
    {
        ctrlreq->len   = min(len, ret);
        ctrlreq->flags = USBDEV_REQFLAGS_NULLPKT;
        ret            = EP_SUBMIT(dev->ep0, ctrlreq);
        if (ret < 0)
        {
            usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_EPRESPQ), (uint16_t)-ret);
            ctrlreq->result = OK;
            usbclass_ep0incomplete(dev->ep0, ctrlreq);
        }
    }
    
    return ret;
}

/****************************************************************************
 * Name: usbclass_disconnect
 *
 * Description:
 *   Invoked after all transfers have been stopped, when the host is
 *   disconnected.  This function is probably called from the context of an
 *   interrupt handler.
 *
 ****************************************************************************/

static void usbclass_disconnect(FAR struct usbdevclass_driver_s *driver,
                                FAR struct usbdev_s *dev)
{
    FAR struct usbtmc_dev_s *priv;
    irqstate_t flags;
    
    usbtrace(TRACE_CLASSDISCONNECT, 0);
    
    #ifdef CONFIG_DEBUG
    if (!driver || !dev || !dev->ep0)
    {
        usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_INVALIDARG), 0);
        return;
    }
    #endif
    
    /* Extract reference to private data */
    
    priv = ((FAR struct usbtmc_driver_s*)driver)->dev;
    
    #ifdef CONFIG_DEBUG
    if (!priv)
    {
        usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_EP0NOTBOUND), 0);
        return;
    }
    #endif
    
    /* Inform the "upper half serial driver that we have lost the USB serial
     * connection.
     */
    
    flags = irqsave();
    
    /* Reset the configuration */
    
    usbclass_resetconfig(priv);
    
    /* Clear out all outgoing data in the circular buffer */
    irqrestore(flags);
    
    /* Perform the soft connect function so that we will we can be
     * re-enumerated.
     */
    
    DEV_CONNECT(dev);
}

void  usbtmc_ficlTextOut(ficlCallback *callback, char *message)
{
  FAR struct usbtmc_dev_s *priv = (struct usbtmc_dev_s *)callback->context;
  uint32_t transfersize,size;
  usbtmc_bulkin_header *pHeader = NULL;
  int ret;
  bool bFirst = true;

  if (NULL == priv) {
    usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_FICL_INVILDARG),EINVAL);
    return;
  }

  transfersize = strlen(message);
  priv->currSendSize = 0;
  
  while (transfersize) {
    ret = sem_wait(&priv->write_sem);
    if (ret == OK) {
      memset(&priv->txbuffer.buff[priv->txbuffer.tail],0,USBTMC_EP_MXPACKET);
      if (bFirst) {
        bFirst = false;
        pHeader = (usbtmc_bulkin_header*)priv->txbuffer.buff[priv->txbuffer.tail];
        pHeader->msgID = DEV_DEP_MSG_IN;
        pHeader->bTag = priv->currTag;
        pHeader->bTagInverse = 0xFF-priv->currTag;
        pHeader->transferSize = transfersize;
        pHeader->header.DevDepMsgIn.bmTransferAttr.EOM = 1;
        if (transfersize > (USBTMC_EP_MXPACKET-USBTMC_HEADER_SIZE)) {
          size = USBTMC_EP_MXPACKET-USBTMC_HEADER_SIZE;
        } else {
          size = transfersize;
        } 
        memcpy(&priv->txbuffer.buff[priv->txbuffer.tail][USBTMC_HEADER_SIZE],message,size);
      } else {
        if (transfersize > USBTMC_EP_MXPACKET) {
          size = USBTMC_EP_MXPACKET;
        } else {
          size = transfersize;
        }
        memcpy(&priv->txbuffer.buff[priv->txbuffer.tail],message,size);
      }
      transfersize -= size;
      message += size;
      priv->currSendSize += size;
      pthread_mutex_lock(&priv->txbuffer.mutex);
      priv->txbuffer.tail = (priv->txbuffer.tail + 1) % CONFIG_USBTMC_BUFFCOUNT;
      priv->txbuffer.used++;
      pthread_mutex_unlock(&priv->txbuffer.mutex);

    }
  }
}

void usbtmc_reportADCData(struct usbtmc_dev_s *priv) 
{
  uint8_t *pADCDMABuff = &g_adc1_dmabuff;
  int ret;

  priv->currSendSize = 0;
  ret = sem_wait(&priv->write_sem);
  if (ret == OK) {
    usbtmc_bulkin_header *pHeader = priv->txbuffer.buff[priv->txbuffer.tail];
    pHeader->msgID = VENDOR_SPECIFIC_IN;
    pHeader->bTag  = priv->currTag;
    pHeader->bTagInverse = 0xFF - priv->currTag;
    pHeader->transferSize = priv->currTransSize;

    memcpy(&priv->txbuffer.buff[priv->txbuffer.tail][USBTMC_HEADER_SIZE],pADCDMABuff,USBTMC_EP_MXPACKET-USBTMC_HEADER_SIZE);
    priv->currTransSize -= USBTMC_EP_MXPACKET-USBTMC_HEADER_SIZE;
    priv->currSendSize += USBTMC_EP_MXPACKET-USBTMC_HEADER_SIZE;
    pADCDMABuff += USBTMC_EP_MXPACKET-USBTMC_HEADER_SIZE;
    pthread_mutex_lock(&priv->txbuffer.mutex);
    priv->txbuffer.tail = (priv->txbuffer.tail+1) % CONFIG_USBTMC_BUFFCOUNT;
    priv->txbuffer.used++;
    pthread_mutex_unlock(&priv->txbuffer.mutex);
  }

  while (priv->currTransSize) {
    ret = sem_wait(&priv->write_sem);
    if (ret == OK) {
      if (priv->currTransSize > USBTMC_EP_MXPACKET) {
        memcpy(&priv->txbuffer.buff[priv->txbuffer.tail],pADCDMABuff,USBTMC_EP_MXPACKET);
        priv->currTransSize -= USBTMC_EP_MXPACKET;
        priv->currSendSize  += USBTMC_EP_MXPACKET;
        pADCDMABuff += USBTMC_EP_MXPACKET;
        
      } else {
        memcpy(&priv->txbuffer.buff[priv->txbuffer.tail],pADCDMABuff,priv->currTransSize);
        priv->currSendSize += priv->currTransSize;
        priv->currTransSize = 0;
      }
      pthread_mutex_lock(&priv->txbuffer.mutex);
      priv->txbuffer.tail = (priv->txbuffer.tail+1) % CONFIG_USBTMC_BUFFCOUNT;
      priv->txbuffer.used++;
      pthread_mutex_unlock(&priv->txbuffer.mutex);
    }
  }
}

void *usbtmc_decode_thread(void *arg) 
{
  int ret;
  FAR struct usbtmc_dev_s *priv = (struct usbtmc_dev_s *)arg;
  uint8_t msg[USBTMC_EP_MXPACKET];
  static bool bFiclCmd;
  uint8_t *pCmd = NULL,pCur=NULL,pDAC1DMABuff = &g_DAC1DMABuffer;

  pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);

  pCmd = malloc(USBTMC_FICL_CMD_SIZE);
  if (pCmd == NULL) {
    usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_ALLOC),ENOMEM);
    pthread_exit(NULL);
    return NULL;
  }
  pCur = pCmd;
  memset(pCur,0,USBTMC_FICL_CMD_SIZE);

  while (1) {
    ret = sem_wait(&priv->read_sem);
    if (ret == OK) {
      /* new usbtmc message available,decode it*/
      memcpy(msg,priv->rxbuffer.buff[priv->rxbuffer.head],USBTMC_EP_MXPACKET);
      pthread_mutex_lock(&priv->rxbuffer.mutex);
      priv->rxbuffer.used--;
      priv->rxbuffer.head = (priv->rxbuffer.head + 1) % CONFIG_USBTMC_BUFFCOUNT;
      pthread_mutex_unlock(&priv->rxbuffer.mutex);

      if (priv->bTransferBegin == false && priv->currTransSize == 0) {
        /* new USBTMC Header*/
        usbtmc_bulkout_header *header = (usbtmc_bulkout_header*)msg;
        // check header tag
        if (header->bTag + header->bTagInverse != 0xFF) {
          // invalid header
          continue;
        }
        if ((header->msgID == DEV_DEP_MSG_OUT) && (header->transferSize > USBTMC_FICL_CMD_SIZE)) {
          usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_FICL_OVERRUN),0);
          continue;
        }
        priv->currTag = header->bTag;
        if (header->transferSize + USBTMC_HEADER_SIZE > USBTMC_EP_MXPACKET) {
          // next packet is usbtmc message byte, set bTransferBegin flag to indicates not decode header
          priv->bTransferBegin = true;
        }
        priv->currTransSize = header->transferSize;
        priv->currRecvSize = 0;


        switch (header->msgID) {
          case DEV_DEP_MSG_OUT:
          {
            /* Ficl command */
            bFiclCmd = true;
            // if (header.header.DevDepMsgOut.bmTransferAttr.EOM) {
            //   // End of Message, next packet will contain a usbtmc header
            //   priv->bTransferBegin = false;
            // } else {
            //   priv->bTransferBegin = true;
            // }
            if (priv->currTransSize <= (USBTMC_EP_MXPACKET - USBTMC_HEADER_SIZE)) {
              memcpy(pCur,&msg[USBTMC_HEADER_SIZE],priv->currTransSize);
              priv->currTransSize = 0;
              priv->currRecvSize = priv->currTransSize;
              ficlVmEvaluate(priv->ficl_vm,pCmd);
              memset(pCmd,0,USBTMC_FICL_CMD_SIZE);
            } else {
              memcpy(pCur,&msg[USBTMC_HEADER_SIZE],USBTMC_EP_MXPACKET - USBTMC_HEADER_SIZE);
              pCur += USBTMC_EP_MXPACKET - USBTMC_HEADER_SIZE;
              priv->currTransSize -= USBTMC_EP_MXPACKET - USBTMC_HEADER_SIZE;
              priv->currRecvSize += USBTMC_EP_MXPACKET - USBTMC_HEADER_SIZE;
            }
          }
          break;

          case REQUEST_DEV_DEP_MSG_IN:
          {
            bFiclCmd = true;
            usbclass_sndpacket(priv);
          }
          break;

          case VENDOR_SPECIFIC_OUT:
          {
            bFiclCmd = false;
            /* Set DAC DMA buffer,always send CONFIG_STM32_DAC_DMA_BUFFER_SIZE bytes
               no need to check TransferSize here*/
            memcpy(pDAC1DMABuff, &msg[USBTMC_HEADER_SIZE], USBTMC_EP_MXPACKET-USBTMC_HEADER_SIZE);
            pDAC1DMABuff += USBTMC_EP_MXPACKET-USBTMC_HEADER_SIZE;
            priv->currTransSize -= USBTMC_EP_MXPACKET-USBTMC_HEADER_SIZE;
            priv->currRecvSize += USBTMC_EP_MXPACKET-USBTMC_HEADER_SIZE;
          }
          break;

          case REQUEST_VENDOR_SPECIFIC_IN:
          {
            bFiclCmd = false;
            usbtmc_reportADCData(priv);
          }
          break;
        }

      } else {
        /* USBTMC message bytes*/
        if (bFiclCmd) {
          if (priv->currTransSize > USBTMC_EP_MXPACKET) {
            memcpy(pCur,msg,USBTMC_EP_MXPACKET);
            priv->currTransSize -= USBTMC_EP_MXPACKET;
            priv->currRecvSize += USBTMC_EP_MXPACKET;
            pCur += USBTMC_EP_MXPACKET;
          } else {
            memcpy(pCur,msg,priv->currTransSize);
            priv->currTransSize = 0;
            ficlVmEvaluate(priv->ficl_vm,pCmd);
            memset(pCmd,0,USBTMC_FICL_CMD_SIZE);
            pCur = pCmd;
            priv->currRecvSize = 0;
          }
        } else {
          // FlashPower DAC output value
          if (priv->currTransSize > USBTMC_EP_MXPACKET) {
            memcpy(pDAC1DMABuff,msg,USBTMC_EP_MXPACKET);
            priv->currTransSize -= USBTMC_EP_MXPACKET;
            priv->currRecvSize += USBTMC_EP_MXPACKET;
            pDAC1DMABuff += USBTMC_EP_MXPACKET;
          } else {
            memcpy(pDAC1DMABuff,msg,priv->currTransSize);
            priv->currTransSize = 0;
            priv->currRecvSize = 0;
            pDAC1DMABuff = &g_DAC1DMABuffer;
          }
        }
      }
    }
  }
}

int usbdev_usbtmcinitialize(int minor)
{
  FAR struct usbtmc_alloc_s *alloc;
  FAR struct usbtmc_dev_s *priv;
  FAR struct usbtmc_driver_s *drvr;
  char devname[16];
  int ret;

  /* Allocate the structures needed */

  alloc = (FAR struct usbtmc_alloc_s*)kmalloc(sizeof(struct usbtmc_alloc_s));
  if (!alloc)
    {
      usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_ALLOCDEVSTRUCT), 0);
      return -ENOMEM;
    }

  /* Convenience pointers into the allocated blob */

  priv = &alloc->dev;
  drvr = &alloc->drvr;

  /* Initialize the USB serial driver structure */

  memset(priv, 0, sizeof(struct usbtmc_dev_s));
  sq_init(&priv->reqlist);

  /* Initialize the USB class driver structure */

#ifdef CONFIG_USBDEV_DUALSPEED
  drvr->drvr.speed         = USB_SPEED_HIGH;
#else
  drvr->drvr.speed         = USB_SPEED_FULL;
#endif
  drvr->drvr.ops           = &g_driverops;
  drvr->dev                = priv;

  /* Register the USB serial class driver */

  ret = usbdev_register(&drvr->drvr);
  if (ret)
    {
      
      usbtrace(TRACE_CLSERROR(USBTMC_TRACEERR_DEVREGISTER), (uint16_t)-ret);
      goto errout_with_alloc;
    }
    return OK;

errout_with_class:
  usbdev_unregister(&drvr->drvr);
errout_with_alloc:
  kfree(alloc);
  return ret;
}
