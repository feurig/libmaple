/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2011 LeafLabs LLC.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 * @file libmaple/usb/stm32f1/usb_cdcacm.c
 * @brief USB CDC ACM (a.k.a. virtual serial terminal, VCOM).
 *
 * FIXME: this works on the STM32F1 USB peripherals, and probably no
 * place else. Nonportable bits really need to be factored out, and
 * the result made cleaner.
 */

#include <libmaple/usb_cdcacm.h>

#include <libmaple/usb.h>
#include <libmaple/nvic.h>
#include <libmaple/delay.h>

/* Private headers */
#include "usb_descriptors.h"
#include "usb_lib_globals.h"
#include "usb_reg_map.h"

/* usb_lib headers */
#include "usb_type.h"
#include "usb_core.h"
#include "usb_def.h"

/******************************************************************************
 ******************************************************************************
 ***
 ***   HACK ALERT! FIXME FIXME FIXME FIXME!
 ***
 ***   A bunch of LeafLabs-specific configuration lives in here for
 ***   now.  This mess REALLY needs to get teased apart, with
 ***   appropriate pieces moved into Wirish.
 ***
 ******************************************************************************
 *****************************************************************************/

#if !(defined(BOARD_maple) || defined(BOARD_maple_RET6) ||      \
      defined(BOARD_maple_mini) || defined(BOARD_maple_native))
#warning USB CDC ACM relies on LeafLabs board-specific configuration.\
    You may have problems on non-LeafLabs boards.
#endif

static void vcomDataTxCb(void);
static void vcomDataRxCb(void);
static uint8* vcomGetSetLineCoding(uint16);

static void usbInit(void);
static void usbReset(void);
static RESULT usbDataSetup(uint8 request);
static RESULT usbNoDataSetup(uint8 request);
static RESULT usbGetInterfaceSetting(uint8 interface, uint8 alt_setting);
static uint8* usbGetDeviceDescriptor(uint16 length);
static uint8* usbGetConfigDescriptor(uint16 length);
static uint8* usbGetStringDescriptor(uint16 length);
static void usbSetConfiguration(void);
static void usbSetDeviceAddress(void);

/*
 * Endpoint configuration
 */

#define USB_CDCACM_CTRL_ENDP            0
#define USB_CDCACM_CTRL_RX_ADDR         0x40
#define USB_CDCACM_CTRL_TX_ADDR         0x80
#define USB_CDCACM_CTRL_EPSIZE          0x40

#define USB_CDCACM_TX_ENDP              1
#define USB_CDCACM_TX_ADDR              0xC0
#define USB_CDCACM_TX_EPSIZE            0x40

#define USB_CDCACM_MANAGEMENT_ENDP      2
#define USB_CDCACM_MANAGEMENT_ADDR      0x100
#define USB_CDCACM_MANAGEMENT_EPSIZE    0x40

#define USB_CDCACM_RX_ENDP              3
#define USB_CDCACM_RX_ADDR              0x110
#define USB_CDCACM_RX_EPSIZE            0x40

/*
 * Descriptors
 */

#define USB_DEVICE_CLASS_CDC              0x02
#define USB_DEVICE_SUBCLASS_CDC           0x00
#define LEAFLABS_ID_VENDOR                0x1EAF
#define MAPLE_ID_PRODUCT                  0x0004
static const USB_Descriptor_Device usbVcomDescriptor_Device = {
    .bLength            = sizeof(USB_Descriptor_Device),
    .bDescriptorType    = USB_DESCRIPTOR_TYPE_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = USB_DEVICE_CLASS_CDC,
    .bDeviceSubClass    = USB_DEVICE_SUBCLASS_CDC,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = 0x40,
    .idVendor           = LEAFLABS_ID_VENDOR,
    .idProduct          = MAPLE_ID_PRODUCT,
    .bcdDevice          = 0x0200,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x00,
    .bNumConfigurations = 0x01,
};

typedef struct {
    USB_Descriptor_Config_Header Config_Header;
    USB_Descriptor_Interface     CCI_Interface;
    CDC_FUNCTIONAL_DESCRIPTOR(2) CDC_Functional_IntHeader;
    CDC_FUNCTIONAL_DESCRIPTOR(2) CDC_Functional_CallManagement;
    CDC_FUNCTIONAL_DESCRIPTOR(1) CDC_Functional_ACM;
    CDC_FUNCTIONAL_DESCRIPTOR(2) CDC_Functional_Union;
    USB_Descriptor_Endpoint      ManagementEndpoint;
    USB_Descriptor_Interface     DCI_Interface;
    USB_Descriptor_Endpoint      DataOutEndpoint;
    USB_Descriptor_Endpoint      DataInEndpoint;
} __packed USB_Descriptor_Config;

#define MAX_POWER (100 >> 1)
static const USB_Descriptor_Config usbVcomDescriptor_Config = {
    .Config_Header = {
        .bLength              = sizeof(USB_Descriptor_Config_Header),
        .bDescriptorType      = USB_DESCRIPTOR_TYPE_CONFIGURATION,
        .wTotalLength         = sizeof(USB_Descriptor_Config),
        .bNumInterfaces       = 0x02,
        .bConfigurationValue  = 0x01,
        .iConfiguration       = 0x00,
        .bmAttributes         = (USB_CONFIG_ATTR_BUSPOWERED |
                                 USB_CONFIG_ATTR_SELF_POWERED),
        .bMaxPower            = MAX_POWER,
    },

    .CCI_Interface = {
        .bLength            = sizeof(USB_Descriptor_Interface),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_INTERFACE,
        .bInterfaceNumber   = 0x00,
        .bAlternateSetting  = 0x00,
        .bNumEndpoints      = 0x01,
        .bInterfaceClass    = USB_INTERFACE_CLASS_CDC,
        .bInterfaceSubClass = USB_INTERFACE_SUBCLASS_CDC_ACM,
        .bInterfaceProtocol = 0x01, /* Common AT Commands */
        .iInterface         = 0x00,
    },

    .CDC_Functional_IntHeader = {
        .bLength         = CDC_FUNCTIONAL_DESCRIPTOR_SIZE(2),
        .bDescriptorType = 0x24,
        .SubType         = 0x00,
        .Data            = {0x01, 0x10},
    },

    .CDC_Functional_CallManagement = {
        .bLength         = CDC_FUNCTIONAL_DESCRIPTOR_SIZE(2),
        .bDescriptorType = 0x24,
        .SubType         = 0x01,
        .Data            = {0x03, 0x01},
    },

    .CDC_Functional_ACM = {
        .bLength         = CDC_FUNCTIONAL_DESCRIPTOR_SIZE(1),
        .bDescriptorType = 0x24,
        .SubType         = 0x02,
        .Data            = {0x06},
    },

    .CDC_Functional_Union = {
        .bLength         = CDC_FUNCTIONAL_DESCRIPTOR_SIZE(2),
        .bDescriptorType = 0x24,
        .SubType         = 0x06,
        .Data            = {0x00, 0x01},
    },

    .ManagementEndpoint = {
        .bLength          = sizeof(USB_Descriptor_Endpoint),
        .bDescriptorType  = USB_DESCRIPTOR_TYPE_ENDPOINT,
        .bEndpointAddress = (USB_DESCRIPTOR_ENDPOINT_IN |
                             USB_CDCACM_MANAGEMENT_ENDP),
        .bmAttributes     = EP_TYPE_INTERRUPT,
        .wMaxPacketSize   = USB_CDCACM_MANAGEMENT_EPSIZE,
        .bInterval        = 0xFF,
    },

    .DCI_Interface = {
        .bLength            = sizeof(USB_Descriptor_Interface),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_INTERFACE,
        .bInterfaceNumber   = 0x01,
        .bAlternateSetting  = 0x00,
        .bNumEndpoints      = 0x02,
        .bInterfaceClass    = USB_INTERFACE_CLASS_DIC,
        .bInterfaceSubClass = 0x00, /* None */
        .bInterfaceProtocol = 0x00, /* None */
        .iInterface         = 0x00,
    },

    .DataOutEndpoint = {
        .bLength          = sizeof(USB_Descriptor_Endpoint),
        .bDescriptorType  = USB_DESCRIPTOR_TYPE_ENDPOINT,
        .bEndpointAddress = (USB_DESCRIPTOR_ENDPOINT_OUT |
                             USB_CDCACM_RX_ENDP),
        .bmAttributes     = EP_TYPE_BULK,
        .wMaxPacketSize   = USB_CDCACM_RX_EPSIZE,
        .bInterval        = 0x00,
    },

    .DataInEndpoint = {
        .bLength          = sizeof(USB_Descriptor_Endpoint),
        .bDescriptorType  = USB_DESCRIPTOR_TYPE_ENDPOINT,
        .bEndpointAddress = (USB_DESCRIPTOR_ENDPOINT_IN | USB_CDCACM_TX_ENDP),
        .bmAttributes     = EP_TYPE_BULK,
        .wMaxPacketSize   = USB_CDCACM_TX_EPSIZE,
        .bInterval        = 0x00,
    },
};

/*
  String Identifiers:

  we may choose to specify any or none of the following string
  identifiers:

  iManufacturer:    LeafLabs
  iProduct:         Maple
  iSerialNumber:    NONE
  iConfiguration:   NONE
  iInterface(CCI):  NONE
  iInterface(DCI):  NONE

  additionally we must provide the unicode language identifier,
  which is 0x0409 for US English
*/

static const uint8 usbVcomDescriptor_LangID[USB_DESCRIPTOR_STRING_LEN(1)] = {
    USB_DESCRIPTOR_STRING_LEN(1),
    USB_DESCRIPTOR_TYPE_STRING,
    0x09,
    0x04,
};

static const uint8 usbVcomDescriptor_iManufacturer[USB_DESCRIPTOR_STRING_LEN(8)] = {
    USB_DESCRIPTOR_STRING_LEN(8),
    USB_DESCRIPTOR_TYPE_STRING,
    'L', 0, 'e', 0, 'a', 0, 'f', 0,
    'L', 0, 'a', 0, 'b', 0, 's', 0,
};

static const uint8 usbVcomDescriptor_iProduct[USB_DESCRIPTOR_STRING_LEN(8)] = {
    USB_DESCRIPTOR_STRING_LEN(8),
    USB_DESCRIPTOR_TYPE_STRING,
    'M', 0, 'a', 0, 'p', 0, 'l', 0,
    'e', 0, ' ', 0, ' ', 0, ' ', 0
};

static ONE_DESCRIPTOR Device_Descriptor = {
    (uint8*)&usbVcomDescriptor_Device,
    sizeof(USB_Descriptor_Device)
};

static ONE_DESCRIPTOR Config_Descriptor = {
    (uint8*)&usbVcomDescriptor_Config,
    sizeof(USB_Descriptor_Config)
};

static ONE_DESCRIPTOR String_Descriptor[3] = {
    {(uint8*)&usbVcomDescriptor_LangID,       USB_DESCRIPTOR_STRING_LEN(1)},
    {(uint8*)&usbVcomDescriptor_iManufacturer,USB_DESCRIPTOR_STRING_LEN(8)},
    {(uint8*)&usbVcomDescriptor_iProduct,     USB_DESCRIPTOR_STRING_LEN(8)}
};

/*
 * Etc.
 */

/* I/O state */

/* Received data */
static volatile uint8 vcomBufferRx[USB_CDCACM_RX_EPSIZE];
/* Read index into vcomBufferRx */
static volatile uint32 rx_offset = 0;
/* Number of bytes left to transmit */
static volatile uint32 n_unsent_bytes = 0;
/* Number of unread bytes */
static volatile uint32 n_unread_bytes = 0;

/* Other state (line coding, DTR/RTS) */

typedef struct usb_line_coding {
    uint32 dwDTERate;           /* Baud rate */

#define STOP_BITS_1   0
#define STOP_BITS_1_5 1
#define STOP_BITS_2   2
    uint8 bCharFormat;          /* Stop bits */

#define PARITY_NONE  0
#define PARITY_ODD   1
#define PARITY_EVEN  2
#define PARITY_MARK  3
#define PARITY_SPACE 4
    uint8 bParityType;          /* Parity type */

    uint8 bDataBits;            /* Data bits: 5, 6, 7, 8, or 16 */
} usb_line_coding;

static volatile usb_line_coding line_coding = {
    /* This default is 115200 baud, 8N1. */
    .dwDTERate   = 115200,
    .bCharFormat = STOP_BITS_1,
    .bParityType = PARITY_NONE,
    .bDataBits   = 8,
};

/* Which of USB_CDCACM_GET_LINE_CODING and USB_CDCACM_SET_LINE_CODING
 * we've most recently received. */
static volatile uint8 last_request = 0;

/* DTR in bit 0, RTS in bit 1. */
static volatile uint8 line_dtr_rts = 0;

/*
 * Endpoint callbacks
 */

static void (*ep_int_in[7])(void) =
    {vcomDataTxCb,
     NOP_Process,
     NOP_Process,
     NOP_Process,
     NOP_Process,
     NOP_Process,
     NOP_Process};

static void (*ep_int_out[7])(void) =
    {NOP_Process,
     NOP_Process,
     vcomDataRxCb,
     NOP_Process,
     NOP_Process,
     NOP_Process,
     NOP_Process};

/*
 * Globals required by usb_lib/
 */

#define NUM_ENDPTS                0x04
DEVICE Device_Table = {
    .Total_Endpoint      = NUM_ENDPTS,
    .Total_Configuration = 1
};

#define MAX_PACKET_SIZE            0x40  /* 64B, maximum for USB FS Devices */
DEVICE_PROP Device_Property = {
    .Init                        = usbInit,
    .Reset                       = usbReset,
    .Process_Status_IN           = NOP_Process,
    .Process_Status_OUT          = NOP_Process,
    .Class_Data_Setup            = usbDataSetup,
    .Class_NoData_Setup          = usbNoDataSetup,
    .Class_Get_Interface_Setting = usbGetInterfaceSetting,
    .GetDeviceDescriptor         = usbGetDeviceDescriptor,
    .GetConfigDescriptor         = usbGetConfigDescriptor,
    .GetStringDescriptor         = usbGetStringDescriptor,
    .RxEP_buffer                 = NULL,
    .MaxPacketSize               = MAX_PACKET_SIZE
};

USER_STANDARD_REQUESTS User_Standard_Requests = {
    .User_GetConfiguration   = NOP_Process,
    .User_SetConfiguration   = usbSetConfiguration,
    .User_GetInterface       = NOP_Process,
    .User_SetInterface       = NOP_Process,
    .User_GetStatus          = NOP_Process,
    .User_ClearFeature       = NOP_Process,
    .User_SetEndPointFeature = NOP_Process,
    .User_SetDeviceFeature   = NOP_Process,
    .User_SetDeviceAddress   = usbSetDeviceAddress
};

/*
 * User hooks
 */

static void (*rx_hook)(unsigned, void*) = 0;
static void (*iface_setup_hook)(unsigned, void*) = 0;

void usb_cdcacm_set_hooks(unsigned hook_flags, void (*hook)(unsigned, void*)) {
    if (hook_flags & USB_CDCACM_HOOK_RX) {
        rx_hook = hook;
    }
    if (hook_flags & USB_CDCACM_HOOK_IFACE_SETUP) {
        iface_setup_hook = hook;
    }
}

/*
 * CDC ACM interface
 */

void usb_cdcacm_enable(gpio_dev *disc_dev, uint8 disc_bit) {
    /* Present ourselves to the host. Writing 0 to "disc" pin must
     * pull USB_DP pin up while leaving USB_DM pulled down by the
     * transceiver. See USB 2.0 spec, section 7.1.7.3. */
    gpio_set_mode(disc_dev, disc_bit, GPIO_OUTPUT_PP);
    gpio_write_bit(disc_dev, disc_bit, 0);

    /* Initialize the USB peripheral. */
    usb_init_usblib(USBLIB, ep_int_in, ep_int_out);
}

void usb_cdcacm_disable(gpio_dev *disc_dev, uint8 disc_bit) {
    /* Turn off the interrupt and signal disconnect (see e.g. USB 2.0
     * spec, section 7.1.7.3). */
    nvic_irq_disable(NVIC_USB_LP_CAN_RX0);
    gpio_write_bit(disc_dev, disc_bit, 1);
}

void usb_cdcacm_putc(char ch) {
    while (!usb_cdcacm_tx((uint8*)&ch, 1))
        ;
}

/* This function is non-blocking.
 *
 * It copies data from a usercode buffer into the USB peripheral TX
 * buffer, and returns the number of bytes copied. */
uint32 usb_cdcacm_tx(const uint8* buf, uint32 len) {
    /* Last transmission hasn't finished, so abort. */
    if (n_unsent_bytes) {
        return 0;
    }

    /* We can only put USB_CDCACM_TX_EPSIZE bytes in the buffer. */
    if (len > USB_CDCACM_TX_EPSIZE) {
        len = USB_CDCACM_TX_EPSIZE;
    }

    /* Queue bytes for sending. */
    if (len) {
        usb_copy_to_pma(buf, len, USB_CDCACM_TX_ADDR);
        usb_set_ep_tx_count(USB_CDCACM_TX_ENDP, len);
        n_unsent_bytes = len;
        usb_set_ep_tx_stat(USB_CDCACM_TX_ENDP, USB_EP_STAT_TX_VALID);
    }

    return len;
}

uint32 usb_cdcacm_data_available(void) {
    return n_unread_bytes;
}

uint16 usb_cdcacm_get_pending() {
    return n_unsent_bytes;
}

/* Nonblocking byte receive.
 *
 * Copies up to len bytes from our private data buffer (*NOT* the PMA)
 * into buf and deq's the FIFO. */
uint32 usb_cdcacm_rx(uint8* buf, uint32 len) {
    /* Copy bytes to buffer. */
    uint32 n_copied = usb_cdcacm_peek(buf, len);

    /* Mark bytes as read. */
    n_unread_bytes -= n_copied;
    rx_offset += n_copied;

    /* If all bytes have been read, re-enable the RX endpoint, which
     * was set to NAK when the current batch of bytes was received. */
    if (n_unread_bytes == 0) {
        usb_set_ep_rx_count(USB_CDCACM_RX_ENDP, USB_CDCACM_RX_EPSIZE);
        usb_set_ep_rx_stat(USB_CDCACM_RX_ENDP, USB_EP_STAT_RX_VALID);
        rx_offset = 0;
    }

    return n_copied;
}

/* Nonblocking byte lookahead.
 *
 * Looks at unread bytes without marking them as read. */
uint32 usb_cdcacm_peek(uint8* buf, uint32 len) {
    int i;

    if (len > n_unread_bytes) {
        len = n_unread_bytes;
    }

    for (i = 0; i < len; i++) {
        buf[i] = vcomBufferRx[i + rx_offset];
    }

    return len;
}

uint8 usb_cdcacm_get_dtr() {
    return ((line_dtr_rts & USB_CDCACM_CONTROL_LINE_DTR) != 0);
}

uint8 usb_cdcacm_get_rts() {
    return ((line_dtr_rts & USB_CDCACM_CONTROL_LINE_RTS) != 0);
}

/*
 * Callbacks
 */

static void vcomDataTxCb(void) {
    /* The following assumes that all of the bytes we copied during
     * the last call to usb_cdcacm_tx were sent during the IN
     * transaction (this seems to be the case). */
    /* TODO find out why this is broken:
     * n_unsent_bytes = usb_get_ep_tx_count(USB_CDCACM_TX_ENDP); */
    n_unsent_bytes = 0;
}

static void vcomDataRxCb(void) {
    /* This following is safe since we set the RX endpoint to NAK
     * after each data packet received, and only set it to VALID when
     * all bytes have been read. */
    n_unread_bytes = usb_get_ep_rx_count(USB_CDCACM_RX_ENDP);
    usb_set_ep_rx_stat(USB_CDCACM_RX_ENDP, USB_EP_STAT_RX_NAK);
    usb_copy_from_pma((uint8*)vcomBufferRx, n_unread_bytes,
                      USB_CDCACM_RX_ADDR);

    if (rx_hook) {
        rx_hook(USB_CDCACM_HOOK_RX, 0);
    }
}

static uint8* vcomGetSetLineCoding(uint16 length) {
    if (length == 0) {
        pInformation->Ctrl_Info.Usb_wLength = sizeof(struct usb_line_coding);
    }
    return (uint8*)&line_coding;
}

static void usbInit(void) {
    pInformation->Current_Configuration = 0;

    USB_BASE->CNTR = USB_CNTR_FRES;

    USBLIB->irq_mask = 0;
    USB_BASE->CNTR = USBLIB->irq_mask;
    USB_BASE->ISTR = 0;
    USBLIB->irq_mask = USB_CNTR_RESETM | USB_CNTR_SUSPM | USB_CNTR_WKUPM;
    USB_BASE->CNTR = USBLIB->irq_mask;

    USB_BASE->ISTR = 0;
    USBLIB->irq_mask = USB_ISR_MSK;
    USB_BASE->CNTR = USBLIB->irq_mask;

    nvic_irq_enable(NVIC_USB_LP_CAN_RX0);
    USBLIB->state = USB_UNCONNECTED;
}

#define BTABLE_ADDRESS        0x00
static void usbReset(void) {
    pInformation->Current_Configuration = 0;

    /* current feature is current bmAttributes */
    pInformation->Current_Feature = (USB_CONFIG_ATTR_BUSPOWERED |
                                     USB_CONFIG_ATTR_SELF_POWERED);

    USB_BASE->BTABLE = BTABLE_ADDRESS;

    /* setup control endpoint 0 */
    usb_set_ep_type(USB_EP0, USB_EP_EP_TYPE_CONTROL);
    usb_set_ep_tx_stat(USB_EP0, USB_EP_STAT_TX_STALL);
    usb_set_ep_rx_addr(USB_EP0, USB_CDCACM_CTRL_RX_ADDR);
    usb_set_ep_tx_addr(USB_EP0, USB_CDCACM_CTRL_TX_ADDR);
    usb_clear_status_out(USB_EP0);

    usb_set_ep_rx_count(USB_EP0, pProperty->MaxPacketSize);
    usb_set_ep_rx_stat(USB_EP0, USB_EP_STAT_RX_VALID);

    /* setup management endpoint 1  */
    usb_set_ep_type(USB_CDCACM_MANAGEMENT_ENDP, USB_EP_EP_TYPE_INTERRUPT);
    usb_set_ep_tx_addr(USB_CDCACM_MANAGEMENT_ENDP,
                       USB_CDCACM_MANAGEMENT_ADDR);
    usb_set_ep_tx_stat(USB_CDCACM_MANAGEMENT_ENDP, USB_EP_STAT_TX_NAK);
    usb_set_ep_rx_stat(USB_CDCACM_MANAGEMENT_ENDP, USB_EP_STAT_RX_DISABLED);

    /* TODO figure out differences in style between RX/TX EP setup */

    /* set up data endpoint OUT (RX) */
    usb_set_ep_type(USB_CDCACM_RX_ENDP, USB_EP_EP_TYPE_BULK);
    usb_set_ep_rx_addr(USB_CDCACM_RX_ENDP, USB_CDCACM_RX_ADDR);
    usb_set_ep_rx_count(USB_CDCACM_RX_ENDP, USB_CDCACM_RX_EPSIZE);
    usb_set_ep_rx_stat(USB_CDCACM_RX_ENDP, USB_EP_STAT_RX_VALID);

    /* set up data endpoint IN (TX)  */
    usb_set_ep_type(USB_CDCACM_TX_ENDP, USB_EP_EP_TYPE_BULK);
    usb_set_ep_tx_addr(USB_CDCACM_TX_ENDP, USB_CDCACM_TX_ADDR);
    usb_set_ep_tx_stat(USB_CDCACM_TX_ENDP, USB_EP_STAT_TX_NAK);
    usb_set_ep_rx_stat(USB_CDCACM_TX_ENDP, USB_EP_STAT_RX_DISABLED);

    USBLIB->state = USB_ATTACHED;
    SetDeviceAddress(0);

    /* Reset the RX/TX state */
    n_unread_bytes = 0;
    n_unsent_bytes = 0;
    rx_offset = 0;
}

static RESULT usbDataSetup(uint8 request) {
    uint8* (*CopyRoutine)(uint16) = 0;

    if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT)) {
        switch (request) {
        case USB_CDCACM_GET_LINE_CODING:
            CopyRoutine = vcomGetSetLineCoding;
            last_request = USB_CDCACM_GET_LINE_CODING;
            break;
        case USB_CDCACM_SET_LINE_CODING:
            CopyRoutine = vcomGetSetLineCoding;
            last_request = USB_CDCACM_SET_LINE_CODING;
            break;
        default:
            break;
        }

        /* Call the user hook. */
        if (iface_setup_hook) {
            uint8 req_copy = request;
            iface_setup_hook(USB_CDCACM_HOOK_IFACE_SETUP, &req_copy);
        }
    }

    if (CopyRoutine == NULL) {
        return USB_UNSUPPORT;
    }

    pInformation->Ctrl_Info.CopyData = CopyRoutine;
    pInformation->Ctrl_Info.Usb_wOffset = 0;
    (*CopyRoutine)(0);
    return USB_SUCCESS;
}

static RESULT usbNoDataSetup(uint8 request) {
    RESULT ret = USB_UNSUPPORT;
    uint8 new_signal;

    if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT)) {
        switch (request) {
        case USB_CDCACM_SET_COMM_FEATURE:
            /* We support set comm. feature, but don't handle it. */
            ret = USB_SUCCESS;
            break;
        case USB_CDCACM_SET_CONTROL_LINE_STATE:
            /* Track changes to DTR and RTS. */
            new_signal = (pInformation->USBwValues.bw.bb0 &
                          (USB_CDCACM_CONTROL_LINE_DTR |
                           USB_CDCACM_CONTROL_LINE_RTS));
            line_dtr_rts = new_signal & 0x03;
            ret = USB_SUCCESS;
            break;
        }

        /* Call the user hook. */
        if (iface_setup_hook) {
            uint8 req_copy = request;
            iface_setup_hook(USB_CDCACM_HOOK_IFACE_SETUP, &req_copy);
        }
    }
    return ret;
}

static RESULT usbGetInterfaceSetting(uint8 interface, uint8 alt_setting) {
    if (alt_setting > 0) {
        return USB_UNSUPPORT;
    } else if (interface > 1) {
        return USB_UNSUPPORT;
    }

    return USB_SUCCESS;
}

static uint8* usbGetDeviceDescriptor(uint16 length) {
    return Standard_GetDescriptorData(length, &Device_Descriptor);
}

static uint8* usbGetConfigDescriptor(uint16 length) {
    return Standard_GetDescriptorData(length, &Config_Descriptor);
}

static uint8* usbGetStringDescriptor(uint16 length) {
    uint8 wValue0 = pInformation->USBwValue0;

    if (wValue0 > 2) {
        return NULL;
    }
    return Standard_GetDescriptorData(length, &String_Descriptor[wValue0]);
}

static void usbSetConfiguration(void) {
    if (pInformation->Current_Configuration != 0) {
        USBLIB->state = USB_CONFIGURED;
    }
}

static void usbSetDeviceAddress(void) {
    USBLIB->state = USB_ADDRESSED;
}