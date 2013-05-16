/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2011 LeafLabs LLC.
 * Copyright (c) 2013 Magnus Lundin.
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
 * @file libmaple/usb/stm32f1/usb_midi_device.c
 * @brief USB MIDI.
 *
 * FIXME: this works on the STM32F1 USB peripherals, and probably no
 * place else. Nonportable bits really need to be factored out, and
 * the result made cleaner.
 */

#define USETXBUFFER

#include <libmaple/usb_midi_device.h>
#include <libmaple/midi_specs.h>
#include <libmaple/usb.h>
#include <libmaple/nvic.h>
#include <libmaple/delay.h>

/* Private headers */
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
#warning USB MIDI relies on LeafLabs board-specific configuration.\
    You may have problems on non-LeafLabs boards.
#endif

/* Descriptor handling functions from usb_midi_descr.c */
extern uint8* usbGetMidiDeviceDescriptor(uint16 length);
extern uint8* usbGetMidiConfigDescriptor(uint16 length);
extern uint8* usbGetMidiStringDescriptor(uint16 length);

/* interrupt callbacks */
static void midiDataTxCb(void);
static void midiDataRxCb(void);

static void usbInit(void);
static void usbReset(void);
static RESULT usbDataSetup(uint8 request);
static RESULT usbNoDataSetup(uint8 request);
static RESULT usbGetInterfaceSetting(uint8 interface, uint8 alt_setting);
static void usbSetConfiguration(void);
static void usbSetDeviceAddress(void);


/* I/O state */
/* I/O state */

/* Received data */
static volatile uint32 midiBufferRx[USB_MIDI_RX_EPSIZE/4];
/* Read index into midiBufferRx */
static volatile uint32 rx_offset = 0;
/* Transmit data */
static volatile uint32 midiBufferTx[USB_MIDI_TX_EPSIZE/4];
/* Write index into midiBufferTx */
static volatile uint32 tx_offset = 0;
/* Number of bytes left to transmit */
static volatile uint32 n_unsent_packets = 0;
/* Are we currently sending an IN packet? */
static volatile uint8 transmitting = 0;
/* Number of unread bytes */
static volatile uint32 n_unread_packets = 0;
/*
 * Endpoint callbacks
 */

static void (*ep_int_in[7])(void) =
    {midiDataTxCb,
     NOP_Process,
     NOP_Process,
     NOP_Process,
     NOP_Process,
     NOP_Process,
     NOP_Process};

static void (*ep_int_out[7])(void) =
    {NOP_Process,
     midiDataRxCb,
     NOP_Process,
     NOP_Process,
     NOP_Process,
     NOP_Process,
     NOP_Process};

/*
 * Globals required by usb_lib/
 *
 * These will override _weak defines in usb_cdcasm.c
 *
 */

#if defined(USB_TYPE) && (USB_TYPE==USB_MIDI)

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
    .GetDeviceDescriptor         = usbGetMidiDeviceDescriptor,
    .GetConfigDescriptor         = usbGetMidiConfigDescriptor,
    .GetStringDescriptor         = usbGetMidiStringDescriptor,
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
 * MIDI interface
 */

void usb_midi_enable(gpio_dev *disc_dev, uint8 disc_bit) {
    /* Present ourselves to the host. Writing 0 to "disc" pin must
     * pull USB_DP pin up while leaving USB_DM pulled down by the
     * transceiver. See USB 2.0 spec, section 7.1.7.3. */
    gpio_set_mode(disc_dev, disc_bit, GPIO_OUTPUT_PP);
    gpio_write_bit(disc_dev, disc_bit, 0);

    /* Initialize the USB peripheral. */
    usb_init_usblib(USBLIB, ep_int_in, ep_int_out);
}

void usb_midi_disable(gpio_dev *disc_dev, uint8 disc_bit) {
    /* Turn off the interrupt and signal disconnect (see e.g. USB 2.0
     * spec, section 7.1.7.3). */
    nvic_irq_disable(NVIC_USB_LP_CAN_RX0);
    gpio_write_bit(disc_dev, disc_bit, 1);
}

//void usb_midi_putc(char ch) {
//    while (!usb_midi_tx((uint8*)&ch, 1))
//        ;
//}

/* This function is non-blocking.
 *
 * It copies data from a usercode buffer into the USB peripheral TX
 * buffer, and returns the number of bytes copied. */
uint32 usb_midi_tx(const uint32* buf, uint32 packets) {
    uint32 bytes=packets*4;
    /* Last transmission hasn't finished, so abort. */
    if (usb_midi_is_transmitting()) {
		/* Copy to TxBuffer */
		
        return 0;  /* return len */
    }
    
    /* We can only put USB_MIDI_TX_EPSIZE bytes in the buffer. */
    if (bytes > USB_MIDI_TX_EPSIZE) {
        bytes = USB_MIDI_TX_EPSIZE;
        packets=bytes/4;
    }
    
    /* Queue bytes for sending. */
    if (packets) {
        usb_copy_to_pma((uint8 *)buf, bytes, USB_MIDI_TX_ADDR);
    }
    // We still need to wait for the interrupt, even if we're sending
    // zero bytes. (Sending zero-size packets is useful for flushing
    // host-side buffers.)
    usb_set_ep_tx_count(USB_MIDI_TX_ENDP, bytes);
    n_unsent_packets = packets;
    transmitting = 1;
    usb_set_ep_tx_stat(USB_MIDI_TX_ENDP, USB_EP_STAT_TX_VALID);
    
    return packets;
}

/* 
 *     Theese functions replaces EP double buffering:
 * 
 *     usb_midi_tx_buffered(const uint8* buf, uint32 len)
 *     usb_midi_tx_send_buffer()
 * 
 *     TODO: Use the modules EP double buffering instead
 * 
 *  */
uint32 usb_midi_tx_buffered(const uint8* buf, uint32 len) {
    if (usb_midi_is_transmitting()) {
		uint8 count;
		/* Copy to TxBuffer */
		/* Disable USB EP interrupts */
		USB_BASE->CNTR = USB_ISR_MSK&~USB_CNTR_CTRM;
		if (len>(USB_MIDI_TX_EPSIZE-tx_offset)) len = USB_MIDI_TX_EPSIZE-tx_offset;
		for (count=0;count<len;count++)
		{
    		midiBufferTx[tx_offset+count]=buf[count];
    	}
		tx_offset += len;
		/* Reenable USB EP interrupts */
		USB_BASE->CNTR = USB_ISR_MSK&~USB_CNTR_CTRM;
        return len;  /* return len */
    }

    /* We can only put USB_MIDI_TX_EPSIZE bytes in the buffer. */
    if (len > USB_MIDI_TX_EPSIZE) {
        len = USB_MIDI_TX_EPSIZE;
    }

    /* Queue bytes for sending. */
    if (len) {
        usb_copy_to_pma(buf, len, USB_MIDI_TX_ADDR);
    }
    // We still need to wait for the interrupt, even if we're sending
    // zero bytes. (Sending zero-size packets is useful for flushing
    // host-side buffers.)
    usb_set_ep_tx_count(USB_MIDI_TX_ENDP, len);
    n_unsent_bytes = len;
    transmitting = 1;
    usb_set_ep_tx_stat(USB_MIDI_TX_ENDP, USB_EP_STAT_TX_VALID);

    return len;
}

#ifdef USETXBUFFER
uint32 usb_midi_tx_send_buffer() {
    if (usb_midi_is_transmitting()) {
        return 0;
    }
    /* Queue bytes for sending. */
    if (tx_offset) {
        usb_copy_to_pma((const uint8 *)midiBufferTx, tx_offset, USB_MIDI_TX_ADDR);
    }
    else return 0;
    // We still need to wait for the interrupt, even if we're sending
    // zero bytes. (Sending zero-size packets is useful for flushing
    // host-side buffers.)
    usb_set_ep_tx_count(USB_MIDI_TX_ENDP, tx_offset);
    n_unsent_bytes = tx_offset;
    tx_offset = 0;
    transmitting = 1;
    usb_set_ep_tx_stat(USB_MIDI_TX_ENDP, USB_EP_STAT_TX_VALID);

    return n_unsent_bytes;
}
#endif

uint32 usb_midi_data_available(void) {
    return n_unread_packets;
}

uint8 usb_midi_is_transmitting(void) {
    return transmitting;
}

uint16 usb_midi_get_pending(void) {
    return n_unsent_packets;
}

/* Nonblocking byte receive.
 *
 * Copies up to len bytes from our private data buffer (*NOT* the PMA)
 * into buf and deq's the FIFO. */
uint32 usb_midi_rx(uint32* buf, uint32 packets) {
    /* Copy bytes to buffer. */
    uint32 n_copied = usb_midi_peek(buf, packets);
    
    /* Mark bytes as read. */
    n_unread_packets -= n_copied;
    rx_offset += n_copied;
    
    /* If all bytes have been read, re-enable the RX endpoint, which
     * was set to NAK when the current batch of bytes was received. */
    if (n_unread_packets == 0) {
        usb_set_ep_rx_count(USB_MIDI_RX_ENDP, USB_MIDI_RX_EPSIZE);
        usb_set_ep_rx_stat(USB_MIDI_RX_ENDP, USB_EP_STAT_RX_VALID);
        rx_offset = 0;
    }
    
    return n_copied;
}

/* Nonblocking byte lookahead.
 *
 * Looks at unread bytes without marking them as read. */
uint32 usb_midi_peek(uint32* buf, uint32 packets) {
    int i;
    if (packets > n_unread_packets) {
        packets = n_unread_packets;
    }
    
    for (i = 0; i < packets; i++) {
        buf[i] = midiBufferRx[i + rx_offset];
    }
    
    return packets;
}

/*
 * Callbacks
 */

static void midiDataTxCb(void) {
    n_unsent_packets = 0;
    transmitting = 0;
#ifdef USETXBUFFER
    usb_midi_tx_send_buffer();
#endif
}

static void midiDataRxCb(void) {
    
    usb_set_ep_rx_stat(USB_MIDI_RX_ENDP, USB_EP_STAT_RX_NAK);
    n_unread_packets = usb_get_ep_rx_count(USB_MIDI_RX_ENDP) / 4;
    /* This copy won't overwrite unread bytes, since we've set the RX
     * endpoint to NAK, and will only set it to VALID when all bytes
     * have been read. */
    
    usb_copy_from_pma((uint8*)midiBufferRx, n_unread_packets * 4,
                      USB_MIDI_RX_ADDR);
    
    usb_minimal_sysex_handler(midiBufferRx,&rx_offset,&n_unread_packets);
    
    if (n_unread_packets == 0) {
        usb_set_ep_rx_count(USB_MIDI_RX_ENDP, USB_MIDI_RX_EPSIZE);
        usb_set_ep_rx_stat(USB_MIDI_RX_ENDP, USB_EP_STAT_RX_VALID);
        rx_offset = 0;
    }
    
}

/* NOTE: Nothing specific to this device class in this function, but depenedent on the device, move to usb_lib or stm32fxx*/
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
    usb_set_ep_rx_addr(USB_EP0, USB_MIDI_CTRL_RX_ADDR);
    usb_set_ep_tx_addr(USB_EP0, USB_MIDI_CTRL_TX_ADDR);
    usb_clear_status_out(USB_EP0);

    usb_set_ep_rx_count(USB_EP0, pProperty->MaxPacketSize);
    usb_set_ep_rx_stat(USB_EP0, USB_EP_STAT_RX_VALID);

    /* TODO figure out differences in style between RX/TX EP setup */

    /* set up data endpoint OUT (RX) */
    usb_set_ep_type(USB_MIDI_RX_ENDP, USB_EP_EP_TYPE_BULK);
    usb_set_ep_rx_addr(USB_MIDI_RX_ENDP, USB_MIDI_RX_ADDR);
    usb_set_ep_rx_count(USB_MIDI_RX_ENDP, USB_MIDI_RX_EPSIZE);
    usb_set_ep_rx_stat(USB_MIDI_RX_ENDP, USB_EP_STAT_RX_VALID);

    /* set up data endpoint IN (TX)  */
    usb_set_ep_type(USB_MIDI_TX_ENDP, USB_EP_EP_TYPE_BULK);
    usb_set_ep_tx_addr(USB_MIDI_TX_ENDP, USB_MIDI_TX_ADDR);
    usb_set_ep_tx_stat(USB_MIDI_TX_ENDP, USB_EP_STAT_TX_NAK);
    usb_set_ep_rx_stat(USB_MIDI_TX_ENDP, USB_EP_STAT_RX_DISABLED);

    USBLIB->state = USB_ATTACHED;
    SetDeviceAddress(0);

    /* Reset the RX/TX state */
    n_unread_packets = 0;
    n_unsent_packets = 0;
    rx_offset = 0;
    tx_offset = 0;
    transmitting = 0;
}

static RESULT usbDataSetup(uint8 request) {
    uint8* (*CopyRoutine)(uint16) = 0;

    if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT)) {

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

    if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT)) {
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

static void usbSetConfiguration(void) {
    if (pInformation->Current_Configuration != 0) {
        USBLIB->state = USB_CONFIGURED;
    }
}

static void usbSetDeviceAddress(void) {
    USBLIB->state = USB_ADDRESSED;
}

#endif
