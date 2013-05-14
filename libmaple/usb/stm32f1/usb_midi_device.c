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
      defined(BOARD_maple_mini) || defined(BOARD_maple_midi) || defined(BOARD_maple_native))
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
 * Mark these weak so they can be overriden to implement other USB
 * functionality.
 */

#define NUM_ENDPTS                0x04
__weak DEVICE Device_Table = {
    .Total_Endpoint      = NUM_ENDPTS,
    .Total_Configuration = 1
};

#define MAX_PACKET_SIZE            0x40  /* 64B, maximum for USB FS Devices */
__weak DEVICE_PROP Device_Property = {
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

__weak USER_STANDARD_REQUESTS User_Standard_Requests = {
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

/* ------------------------------------------------------------------------------------------
 * 
 *                                    Minimal Sysex Handler
 *
 *-------------------------------------------------------------------------------------------
 *     The idea is to provide enough sysex functionality to id the device, and reset it.
 *------------------------------------------------------------------------------------------*/


const uint8 standardIDResponse[]={
    MIDIv1_SYSEX_START,
    USYSEX_NON_REAL_TIME,
    USYSEX_ALL_CHANNELS,
    USYSEX_GENERAL_INFO,
    USYSEX_GI_ID_RESPONSE,
    LEAFLABS_MMA_VENDOR_1,
    LEAFLABS_MMA_VENDOR_2, // extended ID
    LEAFLABS_MMA_VENDOR_3, // extended ID
    1, // family #1
    2, // family #2
    1, // part   #1
    2, // part   #2
    0, // version 1
    0, // version 2
    1, // version 3
    '!', // lgl compatible
    MIDIv1_SYSEX_END
};

#define STANDARD_ID_RESPONSE_LENGTH (sizeof(standardIDResponse))

typedef enum  {NOT_IN_SYSEX=0,COULD_BE_MY_SYSEX,YUP_ITS_MY_SYSEX,ITS_NOT_MY_SYSEX} sysexStates;
volatile uint8 sysexBuffer[MAX_SYSEX_LENGTH];
volatile sysexStates sysexState;
volatile int sysexFinger=0;
volatile uint8 myMidiChannel = DEFAULT_MIDI_CHANNEL;
volatile uint8 myMidiDevice = DEFAULT_MIDI_DEVICE;
volatile uint8 myMidiCable = DEFAULT_MIDI_CABLE;
volatile uint8 myMidiID[] = { LEAFLABS_MMA_VENDOR_1,LEAFLABS_MMA_VENDOR_2,LEAFLABS_MMA_VENDOR_3,0};

/*
 0xF0  SysEx
 0x??  LEAFLABS_MMA_VENDOR_1
 0x??  LEAFLABS_MMA_VENDOR_2
 
 0x??  LEAFLABS_MMA_VENDOR_3
 0x10  LGL_DEVICE_NUMBER
 0xLE  CMD: REBOOT
 
 0xf7  EOSysEx
 */
#define STACK_TOP 0x20000800
#define EXC_RETURN 0xFFFFFFF9
#define DEFAULT_CPSR 0x61000000
#define RESET_DELAY 100000
static void wait_reset(void) {
    delay_us(RESET_DELAY);
    nvic_sys_reset();
}

/* -----------------------------------------------------------------------------dealWithItQuickly()
 * Note: at this point we have established that the sysex belongs to us.
 * So we need to respond to any generic requests like information requests.
 * We also need to handle requests which are meant for us. At the moment this is just the
 * reset request.
 *
 */
void dealWithItQuickly(){
    switch (sysexBuffer[1]) {
        case USYSEX_NON_REAL_TIME:
            switch (sysexBuffer[3]) {
                case USYSEX_GENERAL_INFO:
                    if (sysexBuffer[4]==USYSEX_GI_ID_REQUEST) {
                        //usb_midi_tx((uint32 *) standardIDResponse, STANDARD_ID_RESPONSE_LENGTH);
                        usb_midi_send_sysex(standardIDResponse,17);
                    }
            }
        case USYSEX_REAL_TIME:
            break;
        case LEAFLABS_MMA_VENDOR_1:
            if (sysexBuffer[5]==LGL_RESET_CMD) {
                uintptr_t target = (uintptr_t)wait_reset | 0x1;
                asm volatile("mov r0, %[stack_top]      \n\t" // Reset stack
                             "mov sp, r0                \n\t"
                             "mov r0, #1                \n\t"
                             "mov r1, %[target_addr]    \n\t"
                             "mov r2, %[cpsr]           \n\t"
                             "push {r2}                 \n\t" // Fake xPSR
                             "push {r1}                 \n\t" // PC target addr
                             "push {r0}                 \n\t" // Fake LR
                             "push {r0}                 \n\t" // Fake R12
                             "push {r0}                 \n\t" // Fake R3
                             "push {r0}                 \n\t" // Fake R2
                             "push {r0}                 \n\t" // Fake R1
                             "push {r0}                 \n\t" // Fake R0
                             "mov lr, %[exc_return]     \n\t"
                             "bx lr"
                             :
                             : [stack_top] "r" (STACK_TOP),
                             [target_addr] "r" (target),
                             [exc_return] "r" (EXC_RETURN),
                             [cpsr] "r" (DEFAULT_CPSR)
                             : "r0", "r1", "r2");
                /* Can't happen. */
                ASSERT_FAULT(0);
                
            }
            
        default:
            break;
    }
    ;//turn the led on?
}

/* ----------------------------------------------------------------------usb_minimal_sysex_handler()
 * The idea here is to identify which Sysex's belong to us and deal with them.
 */
void usb_minimal_sysex_handler(uint32 *midiBufferRx, uint32 *rx_offset, uint32 *n_unread_packets) {
    USB_MIDI_Event_Packet * midiPackets = (USB_MIDI_Event_Packet *) (midiBufferRx+(*rx_offset));
    uint8 nPackets=((*n_unread_packets)-(*rx_offset));
    uint32 cPacket;
    uint8 soPackets=0;
    /********************************* ACHTUNG! ignores usbmidi cable ********************************/
    USB_MIDI_Event_Packet e;
    for (cPacket=0;cPacket<nPackets;cPacket++){
        e=midiPackets[cPacket];
        if (!CIN_IS_SYSEX(e.packet.cin)) {
            continue;
        } // else {
        if (!soPackets) {
            soPackets=cPacket;
        }
        if ((sysexState==YUP_ITS_MY_SYSEX) && ((sysexFinger+3)>=MAX_SYSEX_LENGTH)){
            sysexState=ITS_NOT_MY_SYSEX;  //eisenhower policy. Even if its mine I cant deal with it.
        }
        switch (e.packet.cin) {
            case CIN_SYSEX:
                switch (sysexState) {
                    case NOT_IN_SYSEX : // new sysex.
                        sysexFinger=0;
                        if (e.packet.midi0 == MIDIv1_SYSEX_START) {
                            if (e.packet.midi1==USYSEX_REAL_TIME
                                ||e.packet.midi1==USYSEX_NON_REAL_TIME) {
                                if ((e.packet.midi2==myMidiChannel)
                                    ||(e.packet.midi2==USYSEX_ALL_CHANNELS)
                                    ) {
                                    sysexState=YUP_ITS_MY_SYSEX;
                                    sysexBuffer[sysexFinger++]=MIDIv1_SYSEX_START;
                                    sysexBuffer[sysexFinger++]=e.packet.midi1;
                                    sysexBuffer[sysexFinger++]=e.packet.midi2;
                                    break;
                                }
                            } else if ((e.packet.midi1==myMidiID[0])
                                       && (e.packet.midi2==myMidiID[1])
                                       ){
                                sysexState=COULD_BE_MY_SYSEX;
                                sysexBuffer[sysexFinger++]=MIDIv1_SYSEX_START;
                                sysexBuffer[sysexFinger++]=e.packet.midi1;
                                sysexBuffer[sysexFinger++]=e.packet.midi2;
                                break;
                            }
                        }
                        break;
                        
                    case COULD_BE_MY_SYSEX:
                        if (e.packet.midi0==myMidiID[2]) {
                            sysexState=YUP_ITS_MY_SYSEX;
                            sysexBuffer[sysexFinger++]=e.packet.midi0;
                            sysexBuffer[sysexFinger++]=e.packet.midi1;
                            sysexBuffer[sysexFinger++]=e.packet.midi2;
                        } else {
                            sysexState=ITS_NOT_MY_SYSEX;
                            sysexFinger=0;
                        }
                        break;
                    default:
                        break;
                        
                }
                
                break;
            case CIN_SYSEX_ENDS_IN_1:
            case CIN_SYSEX_ENDS_IN_2:
            case CIN_SYSEX_ENDS_IN_3:
                sysexBuffer[sysexFinger++]=e.packet.midi0;
                sysexBuffer[sysexFinger++]=e.packet.midi1;
                sysexBuffer[sysexFinger++]=e.packet.midi2;
                if (sysexState==YUP_ITS_MY_SYSEX) {
                    if(cPacket>=(*n_unread_packets)){
                        *n_unread_packets = soPackets;
                        *rx_offset = soPackets;
                    } else {
                        uint32 s;
                        uint32 d;
                        for (s = cPacket, d=soPackets;
                             ((*n_unread_packets) && (s <= USB_MIDI_RX_EPSIZE/4));
                             d++,s++
                             ) {
                            midiPackets[d]=midiPackets[s];
                            (*n_unread_packets)--;
                            (*rx_offset)++;
                            
                        }
                        // we need to reset the for loop variables to re process remaining data.
                        nPackets=((*n_unread_packets)-(*rx_offset));
                        cPacket=(*rx_offset);
                    }
                    dealWithItQuickly();
                    
                }
                sysexFinger=0;
                sysexState=NOT_IN_SYSEX;
                
                break;
            default:
                return;
        }
        //}
        
        
        
    }
    // its our sysex and we will cry if we want to
    return;
}
/* ---------------------------------------------------------------------------usb_midi_send_sysex()
 * This is currently more expensive memory wise than I would like and will only send sysex
 * the sysexex can be complete or just the body. 
 * all characters between start and end of the sysex will be stripped of the hi bit.
 */

#define SYSEX_OUT_BUFFER_LENGTH (MAX_SYSEX_LENGTH*7/3)+4
static volatile uint8 sysexOutBuffer[SYSEX_OUT_BUFFER_LENGTH];

uint32 usb_midi_send_sysex(uint8 *sysex, uint32 bytes2send) {
    int j=1;
    uint32 i=0;
    uint32 buffered=0;
    
    if (bytes2send && (  sysex[bytes2send-1] != MIDIv1_SYSEX_END ) ) {
        bytes2send++;  // make room for end of sysex
    }
    if (bytes2send > (SYSEX_OUT_BUFFER_LENGTH - 4)) {
        return 0; // if its to big then fahgetaboutit
    }
    while (bytes2send > 3 /*&& buffered<=SYSEX_OUT_BUFFER_LENGTH */) {
        sysexOutBuffer[buffered++]=CIN_SYSEX;
        if (buffered==1) {
            sysexOutBuffer[buffered++]=MIDIv1_SYSEX_START;
            if (sysex[i]==MIDIv1_SYSEX_START){
                i++; // note: output count will be off by 1 for incomplete sysex messages
            }
        } else {
            sysexOutBuffer[buffered++]=sysex[i++] & 0x7f;
        }
        sysexOutBuffer[buffered++]=sysex[i++] & 0x7f;
        sysexOutBuffer[buffered++]=sysex[i++] & 0x7f;
        //usb_midi_tx((uint32 *)sysexOutBuffer, 1);
        bytes2send-=3;
    }
    //if (bytes2send>3) { // this should already be covered above.
    //    return 0;
    //}
    sysexOutBuffer[buffered++] = CIN_SYSEX+bytes2send;
    j=3;
    while((bytes2send--)>1) {
        sysexOutBuffer[buffered++]= sysex[i++] & 0x7f;
        j--;
    }
    
    sysexOutBuffer[buffered++] = MIDIv1_SYSEX_END;
    i++;
    j--;
    
    while (j--) {
        sysexOutBuffer[buffered++]=0; // pad packets (probably unnecisary)
        // if we eliminate this then must add j to math below.
    }
    usb_midi_tx((uint32 *)sysexOutBuffer, buffered/4);
    return i;
}



#define DEBUGGER_MMA_ID1 125
#define DEBUGGER_MMA_ID2 51
#define DEBUGGER_MMA_ID3 51

uint8 debugSysexPrelude[]= {
    MIDIv1_SYSEX_START,
    DEBUGGER_MMA_ID1,
    DEBUGGER_MMA_ID2, //2
    //--
    DEBUGGER_MMA_ID3,
    LEAFLABS_MMA_VENDOR_1,
    LEAFLABS_MMA_VENDOR_2,//length       5
    //---
    LEAFLABS_MMA_VENDOR_3,//length
    0x00,//channel or id
    0x00,//severity 8
    //---
};

uint32 usb_midi_send_debug_string(uint8 *string, uint32 bytes2send) {

    if (bytes2send<1)
        return 0;
    
    int j=1;
    uint32 i=0;
    uint32 buffered=0;
    for (i=0; i<9; i++) {
        sysexOutBuffer[buffered++]= CIN_SYSEX;
        sysexOutBuffer[buffered++]=debugSysexPrelude[i++];
        sysexOutBuffer[buffered++]=debugSysexPrelude[i++];
        sysexOutBuffer[buffered++]=debugSysexPrelude[i++];
    }
    j=0;i=0;

    bytes2send++; // make room for end of sysex
    
    if ((bytes2send + buffered) > (SYSEX_OUT_BUFFER_LENGTH - 4)) {
        return 0; // if its to big then fahgetaboutit
    }
    while (bytes2send > 3 /*&& buffered<=SYSEX_OUT_BUFFER_LENGTH */) {
        sysexOutBuffer[buffered++]=CIN_SYSEX;
        sysexOutBuffer[buffered++]=string[i++] & 0x7f;
        sysexOutBuffer[buffered++]=string[i++] & 0x7f;
        sysexOutBuffer[buffered++]=string[i++] & 0x7f;
        bytes2send-=3;
    }
    //if (bytes2send>3) { // this should already be covered above.
    //    return 0;
    //}
    sysexOutBuffer[buffered++] = CIN_SYSEX+bytes2send;
    j=3;
    while((bytes2send--)>1) {
        sysexOutBuffer[buffered++]= string[i++] & 0x7f;
        j--;
    }
    
    sysexOutBuffer[buffered++] = MIDIv1_SYSEX_END;
    //i++; // returns sent minux 
    j--;
    
    while (j--) {
        sysexOutBuffer[buffered++]=0; // pad packets (probably unnecisary)
        // if we eliminate this then must add j to math below.
    }
    usb_midi_tx((uint32 *)sysexOutBuffer, buffered/4);
    return i;
}





