/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2011 LeafLabs LLC.
 * Copyright (c) 2013 Magnus Lundin.
 * Rework for multiple MIDI interfaces Copyright (c) 2020 Donna Whisnant
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

#include <string.h>

#include "usb_generic.h"
#include "usb_midi_device.h"
#include "MidiSpecs.h"
#if 0
#include "MinSysex.h"
#endif

#include <libmaple/usb.h>
#include <libmaple/delay.h>

/* Private headers */
#include <usb_lib_globals.h>
#include <usb_reg_map.h>

/* usb_lib headers */
#include <usb_type.h>
#include <usb_core.h>
#include <usb_def.h>


CMIDIDevices g_MIDIDevices;
CMIDIDevices::CMIDIDevice CMIDIDevices::m_ports[CMIDIDevices::MIDI_PORT_COUNT];

extern "C" {

/*
 * Descriptors
 */

typedef struct {
	MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_1 = {
			.bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
			.bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
			.SubType            = MIDI_IN_JACK,
			.bJackType          = MIDI_JACK_EMBEDDED,
			.bJackId            = 0x01,			// Patch
			.iJack              = 0x00,
		};

	MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_2 = {
			.bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
			.bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
			.SubType            = MIDI_IN_JACK,
			.bJackType          = MIDI_JACK_EXTERNAL,
			.bJackId            = 0x02,			// Patch
			.iJack              = 0x00,
		};

	MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_3 = {
			.bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
			.bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
			.SubType            = MIDI_OUT_JACK,
			.bJackType          = MIDI_JACK_EMBEDDED,
			.bJackId            = 0x03,			// Patch
			.bNrInputPins       = 0x01,
			.baSourceId         = {0x02},		// Patch
			.baSourcePin        = {0x01},		// Patch
			.iJack              = 0x00,
		};

	MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_4 = {
			.bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
			.bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
			.SubType            = MIDI_OUT_JACK,
			.bJackType          = MIDI_JACK_EXTERNAL,
			.bJackId            = 0x04,
			.bNrInputPins       = 0x01,
			.baSourceId         = {0x01},		// Patch
			.baSourcePin        = {0x01},		// Patch
			.iJack              = 0x00,
		};

	usb_descriptor_endpoint            DataOutEndpoint = {
			.bLength            = sizeof(usb_descriptor_endpoint),
			.bDescriptorType    = USB_DESCRIPTOR_TYPE_ENDPOINT,
			.bEndpointAddress   = (USB_DESCRIPTOR_ENDPOINT_OUT |
								 0), // PATCH: USB_MIDI_RX_ENDP
			.bmAttributes       = USB_EP_TYPE_BULK,
			.wMaxPacketSize     = 64, // PATCH
			.bInterval          = 0x00,
		};

	MS_CS_BULK_ENDPOINT_DESCRIPTOR(1)  MS_CS_DataOutEndpoint = {
			.bLength              = MS_CS_BULK_ENDPOINT_DESCRIPTOR_SIZE(1),
			.bDescriptorType      = USB_DESCRIPTOR_TYPE_CS_ENDPOINT,
			.SubType              = 0x01,
			.bNumEmbMIDIJack      = 0x01,
			.baAssocJackID        = {0x01},		// Patch
		  };

	usb_descriptor_endpoint            DataInEndpoint = {
			.bLength          = sizeof(usb_descriptor_endpoint),
			.bDescriptorType  = USB_DESCRIPTOR_TYPE_ENDPOINT,
			.bEndpointAddress = (USB_DESCRIPTOR_ENDPOINT_IN | 0), // PATCH: USB_MIDI_TX_ENDP
			.bmAttributes     = USB_EP_TYPE_BULK,
			.wMaxPacketSize   = 64, // PATCH
			.bInterval        = 0x00,
		};

	MS_CS_BULK_ENDPOINT_DESCRIPTOR(1)  MS_CS_DataInEndpoint = {
			.bLength              = MS_CS_BULK_ENDPOINT_DESCRIPTOR_SIZE(1),
			.bDescriptorType      = USB_DESCRIPTOR_TYPE_CS_ENDPOINT,
			.SubType              = 0x01,
			.bNumEmbMIDIJack      = 0x01,
			.baAssocJackID        = {0x03},		// Patch
		  };

} __packed midi_port_descriptor;

typedef struct {
//    usb_descriptor_config_header       Config_Header;
	/* Control Interface */
	usb_descriptor_interface           AC_Interface;
	AC_CS_INTERFACE_DESCRIPTOR(CMIDIDevices::MIDI_PORT_COUNT)      AC_CS_Interface;
	/* Control Interface */
	usb_descriptor_interface           MS_Interface;
	MS_CS_INTERFACE_DESCRIPTOR         MS_CS_Interface;
	midi_port_descriptor               MS_PORTS[CMIDIDevices::MIDI_PORT_COUNT];
} __packed usb_descriptor_config;

static const usb_descriptor_config usbMIDIDescriptor_Config = {
	/* .Config_Header = {
		.bLength              = sizeof(usb_descriptor_config_header),
		.bDescriptorType      = USB_DESCRIPTOR_TYPE_CONFIGURATION,
		.wTotalLength         = sizeof(usb_descriptor_config),
		.bNumInterfaces       = 0x02,
		.bConfigurationValue  = 0x01,
		.iConfiguration       = 0x00,
		.bmAttributes         = (USB_CONFIG_ATTR_BUSPOWERED |
								 USB_CONFIG_ATTR_SELF_POWERED),
		.bMaxPower            = MAX_POWER,
	}, */

	.AC_Interface = {
		.bLength            = sizeof(usb_descriptor_interface),
		.bDescriptorType    = USB_DESCRIPTOR_TYPE_INTERFACE,
		.bInterfaceNumber   = 0x00,		// PATCH
		.bAlternateSetting  = 0x00,
		.bNumEndpoints      = 0x00,
		.bInterfaceClass    = USB_INTERFACE_CLASS_AUDIO,
		.bInterfaceSubClass = USB_INTERFACE_AUDIOCONTROL,
		.bInterfaceProtocol = 0x00,
		.iInterface         = 0x00,
	},

	.AC_CS_Interface = {
		.bLength            = AC_CS_INTERFACE_DESCRIPTOR_SIZE(1),
		.bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
		.SubType            = 0x01,
		.bcdADC             = 0x0100,
		.wTotalLength       = AC_CS_INTERFACE_DESCRIPTOR_SIZE(1),
		.bInCollection      = 0x01,
		.baInterfaceNr      = {0x01},	// Patch
	},

	/* Control Interface */
	.MS_Interface = {
		.bLength            = sizeof(usb_descriptor_interface),
		.bDescriptorType    = USB_DESCRIPTOR_TYPE_INTERFACE,
		.bInterfaceNumber   = 0x01, // PATCH
		.bAlternateSetting  = 0x00,
		.bNumEndpoints      = 0x02 * CMIDIDevices::MIDI_PORT_COUNT,
		.bInterfaceClass    = USB_INTERFACE_CLASS_AUDIO,
		.bInterfaceSubClass = USB_INTERFACE_MIDISTREAMING,
		.bInterfaceProtocol = 0x00,
		.iInterface         = 0, // was 0x04
	},

	.MS_CS_Interface = {
		.bLength            = sizeof(MS_CS_INTERFACE_DESCRIPTOR),
		.bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
		.SubType            = 0x01,
		.bcdADC             = 0x0100,
		.wTotalLength       = sizeof(MS_CS_INTERFACE_DESCRIPTOR) +
							  (sizeof(MIDI_IN_JACK_DESCRIPTOR)
							   +sizeof(MIDI_IN_JACK_DESCRIPTOR)
							   +MIDI_OUT_JACK_DESCRIPTOR_SIZE(1)
							   +MIDI_OUT_JACK_DESCRIPTOR_SIZE(1)
							   +sizeof(usb_descriptor_endpoint)
							   +MS_CS_BULK_ENDPOINT_DESCRIPTOR_SIZE(1)
							   +sizeof(usb_descriptor_endpoint)
							   +MS_CS_BULK_ENDPOINT_DESCRIPTOR_SIZE(1)
							  ) * CMIDIDevices::MIDI_PORT_COUNT
								 /* 0x41-4 */,
	},

};

static USBEndpointInfo midiEndpoints[2 * CMIDIDevices::MIDI_PORT_COUNT] = {};

}	// extern "C"

void CMIDIDevices::getMIDIPartDescriptor(uint8* out)
{
	usb_descriptor_config *pDescriptorOut = reinterpret_cast<usb_descriptor_config *>(out);

	memcpy(pDescriptorOut, &usbMIDIDescriptor_Config, sizeof(usbMIDIDescriptor_Config));

	// patch to reflect where the part goes in the descriptor
	pDescriptorOut->AC_Interface.bInterfaceNumber += usbMIDIPart.startInterface;
	pDescriptorOut->AC_CS_Interface.baInterfaceNr[0] += usbMIDIPart.startInterface;
	pDescriptorOut->MS_Interface.bInterfaceNumber += usbMIDIPart.startInterface;
	for (int nPort = 0; nPort < MIDI_PORT_COUNT; ++nPort) {
		USBEndpointInfo *pRxEndpoint = &midiEndpoints[nPort*2+MIDI_ENDPOINT_OFFSET_RX];
		USBEndpointInfo *pTxEndpoint = &midiEndpoints[nPort*2+MIDI_ENDPOINT_OFFSET_TX];

		pDescriptorOut->MS_PORTS[nPort].DataOutEndpoint.bEndpointAddress += pRxEndpoint->address;
		pDescriptorOut->MS_PORTS[nPort].DataOutEndpoint.wMaxPacketSize = static_cast<uint16_t>(g_MIDIDevices.port(nPort).usb_midi_rxEPSize());
		pDescriptorOut->MS_PORTS[nPort].DataInEndpoint.bEndpointAddress += pTxEndpoint->address;
		pDescriptorOut->MS_PORTS[nPort].DataInEndpoint.wMaxPacketSize = static_cast<uint16_t>(g_MIDIDevices.port(nPort).usb_midi_txEPSize());

		pDescriptorOut->MS_PORTS[nPort].MIDI_IN_JACK_1.bJackId += (nPort*4);
		pDescriptorOut->MS_PORTS[nPort].MIDI_IN_JACK_2.bJackId += (nPort*4);
		pDescriptorOut->MS_PORTS[nPort].MIDI_OUT_JACK_3.bJackId += (nPort*4);
		pDescriptorOut->MS_PORTS[nPort].MIDI_OUT_JACK_4.bJackId += (nPort*4);

		pDescriptorOut->MS_PORTS[nPort].MIDI_OUT_JACK_3.baSourceId[0] += (nPort*2);
		pDescriptorOut->MS_PORTS[nPort].MIDI_OUT_JACK_4.baSourceId[0] += (nPort*2);

		pDescriptorOut->MS_PORTS[nPort].MS_CS_DataOutEndpoint.baAssocJackID[0] += (nPort*4);
		pDescriptorOut->MS_PORTS[nPort].MS_CS_DataInEndpoint.baAssocJackID[0] += (nPort*4);
	}
}

USBCompositePart usbMIDIPart = {
	.numInterfaces = 2,
	.numEndpoints = sizeof(midiEndpoints)/sizeof(*midiEndpoints),
	.startInterface = 0,
	.descriptorSize = sizeof(usbMIDIDescriptor_Config),
	.getPartDescriptor = g_MIDIDevices.getMIDIPartDescriptor,
	.usbInit = nullptr,
	.usbReset = g_MIDIDevices.usbMIDIReset,
	.usbSetConfiguration = nullptr,
	.usbClearFeature = nullptr,
	.clear = nullptr,
	.usbDataSetup = nullptr,
	.usbNoDataSetup = nullptr,
	.endpoints = midiEndpoints
};


// ------------------------------------------------------------------------------------------------

CMIDIDevices::CMIDIDevices()
{
	void (* arrRxCallbacks[MIDI_PORT_COUNT])(void) = {
		&CMIDIDevices::CMIDIDevice::midiDataRxCb<0>,
#if (NUM_MIDI_PORTS >= 2)
		&CMIDIDevices::CMIDIDevice::midiDataRxCb<1>,
#endif
#if (NUM_MIDI_PORTS >= 3)
		&CMIDIDevices::CMIDIDevice::midiDataRxCb<2>,
#endif
#if (NUM_MIDI_PORTS >= 4)
		&CMIDIDevices::CMIDIDevice::midiDataRxCb<3>,
#endif
#if (NUM_MIDI_PORTS >= 5)
		&CMIDIDevices::CMIDIDevice::midiDataRxCb<4>,
#endif
#if (NUM_MIDI_PORTS >= 6)
		&CMIDIDevices::CMIDIDevice::midiDataRxCb<5>,
#endif
#if (NUM_MIDI_PORTS >= 7)
		&CMIDIDevices::CMIDIDevice::midiDataRxCb<6>,
#endif
	};

	void (* arrTxCallbacks[MIDI_PORT_COUNT])(void) = {
		&CMIDIDevices::CMIDIDevice::midiDataTxCb<0>,
#if (NUM_MIDI_PORTS >= 2)
		&CMIDIDevices::CMIDIDevice::midiDataTxCb<1>,
#endif
#if (NUM_MIDI_PORTS >= 3)
		&CMIDIDevices::CMIDIDevice::midiDataTxCb<2>,
#endif
#if (NUM_MIDI_PORTS >= 4)
		&CMIDIDevices::CMIDIDevice::midiDataTxCb<3>,
#endif
#if (NUM_MIDI_PORTS >= 5)
		&CMIDIDevices::CMIDIDevice::midiDataTxCb<4>,
#endif
#if (NUM_MIDI_PORTS >= 6)
		&CMIDIDevices::CMIDIDevice::midiDataTxCb<5>,
#endif
#if (NUM_MIDI_PORTS >= 7)
		&CMIDIDevices::CMIDIDevice::midiDataTxCb<6>,
#endif
	};

	for (int i = 0; i < MIDI_PORT_COUNT; ++i) {
		port(i).m_pEndpoints = &midiEndpoints[i*2];

		midiEndpoints[i*2].callback = arrRxCallbacks[i];
		midiEndpoints[i*2].pmaSize = 64;		// patch
		midiEndpoints[i*2].type = USB_GENERIC_ENDPOINT_TYPE_BULK;
		midiEndpoints[i*2].tx = 0;

		midiEndpoints[i*2+1].callback = arrTxCallbacks[i];
		midiEndpoints[i*2+1].pmaSize = 64;		// patch
		midiEndpoints[i*2+1].type = USB_GENERIC_ENDPOINT_TYPE_BULK;
		midiEndpoints[i*2+1].tx = 1;
	}
}

void CMIDIDevices::usbMIDIReset(void)
{
	for (int i = 0; i < MIDI_PORT_COUNT; ++i) {
		g_MIDIDevices.port(i).usbMIDIReset();
	}
}

// ------------------------------------------------------------------------------------------------

void CMIDIDevices::CMIDIDevice::usb_midi_setTXEPSize(uint32_t size)
{
	size = (size+3)/4*4;
	if (size == 0 || size > 64)
		size = 64;
	midiEndpoints[1].pmaSize = size;
	m_txEPSize = size;
}

void CMIDIDevices::CMIDIDevice::usb_midi_setRXEPSize(uint32_t size)
{
	size = (size+3)/4*4;
	if (size == 0 || size > 64)
		size = 64;
	midiEndpoints[0].pmaSize = size;
	m_rxEPSize = size;
}

// ------------------------------------------------------------------------------------------------

/*
 * MIDI interface
 */

/* This function is non-blocking.
 *
 * It copies data from a usercode buffer into the USB peripheral TX
 * buffer, and returns the number of bytes copied. */
uint32_t CMIDIDevices::CMIDIDevice::usb_midi_tx(const uint32_t* buf, uint32_t packets)
{
	uint32_t bytes=packets*4;
	/* Last transmission hasn't finished, so abort. */
	if (usb_midi_is_transmitting()) return 0;  /* return len */

	/* We can only put m_txEPSize bytes in the buffer. */
	if (bytes > m_txEPSize) {
		bytes = m_txEPSize;
		packets=bytes/4;
	}

	/* Queue bytes for sending. */
	if (packets) {
		usb_copy_to_pma_ptr((const uint8_t *)buf, static_cast<uint16_t>(bytes), (uint32_t*)m_pEndpoints[MIDI_ENDPOINT_OFFSET_TX].pma);
	}
	// We still need to wait for the interrupt, even if we're sending
	// zero bytes. (Sending zero-size packets is useful for flushing
	// host-side buffers.)
	n_unsent_packets = packets;
	transmitting = true;
	usb_generic_set_tx(&m_pEndpoints[MIDI_ENDPOINT_OFFSET_TX], bytes);

	return packets;
}

uint32_t CMIDIDevices::CMIDIDevice::usb_midi_data_available(void) const
{
	return n_unread_packets;
}

bool CMIDIDevices::CMIDIDevice::usb_midi_is_transmitting(void) const
{
	return transmitting;
}

uint16_t CMIDIDevices::CMIDIDevice::usb_midi_get_pending(void) const
{
	return static_cast<uint16_t>(n_unsent_packets);
}

/* Nonblocking byte receive.
 *
 * Copies up to len bytes from our private data buffer (*NOT* the PMA)
 * into buf and deq's the FIFO. */
uint32_t CMIDIDevices::CMIDIDevice::usb_midi_rx(uint32_t* buf, uint32_t packets)
{
	/* Copy bytes to buffer. */
	uint32_t n_copied = usb_midi_peek(buf, packets);

	/* Mark bytes as read. */
	n_unread_packets -= n_copied;
	rx_offset += n_copied;

	/* If all bytes have been read, re-enable the RX endpoint, which
	 * was set to NAK when the current batch of bytes was received. */
	if (n_unread_packets == 0) {
		usb_generic_enable_rx(&m_pEndpoints[MIDI_ENDPOINT_OFFSET_RX]);
		rx_offset = 0;
	}

	return n_copied;
}

/* Nonblocking byte lookahead.
 *
 * Looks at unread bytes without marking them as read. */
uint32_t CMIDIDevices::CMIDIDevice::usb_midi_peek(uint32_t* buf, uint32_t packets) const
{
	uint32_t i;
	if (packets > n_unread_packets) {
		packets = n_unread_packets;
	}

	for (i = 0; i < packets; i++) {
		buf[i] = midiBufferRx[i + rx_offset];
	}

	return packets;
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

#if 0
// eventually all of this should be in a place for settings which can be written to flash.
volatile uint8 myMidiChannel = DEFAULT_MIDI_CHANNEL;
volatile uint8 myMidiDevice = DEFAULT_MIDI_DEVICE;
volatile uint8 myMidiCable = DEFAULT_MIDI_CABLE;
volatile uint8 myMidiID[] = { LEAFLABS_MMA_VENDOR_1,LEAFLABS_MMA_VENDOR_2,LEAFLABS_MMA_VENDOR_3,0};


// .............THIS IS NOT WORKING YET................
// send debugging information to
static uint8_t sysexbuffer[80]={CIN_SYSEX,0xF0,0x7D,0x33,CIN_SYSEX,0x33,0x00,0xf7}; // !!!bad hardcoded number foo !!!
uint8_t iSysHexLine(uint8_t rectype, uint16_t address, uint8_t *payload,uint8_t payloadlength, uint8_t *buffer);
void sendThroughSysex(char *printbuffer, int bufferlength) {
	int n;
	n = iSysHexLine(1, 0 , (uint8_t *) printbuffer, (uint8_t) bufferlength , sysexbuffer+6);
	usb_midi_tx((uint32*)sysexbuffer,n/4);
}

#define HIGHBYTE(x) ((uint8_t) (((x) >> 8) &  0x00ff) )
#define LOWBYTE(x) ((uint8_t) ((x) & 0x00ff) )
#define HIGHNIBBLE(x) ((((uint8_t)(x)) & 0xF0) >> 4)
#define LOWNIBBLE(x) (((uint8_t)(x)) & 0x0F)
#define HEXCHAR(c) ((c>9)?55+c:48+c)


uint8_t iSysHexLine(uint8_t rectype, uint16_t address, uint8_t *payload,uint8_t payloadlength, uint8_t *buffer) {

	int i=0; int j; int thirdone;
	uint8_t n=0;
	uint16_t checksum=0;
	//uint16_t length=0;

	buffer[i++]=':';

	checksum+=payloadlength;
	buffer[i++]=HEXCHAR(HIGHNIBBLE(payloadlength));
	buffer[i++]=HEXCHAR(LOWNIBBLE(payloadlength));
	buffer[i++]=CIN_SYSEX;

	n=HIGHBYTE(address);
	checksum+=n;
	buffer[i++]=HEXCHAR(HIGHNIBBLE(n));
	buffer[i++]=HEXCHAR(LOWNIBBLE(n));

	n=LOWBYTE(address);
	checksum+=n;
	buffer[i++]=HEXCHAR(HIGHNIBBLE(n));
	buffer[i++]=CIN_SYSEX;
	buffer[i++]=HEXCHAR(LOWNIBBLE(n));

	n=rectype;
	checksum+=n;
	buffer[i++]=HEXCHAR(HIGHNIBBLE(n));
	buffer[i++]=HEXCHAR(LOWNIBBLE(n));
	buffer[i++]=CIN_SYSEX;
	thirdone=0;
	for (j=0; j<payloadlength ; j++) {
		n=payload[j];
		checksum+=n;
		buffer[i++]=HEXCHAR(HIGHNIBBLE(n));
		if (++thirdone==3) {
			buffer[i++]=CIN_SYSEX;
			thirdone=0;
		}
		buffer[i++]=HEXCHAR(LOWNIBBLE(n));
		if (++thirdone==3) {
			buffer[i++]=CIN_SYSEX;
			thirdone=0;
		}
	}
	if (thirdone==0) {
		buffer[i-1]=CIN_SYSEX_ENDS_IN_3;
	}
	n=~((uint8_t) checksum&0x00ff)+1;
	buffer[i++]=HEXCHAR(HIGHNIBBLE(n));
	if (thirdone==1) {
		buffer[i++]=CIN_SYSEX_ENDS_IN_2;
	}
	buffer[i++]=HEXCHAR(LOWNIBBLE(n));
	if (thirdone==2) {
		buffer[i++]=CIN_SYSEX_ENDS_IN_1;
	}
	buffer[i++]=0xf7;
	return i+thirdone;
}
#endif
