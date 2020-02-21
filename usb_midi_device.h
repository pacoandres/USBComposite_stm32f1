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
 * IMPORTANT: this API is unstable, and may change without notice.
 */

#ifndef _LIBMAPLE_USB_MIDI_H_
#define _LIBMAPLE_USB_MIDI_H_

#include <libmaple/libmaple_types.h>
#include <libmaple/gpio.h>
#include <libmaple/usb.h>
#include "MidiSpecs.h"
#include "usb_generic.h"

#define DEFAULT_MIDI_CABLE      0x00

#ifdef __cplusplus
extern "C" {
#endif

extern USBCompositePart usbMIDIPart;

typedef union {
	uint8  byte[4];
	uint32 data;
} USB_MIDI_Event_Packet;

/*
 * USB MIDI Requests
 */

/*
 * Descriptors, etc.
 */
#define USB_DESCRIPTOR_TYPE_CS_INTERFACE     0x24
#define USB_DESCRIPTOR_TYPE_CS_ENDPOINT      0x25


#define USB_DEVICE_CLASS_UNDEFINED        0x00
#define USB_DEVICE_CLASS_CDC              0x02
#define USB_DEVICE_SUBCLASS_UNDEFINED     0x00

#define USB_INTERFACE_CLASS_AUDIO         0x01
#define USB_INTERFACE_SUBCLASS_UNDEFINED  0x00
#define USB_INTERFACE_AUDIOCONTROL        0x01
#define USB_INTERFACE_AUDIOSTREAMING      0x02
#define USB_INTERFACE_MIDISTREAMING       0x03

/* MIDI Streaming class specific interfaces */
#define MIDI_IN_JACK                      0x02
#define MIDI_OUT_JACK                     0x03

#define MIDI_JACK_EMBEDDED                0x01
#define MIDI_JACK_EXTERNAL                0x02


#define AC_CS_INTERFACE_DESCRIPTOR_SIZE(DataSize) (8 + DataSize)
#define AC_CS_INTERFACE_DESCRIPTOR(DataSize)        \
 struct {                                           \
	  uint8  bLength;                               \
	  uint8  bDescriptorType;                       \
	  uint8  SubType;                               \
	  uint16 bcdADC;                                \
	  uint16 wTotalLength;                          \
	  uint8  bInCollection;                         \
	  uint8  baInterfaceNr[DataSize];               \
  } __packed

typedef struct {
	  uint8  bLength;
	  uint8  bDescriptorType;
	  uint8  SubType;
	  uint16 bcdADC;
	  uint16 wTotalLength;
  } __packed MS_CS_INTERFACE_DESCRIPTOR;

typedef struct {
	  uint8  bLength;
	  uint8  bDescriptorType;
	  uint8  SubType;
	  uint8  bJackType;
	  uint8  bJackId;
	  uint8  iJack;
  } __packed MIDI_IN_JACK_DESCRIPTOR;

#define MIDI_OUT_JACK_DESCRIPTOR_SIZE(DataSize) (7 + 2*DataSize)
#define MIDI_OUT_JACK_DESCRIPTOR(DataSize)        \
 struct {                                           \
	  uint8  bLength;                               \
	  uint8  bDescriptorType;                       \
	  uint8  SubType;                               \
	  uint8  bJackType;                             \
	  uint8  bJackId;                               \
	  uint8  bNrInputPins;                          \
	  uint8  baSourceId[DataSize];                  \
	  uint8  baSourcePin[DataSize];                 \
	  uint8  iJack;                                 \
  } __packed


#define MS_CS_BULK_ENDPOINT_DESCRIPTOR_SIZE(DataSize) (4 + DataSize)
#define MS_CS_BULK_ENDPOINT_DESCRIPTOR(DataSize)    \
 struct {                                           \
	  uint8  bLength;                               \
	  uint8  bDescriptorType;                       \
	  uint8  SubType;                               \
	  uint8  bNumEmbMIDIJack;                       \
	  uint8  baAssocJackID[DataSize];               \
  } __packed

/*
 * Endpoint configuration
 */

#ifndef __cplusplus
#define USB_MIDI_DECLARE_DEV_DESC(vid, pid)                           \
  {                                                                     \
	  .bLength            = sizeof(usb_descriptor_device),              \
	  .bDescriptorType    = USB_DESCRIPTOR_TYPE_DEVICE,                 \
	  .bcdUSB             = 0x0110,                                     \
	  .bDeviceClass       = USB_DEVICE_CLASS_UNDEFINED,                 \
	  .bDeviceSubClass    = USB_DEVICE_SUBCLASS_UNDEFINED,              \
	  .bDeviceProtocol    = 0x00,                                       \
	  .bMaxPacketSize0    = 0x40,                                       \
	  .idVendor           = vid,                                        \
	  .idProduct          = pid,                                        \
	  .bcdDevice          = 0x0200,                                     \
	  .iManufacturer      = 0x01,                                       \
	  .iProduct           = 0x02,                                       \
	  .iSerialNumber      = 0x00,                                       \
	  .bNumConfigurations = 0x01,                                       \
 }
#endif

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

/*
* MIDI interface
*/

#define MIDI_ENDPOINT_OFFSET_RX 0
#define MIDI_ENDPOINT_OFFSET_TX 1

extern class CMIDIDevices g_MIDIDevices;

class CMIDIDevices
{
public:
	class CMIDIDevice
	{
	public:
		void usb_midi_setTXEPSize(uint32_t size);
		uint32_t usb_midi_txEPSize() const { return m_txEPSize; }
		void usb_midi_setRXEPSize(uint32_t size);
		uint32_t usb_midi_rxEPSize() const { return m_rxEPSize; }
		uint32_t usb_midi_tx(const uint32_t* buf, uint32_t len);
		uint32_t usb_midi_rx(uint32_t* buf, uint32_t len);
		uint32_t usb_midi_peek(uint32_t* buf, uint32_t len) const;

		uint32_t usb_midi_data_available(void) const;		/* in RX buffer */
		uint16_t usb_midi_get_pending(void) const;
		bool usb_midi_is_transmitting(void) const;

	protected:
		friend class CMIDIDevices;

		// Callback Functions
		template<int nIndex>
		static void midiDataTxCb(void)
		{
			CMIDIDevice &dev = g_MIDIDevices.port(nIndex);

			dev.n_unsent_packets = 0;
			dev.transmitting = false;
		}

		template<int nIndex>
		static void midiDataRxCb(void)
		{
			CMIDIDevice &dev = g_MIDIDevices.port(nIndex);
			USBEndpointInfo *pEndpoint = &dev.m_pEndpoints[MIDI_ENDPOINT_OFFSET_RX];

			usb_generic_pause_rx(pEndpoint);
			dev.n_unread_packets = usb_get_ep_rx_count(pEndpoint->address) / sizeof(uint32_t);
			/* This copy won't overwrite unread bytes, since we've set the RX
			 * endpoint to NAK, and will only set it to VALID when all bytes
			 * have been read. */

			usb_copy_from_pma_ptr((uint8_t*)dev.midiBufferRx, dev.n_unread_packets * sizeof(uint32_t),
							  (uint32_t*)pEndpoint->pma);

			if (dev.n_unread_packets == 0) {
				usb_generic_enable_rx(pEndpoint);
				dev.rx_offset = 0;
			}
		}

		void usbMIDIReset(void)
		{
			/* Reset the RX/TX state */
			n_unread_packets = 0;
			n_unsent_packets = 0;
			rx_offset = 0;
		}

	private:
		/* I/O state */

		/* Received data */
		volatile uint32_t midiBufferRx[64/sizeof(uint32_t)];
		/* Read index into midiBufferRx */
		volatile uint32_t rx_offset = 0;
		/* Transmit data */
		volatile uint32_t midiBufferTx[64/sizeof(uint32_t)];
		/* Write index into midiBufferTx */
		volatile uint32_t tx_offset = 0;
		/* Number of bytes left to transmit */
		volatile uint32_t n_unsent_packets = 0;
		/* Are we currently sending an IN packet? */
		volatile bool transmitting = false;
		/* Number of unread bytes */
		volatile uint32_t n_unread_packets = 0;

		uint32_t m_txEPSize = 64;
		uint32_t m_rxEPSize = 64;

		USBEndpointInfo *m_pEndpoints = nullptr;
	};

	CMIDIDevices();
	CMIDIDevices(const CMIDIDevices &) = delete;				// No Copy Constructor
	CMIDIDevices & operator=(const CMIDIDevices &) = delete;	// No Assignment
	CMIDIDevices(CMIDIDevices &&) = delete;						// No Move Constructor
	CMIDIDevices & operator=(CMIDIDevices &&) = delete;			// No Move Assigment

	static void usbMIDIReset(void);
	static void getMIDIPartDescriptor(uint8* out);

	CMIDIDevice &port(int nIndex) const
	{
		return m_ports[nIndex];
	}

private:
#ifndef NUM_MIDI_PORTS
#define NUM_MIDI_PORTS 1
#endif

	static CMIDIDevice m_ports[NUM_MIDI_PORTS];

	template<int nIndex>
	void init_callbacks()
	{
		init_callbacks<nIndex-1>();
		marr_midiDataTxCb[nIndex] = &CMIDIDevice::template midiDataTxCb<nIndex>;
		marr_midiDataRxCb[nIndex] = &CMIDIDevice::template midiDataRxCb<nIndex>;
	}

	void (*marr_midiDataTxCb[NUM_MIDI_PORTS])(void);
	void (*marr_midiDataRxCb[NUM_MIDI_PORTS])(void);

public:
	static constexpr int MIDI_PORT_COUNT = NUM_MIDI_PORTS;
};

#endif


#endif	// _LIBMAPLE_USB_MIDI_H_
