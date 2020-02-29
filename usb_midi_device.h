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
#define MS_HEADER                         0x01
#define MIDI_IN_JACK                      0x02
#define MIDI_OUT_JACK                     0x03
#define MIDI_ELEMENT                      0x04

#define MIDI_JACK_EMBEDDED                0x01
#define MIDI_JACK_EXTERNAL                0x02

/* Element Capabilities Bitmask flags */
#define EL_CAPS_B0_CUSTOM                 0x01
#define EL_CAPS_B0_MIDI_CLOCK             0x02
#define EL_CAPS_B0_MTC                    0x04
#define EL_CAPS_B0_MMC                    0x08
#define EL_CAPS_B0_GM1                    0x10
#define EL_CAPS_B0_GM2                    0x20
#define EL_CAPS_B0_GS                     0x40
#define EL_CAPS_B0_XG                     0x80
#define EL_CAPS_B1_EFX                    0x01
#define EL_CAPS_B1_MIDI_Patch_Bay         0x02
#define EL_CAPS_B1_DLS1                   0x04
#define EL_CAPS_B1_DLS2                   0x08

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
	  struct {                                      \
		uint8  baSourceId;                          \
		uint8  baSourcePin;                         \
	  } __packed baSource[DataSize];                \
	  uint8  iJack;                                 \
  } __packed

#define MIDI_ELEMENT_DESCRIPTOR_SIZE(baSourceSize, bmElCapsSize) (10 + (2*baSourceSize) + (bmElCapsSize))
#define MIDI_ELEMENT_DESCRIPTOR(baSourceSize, bmElCapsSize)     \
  struct {                                                      \
	  uint8  bLength;                                           \
	  uint8  bDescriptorType;                                   \
	  uint8  SubType;                                           \
	  uint8  bElementID;                                        \
	  uint8  bNrInputPins;                                      \
	  struct {                                                  \
		uint8  baSourceId;                                      \
		uint8  baSourcePin;                                     \
	  } __packed baSource[baSourceSize];                        \
	  uint8  bNrOutputPins;                                     \
	  uint8  bInTerminalLink;                                   \
	  uint8  bOutTerminalLink;                                  \
	  uint8  bElCapsSize;                                       \
	  uint8  bmElementCaps[bmElCapsSize];                       \
	  uint8  iElement;                                          \
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

class CMIDIDevices
{
public:
	CMIDIDevices();
	CMIDIDevices(const CMIDIDevices &) = delete;				// No Copy Constructor
	CMIDIDevices & operator=(const CMIDIDevices &) = delete;	// No Assignment
	CMIDIDevices(CMIDIDevices &&) = delete;						// No Move Constructor
	CMIDIDevices & operator=(CMIDIDevices &&) = delete;			// No Move Assigment

	static USBCompositePart *getUSBMIDIPart() { return &usbMIDIPart; }
	static void setMIDICapabilities(uint8_t nElCapsB0, uint8_t nElCapsB1);

	static void usb_midi_setTXEPSize(uint32_t size);
	static uint32_t usb_midi_txEPSize() { return m_txEPSize; }
	static void usb_midi_setRXEPSize(uint32_t size);
	static uint32_t usb_midi_rxEPSize() { return m_rxEPSize; }

	static uint32_t usb_midi_tx(const uint32_t* buf, uint32_t len);
	static uint32_t usb_midi_rx(uint32_t* buf, uint32_t len);
	static uint32_t usb_midi_peek(uint32_t* buf, uint32_t len);

	static uint32_t usb_midi_data_available(void);		/* in RX buffer */
	static uint16_t usb_midi_get_pending(void);
	static bool usb_midi_is_transmitting(void);

	static void usbMIDIReset(void);

protected:
	// Callback Functions
	static void midiDataTxCb(void);
	static void midiDataRxCb(void);

protected:
	static USBCompositePart usbMIDIPart;

private:
	/* I/O state */

	/* Received data */
	static volatile uint32_t midiBufferRx[64/sizeof(uint32_t)];
	/* Read index into midiBufferRx */
	static volatile uint32_t rx_offset;
	/* Transmit data */
	static volatile uint32_t midiBufferTx[64/sizeof(uint32_t)];
	/* Write index into midiBufferTx */
	static volatile uint32_t tx_offset;
	/* Number of bytes left to transmit */
	static volatile uint32_t n_unsent_packets;
	/* Are we currently sending an IN packet? */
	static volatile bool transmitting;
	/* Number of unread bytes */
	static volatile uint32_t n_unread_packets;

	static uint32_t m_txEPSize;
	static uint32_t m_rxEPSize;

	static uint8_t m_nElCapsB0;
	static uint8_t m_nElCapsB1;

	static USBEndpointInfo m_midiEndpoints[2];

	static void getMIDIPartDescriptor(uint8* out);

private:
	#ifndef NUM_MIDI_PORTS
	#define NUM_MIDI_PORTS 1
	#endif

public:
	static constexpr int MIDI_PORT_COUNT = NUM_MIDI_PORTS;
};

#endif

#endif	// _LIBMAPLE_USB_MIDI_H_
