#ifndef __UVC__
#define __UVC__

#define EP_TYPE_CONTROL			0x00
#define EP_TYPE_BULK_IN			0x81
#define EP_TYPE_BULK_OUT		0x80
#define EP_TYPE_INTERRUPT_IN	0xC1
#define EP_TYPE_INTERRUPT_OUT	0xC0
#define EP_TYPE_ISOCHRONOUS_IN	0x41
#define EP_TYPE_ISOCHRONOUS_OUT	0x40

#define EP_SINGLE_BUFFER		0x02
#define EP_DOUBLE_BUFFER		0x06

#define EP_SIZE(s)	((s) == 64 ? 0x30 :	\
			((s) == 32 ? 0x20 :	\
			((s) == 16 ? 0x10 :	\
			             0x00)))

#define MAX_ENDPOINT		4

#define LSB(n) (n & 255)
#define MSB(n) ((n >> 8) & 255)

#if defined(__AVR_AT90USB162__)
#define HW_CONFIG() 
#define PLL_CONFIG() (PLLCSR = ((1<<PLLE)|(1<<PLLP0)))
#define USB_CONFIG() (USBCON = (1<<USBE))
#define USB_FREEZE() (USBCON = ((1<<USBE)|(1<<FRZCLK)))
#elif defined(__AVR_ATmega32U4__)
#define HW_CONFIG() (UHWCON = 0x01)
#define PLL_CONFIG() (PLLCSR = 0x12)
#define USB_CONFIG() (USBCON = ((1<<USBE)|(1<<OTGPADE)))
#define USB_FREEZE() (USBCON = ((1<<USBE)|(1<<FRZCLK)))
#elif defined(__AVR_AT90USB646__)
#define HW_CONFIG() (UHWCON = 0x81)
#define PLL_CONFIG() (PLLCSR = 0x1A)
#define USB_CONFIG() (USBCON = ((1<<USBE)|(1<<OTGPADE)))
#define USB_FREEZE() (USBCON = ((1<<USBE)|(1<<FRZCLK)))
#elif defined(__AVR_AT90USB1286__)
#define HW_CONFIG() (UHWCON = 0x81)
#define PLL_CONFIG() (PLLCSR = 0x16)
#define USB_CONFIG() (USBCON = ((1<<USBE)|(1<<OTGPADE)))
#define USB_FREEZE() (USBCON = ((1<<USBE)|(1<<FRZCLK)))
#endif

// standard control endpoint request types
#define GET_STATUS			0
#define CLEAR_FEATURE	    1
#define SET_FEATURE			3
#define SET_ADDRESS			5
#define GET_DESCRIPTOR	  	6
#define GET_CONFIGURATION	8
#define SET_CONFIGURATION	9
#define GET_INTERFACE		10
#define SET_INTERFACE		11

// UVC Definitions
// Video Class-Specific VC Interface Descriptor Subtypes
#define VC_DESCRIPTOR_UNDEFINED 0x00
#define VC_HEADER 0x01
#define VC_INPUT_TERMINAL 0x02
#define VC_OUTPUT_TERMINAL 0x03
#define VC_SELECTOR_UNIT 0x04
#define VC_PROCESSING_UNIT 0x05
#define VC_EXTENSION_UNIT 0x06

// Video Class-Specific VS Interface Descriptor Subtypes
#define VS_UNDEFINED 0x00
#define VS_INPUT_HEADER 0x01
#define VS_OUTPUT_HEADER 0x02
#define VS_STILL_IMAGE_FRAME 0x03
#define VS_FORMAT_UNCOMPRESSED 0x04
#define VS_FRAME_UNCOMPRESSED 0x05
#define VS_FORMAT_MJPEG 0x06
#define VS_FRAME_MJPEG 0x07
#define VS_FORMAT_MPEG2TS 0x0A
#define VS_FORMAT_DV 0x0C
#define VS_COLORFORMAT 0x0D
#define VS_FORMAT_FRAME_BASED 0x10
#define VS_FRAME_FRAME_BASED 0x11
#define VS_FORMAT_STREAM_BASED 0x12

// USB Terminal Types
#define TT_VENDOR_SPECIFIC 0x0100
#define TT_STREAMING 0x0101

// Input Terminal Types
#define ITT_VENDOR_SPECIFIC 0x0200
#define ITT_CAMERA 0x0201
#define ITT_MEDIA_TRANSPORT_INPUT 0x0202

// Output Terminal Types
#define OTT_VENDOR_SPECIFIC 0x0300
#define OTT_DISPLAY 0x0301
#define OTT_MEDIA_TRANSPORT_OUTPUT 0x0302

// External Terminal Types
#define EXTERNAL_VENDOR_SPECIFIC 0x0400
#define COMPOSITE_CONNECTOR 0x0401
#define SVIDEO_CONNECTOR 0x0402
#define COMPONENT_CONNECTOR 0x0403

#endif // #define __UVC__
