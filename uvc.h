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

#define EP_SIZE_8  	(0<<4)  // 001b: 8  bytes
#define EP_SIZE_16 	(1<<4)  // 001b: 16 bytes
#define EP_SIZE_32 	(2<<4)  // 010b: 32 bytes
#define EP_SIZE_64 	(3<<4)  // 011b: 64 bytes
#define EP_SIZE_128 (4<<4)  // 100b: 128 bytes
#define EP_SIZE_256 (5<<4)  // 101b: 256 bytes
#define EP_SIZE_512 (6<<4)  // 110b: 512 bytes
#define EP_SIZE(s)	((s) == 64 ? EP_SIZE_64 :	\
			((s) == 32 ? EP_SIZE_32 :	\
			((s) == 16 ? EP_SIZE_16 :	\
			             0x00)))


#define MAX_ENDPOINT		4

#define LSB(n) (n & 0xff)
#define MSB(n) ((n >> 8) & 0xff)
#define LSW(n) (n & 0xffff)
#define MSW(n) ((n >> 16) & 0xffff)
#define MKWORD(m,l) ((m << 8) | l)

#define W_TO_B(w) LSB(w), MSB(w)
#define DW_TO_B(dw) LSB(LSW(dw)),MSB(LSW(dw)),LSB(MSW(dw)), MSB(MSW(dw))

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

// Standard control endpoint request types
#define USB_REQ_GET_STATUS			0
#define USB_REQ_CLEAR_FEATURE	    1
#define USB_REQ_SET_FEATURE			3
#define USB_REQ_SET_ADDRESS			5
#define USB_REQ_GET_DESCRIPTOR	  	6
#define USB_REQ_GET_CONFIGURATION	8
#define USB_REQ_SET_CONFIGURATION	9
#define USB_REQ_GET_INTERFACE		10
#define USB_REQ_SET_INTERFACE		11

// USB descriptor types
#define USB_DT_DEVICE  0x1
#define USB_DT_CONFIG  0x2
#define USB_DT_STRING  0x3
#define USB_DT_INTERFACE  0x4
#define USB_DT_ENDPOINT  0x5
#define USB_DT_DEVICE_QUALIFIER  0x6
#define USB_DT_OTHER_SPEED_CONFIGURATION  0x7
#define USB_DT_INTERFACE_POWER  0x8
#define USB_DT_OTG  0x9
#define USB_DT_IAD  0x0B
#define USB_DT_CS_INTERFACE  0x24
#define USB_DT_CS_ENDPOINT  0x25

// UVC Definitions
// Video Class-Specific VC Interface Descriptor Subtypes
#define UVC_VC_DESCRIPTOR_UNDEFINED 0x00
#define UVC_VC_HEADER 0x01
#define UVC_VC_INPUT_TERMINAL 0x02
#define UVC_VC_OUTPUT_TERMINAL 0x03
#define UVC_VC_SELECTOR_UNIT 0x04
#define UVC_VC_PROCESSING_UNIT 0x05
#define UVC_VC_EXTENSION_UNIT 0x06

// Video Class-Specific VS Interface Descriptor Subtypes
#define UVC_VS_UNDEFINED 0x00
#define UVC_VS_INPUT_HEADER 0x01
#define UVC_VS_OUTPUT_HEADER 0x02
#define UVC_VS_STILL_IMAGE_FRAME 0x03
#define UVC_VS_FORMAT_UNCOMPRESSED 0x04
#define UVC_VS_FRAME_UNCOMPRESSED 0x05
#define UVC_VS_FORMAT_MJPEG 0x06
#define UVC_VS_FRAME_MJPEG 0x07
#define UVC_VS_FORMAT_MPEG2TS 0x0A
#define UVC_VS_FORMAT_DV 0x0C
#define UVC_VS_COLORFORMAT 0x0D
#define UVC_VS_FORMAT_FRAME_BASED 0x10
#define UVC_VS_FRAME_FRAME_BASED 0x11
#define UVC_VS_FORMAT_STREAM_BASED 0x12

// USB Terminal Types
#define UVC_TT_VENDOR_SPECIFIC 0x0100
#define UVC_TT_STREAMING 0x0101

// Input Terminal Types
#define UVC_ITT_VENDOR_SPECIFIC 0x0200
#define UVC_ITT_CAMERA 0x0201
#define UVC_ITT_MEDIA_TRANSPORT_INPUT 0x0202

// Output Terminal Types
#define UVC_OTT_VENDOR_SPECIFIC 0x0300
#define UVC_OTT_DISPLAY 0x0301
#define UVC_OTT_MEDIA_TRANSPORT_OUTPUT 0x0302

// External Terminal Types
#define UVC_EXTERNAL_VENDOR_SPECIFIC 0x0400
#define UVC_COMPOSITE_CONNECTOR 0x0401
#define UVC_SVIDEO_CONNECTOR 0x0402
#define UVC_COMPONENT_CONNECTOR 0x0403

// General AVR
#define BAUD_RATE 38400
#define LED_CONFIG      (DDRD |= (1<<6))
#define CPU_PRESCALE(n) (CLKPR = 0x80, CLKPR = (n))

#define LED_CONFIG      (DDRD |= (1<<6))
#define LED_ON          (PORTD &= ~(1<<6))
#define LED_OFF         (PORTD |= (1<<6))

#endif // #define __UVC__
