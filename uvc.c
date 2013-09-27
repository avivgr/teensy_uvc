#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h> 
#include "uart.h"
#include "uvc.h"

#define DBG(x) uart_print(x)
char _buff[32];
#define DBGV(format, ...) \
    snprintf_P(_buff, sizeof(_buff)-1, PSTR(format), __VA_ARGS__);\
    uart_print_S(_buff);



#define STR_MANUFACTURER    L"Acme"
#define STR_MANUFACTURER_I  1
#define STR_PRODUCT     L"Teensy UVC Camera"
#define STR_PRODUCT_I 2

#define VENDOR_ID       0x045e //Microsoft Corp.
#define PRODUCT_ID      0x375d //LifeCam Cinema

// Endpoint config
#define ENDPOINT0_SIZE  16
#define UVC_INTERFACE   0
#define UVC_TX_ENDPOINT 1
#define UVC_TX_BUFFER   EP_DOUBLE_BUFFER
#define UVC_TX_SIZE 256

static const uint8_t PROGMEM endpoint_config_table[] = {
    1, EP_TYPE_ISOCHRONOUS_IN, EP_SIZE(UVC_TX_SIZE) | UVC_TX_BUFFER,
    0,
    0,
    0
};

static uint8_t PROGMEM device_descriptor[] = {
    18,                 // bLength
    USB_DT_DEVICE,      // bDescriptorType
    W_TO_B(0x0200),     // bcdUSB
    0xef,               // bDeviceClass = Miscellaneous Device Class
    0x02,               // bDeviceSubClass = Common Class
    0x01,               // bDeviceProtocol = Interface Association Descriptor
    ENDPOINT0_SIZE,     // bMaxPacketSize0
    W_TO_B(VENDOR_ID),  // idVendor
    W_TO_B(PRODUCT_ID), // idProduct
    W_TO_B(0x0100),     // bcdDevice
    STR_MANUFACTURER_I, // iManufacturer
    STR_PRODUCT_I,      // iProduct
    0,                  // iSerialNumber
    1                   // bNumConfigurations
};

#define CONFIG1_DESC_SIZE (9+8+9+13+17+9+11+9+14+27+46+9+7)
#define VC_DESC_SIZE (13+17+9+11)
#define VS_DESC_SIZE (14+27+46) 
static uint8_t PROGMEM config1_descriptor[] = {
    // configuration descriptor, USB spec 9.6.3, page 264-266, Table 9-10
    9,                  // bLength;
    USB_DT_CONFIG,      // bDescriptorType;
    W_TO_B(CONFIG1_DESC_SIZE),  // wTotalLength
    2,                  // bNumInterfaces
    1,                  // bConfigurationValue
    0,                  // iConfiguration
    0x80,               // bmAttributes  = Bus-powered device, no remote wakeup capability
    0xFA,               // bMaxPower

    // Interface Association Descriptor
    8,                  // bLength
    USB_DT_IAD,         // bDescriptorType
    0x00,               // bFirstInterface = Interface number of the VideoControl interface for this function 
    0x02,               // bInterfaceCount = Number of contiguous Video interfaces that are for this function 
    0x0E,               // bFunctionClass = CC_VIDEO 
    0x03,               // bFunctionSubClass = SC_VIDEO_INTERFACE_COLLECTION 
    0x00,               // bFunctionProtocol = Not used. Must be set to PC_PROTOCOL_UNDEFINED. 
    STR_PRODUCT_I,      // iFunction = Index to product string descriptor

    // Standard VideoControl Interface Descriptor
    9,                  // bLength = Size of this descriptor, in bytes.
    USB_DT_INTERFACE,   // bDescriptorType = INTERFACE descriptor type
    0x00,               // bInterfaceNumber = Index of this interface
    0x00,               // bAlternateSetting = Index of this setting
    0x00,               // bNumEndpoints = 0 endpoints (NO interrupt endpoint)
    0x0E,               // bInterfaceClass = CC_VIDEO
    0x01,               // bInterfaceSubClass = SC_VIDEOCONTROL
    0x00,               // bInterfaceProtocol = Not used. Must be set to PC_PROTOCOL_UNDEFINED.
    STR_PRODUCT_I,      // iInterface = Index to string descriptor that contains the product string 

    // Class-specific VideoControl Interface Descriptor
    13,                 // bLength = Size of this descriptor, in bytes.
    USB_DT_CS_INTERFACE,// bDescriptorType = USB_DT_CS_INTERFACE
    0x01,               // bDescriptorSubType = VC_HEADER subtype
    W_TO_B(0x0110),     // bcdUVC = 0x0110 version 1.1.
    W_TO_B(VC_DESC_SIZE),// wTotalLength = Total size of class-specific descriptors
    DW_TO_B(0x005B8D80),// dwClockFrequency = deprecated. 0x005B8D80 This device will provide timestamps and a device clock reference based on a 6MHz clock.
    0x01,               // bInCollection = Number of streaming interfaces.
    0x01,               // baInterfaceNr(1) = VideoStreaming interface 1 belongs to this VideoControl interface.

    // Input Terminal Descriptor (Camera)
    17,                 // bLength = Size of this descriptor, in bytes.
    USB_DT_CS_INTERFACE,        // bDescriptorType = USB_DT_CS_INTERFACE
    UVC_VC_INPUT_TERMINAL,// bDescriptorSubtype = VC_INPUT_TERMINAL subtype
    0x01,               // bTerminalID = ID of this input terminal
    W_TO_B(UVC_ITT_CAMERA),// wTerminalType = ITT_CAMERA type. This terminal is a camera terminal representing the CCD sensor.
    0x00,               // bAssocTerminal = No association
    0x00,               // iTerminal = Unused
    W_TO_B(0x0000),     // wObjectiveFocalLengthMin = No optical zoom supported
    W_TO_B(0x0000),     // wObjectiveFocalLengthMax = No optical zoom supported
    W_TO_B(0x0000),     // wOcularFocalLength = No optical zoom supported
    0x02,               // bControlSize = The size of the bmControls is 2 bytes (this terminal doesn’t implement any controls).
    W_TO_B(0x0000),     // bmControls = No controls are supported.

    // Output Terminal Descriptor
    9,                  // bLength = Size of this descriptor, in bytes.
    USB_DT_CS_INTERFACE, // bDescriptorType = USB_DT_CS_INTERFACE
    UVC_VC_OUTPUT_TERMINAL, // bDescriptorSubtype = VC_OUTPUT_TERMINAL
    0x02,               // bTerminalID = ID of this terminal
    W_TO_B(UVC_TT_STREAMING),// wTerminalType = TT_STREAMING type. This terminal is a USB streaming terminal.
    0x00,               // bAssocTerminal = No association
    0x03,               // bSourceID = The input pin of this unit is connected to the output pin of unit 3.
    0x00,               // iTerminal = Unused

    // Processing Unit Descriptor
    11,                 // bLength = Size of this descriptor, in bytes.
    USB_DT_CS_INTERFACE, // bDescriptorType = USB_DT_CS_INTERFACE
    UVC_VC_PROCESSING_UNIT,// bDescriptorSubtype = VC_PROCESSING_UNIT
    0x03,               // bUnitID = ID of this unit
    0x01,               // bSourceID = This input pin of this unit is connected to the output pin of unit with ID 1.
    0x00, 0x00,         // wMaxMultiplier = unused
    0x02,               // bControlSize = Size of the bmControls field, in bytes.
    0x01, 0x00,         // bmControls = Brightness control supported
    0x00,               // iProcessing = Unused

#if 0
    // Standard Interrupt Endpoint Descriptor
    0x07,               // bLength = Size of this descriptor, in bytes.
    USB_DT_ENDPOINT,    // bDescriptorType = ENDPOINT descriptor
    0x83,               // bEndpointAddress = IN endpoint 3
    0x03,               // bmAttributes = Interrupt transfer type
    0x08, 0x00,         // wMaxPacketSize = 8-byte status packet
    0x0A,               // bInterval = Poll at least every 10ms.

    // Class-specific Interrupt Endpoint Descriptor
    0x05,               // bLength = Size of this descriptor, in bytes.
    USB_DT_CS_ENDPOINT, // bDescriptorType = CS_ENDPOINT descriptor
    0x03,               // bDescriptorSubType = EP_INTERRUPT
    0x20, 0x00,         // wMaxTransferSize = 32-byte status packet
#endif

    // Standard VideoStreaming Interface Descriptor - - Operational Alternate Setting 0
    9,                  // bLength = Size of this descriptor, in bytes.
    USB_DT_INTERFACE,   // bDescriptorType = INTERFACE descriptor type
    0x01,               // bInterfaceNumber = Index of this interface
    0x00,               // bAlternateSetting = Index of this alternate setting
    0x00,               // bNumEndpoints = 0 endpoints – no bandwidth used
    0x0E,               // bInterfaceClass = CC_VIDEO
    0x02,               // bInterfaceSubClass = SC_VIDEOSTREAMING
    0x00,               // bInterfaceProtocol = PC_PROTOCOL_UNDEFINED
    0x00,               // iInterface = Unused

    // Class-specific VideoStreaming Header Descriptor (Input)
    14,                 // bLength = Size of this descriptor, in bytes.
    USB_DT_CS_INTERFACE,// bDescriptorType = USB_DT_CS_INTERFACE
    UVC_VS_INPUT_HEADER,// bDescriptorSubtype = VS_INPUT_HEADER.
    0x01,               // bNumFormats = One format descriptor follows.
    W_TO_B(VS_DESC_SIZE),// wTotalLength = Total size of class-specific VideoStreaming interface descriptors
    UVC_TX_ENDPOINT|0x80, // bEndpointAddress = Address of the isochronous endpoint used for video data
    0x00,               // bmInfo = No dynamic format change supported
    0x02,               // bTerminalLink = This VideoStreaming interface supplies terminal ID 2 (Output Terminal).
    0x01,               // bStillCaptureMethod = Device supports still image capture method 1.
    0x00,               // bTriggerSupport = Hardware trigger supported for still image capture
    0x00,               // bTriggerUsage = Hardware trigger should initiate a still image capture.
    0x01,               // bControlSize = Size of the bmaControls field
    0x00,               // bmaControls = No VideoStreaming specific controls are supported.

    // Class-specific VideoStreaming Format Descriptor
    27,                 // bLength = Size of this descriptor, in bytes.
    USB_DT_CS_INTERFACE,        // bDescriptorType = USB_DT_CS_INTERFACE
    UVC_VS_FORMAT_UNCOMPRESSED, // bDescriptorSubtype = UVC_VS_FORMAT_UNCOMPRESSED
    1,                  // bFormatIndex         
    1,              // bNumFrameDescriptors
    0x59, 0x55, 0x59, 0x32, // guidFormat
    0x00, 0x00, 0x10, 0x00,
    0x80, 0x00, 0x00, 0xaa, 
    0x00, 0x38, 0x9b, 0x71, 
    16,                 // bBitsPerPixel      
    1,                  // bDefaultFrameIndex 
    0,                  // bAspectRatioX      
    0,                  // bAspectRatioY      
    0x00,               // bmInterlaceFlags
    0,                  // bCopyProtect   

    // Class-specific VideoStreaming Frame Descriptor
    46,                 // bLength                
    USB_DT_CS_INTERFACE,// bDescriptorType        
    UVC_VS_FRAME_UNCOMPRESSED,// bDescriptorSubtype     
    1,                  // bFrameIndex            
    0x01,               // bmCapabilities
    W_TO_B(640),        // wWidth 
    W_TO_B(480),        // wHeight  
    DW_TO_B(36864000),  // dwMinBitRate              
    DW_TO_B(147456000), // dwMaxBitRate              
    DW_TO_B(614400),    // dwMaxVideoFrameBufferSize
    DW_TO_B(333333),    // dwDefaultFrameInterval   
    5,                  // bFrameIntervalType       
    DW_TO_B(333333),    // dwFrameInterval( 0)      
    DW_TO_B(500000),    // dwFrameInterval( 1)      
    DW_TO_B(666666),    // dwFrameInterval( 2)      
    DW_TO_B(1000000),   // dwFrameInterval( 3)       
    DW_TO_B(1333333),   // dwFrameInterval( 4)       

    // Standard VS Interface Descriptor - Operational Alternate Setting 1
    9,                  // bLength = Size of this descriptor, in bytes.
    USB_DT_INTERFACE,   // bDescriptorType = INTERFACE descriptor type
    0x01,               // bInterfaceNumber = Index of this interface
    0x01,               // bAlternateSetting = Index of this alternate setting
    0x01,               // bNumEndpoints = 0 endpoints – no bandwidth used
    0x0E,               // bInterfaceClass = CC_VIDEO
    0x02,               // bInterfaceSubClass = SC_VIDEOSTREAMING
    0x00,               // bInterfaceProtocol = PC_PROTOCOL_UNDEFINED
    0x00,               // iInterface = Unused

    // Standard VS Isochronous Video Data Endpoint Descriptor
    7,                  // bLength = Size of this descriptor, in bytes.
    USB_DT_ENDPOINT,    // bDescriptorType = ENDPOINT
    UVC_TX_ENDPOINT|0x80,// bEndpointAddress = IN endpoint 2
    0x05,               // bmAttributes = Isochronous transfer type. 
    W_TO_B(UVC_TX_SIZE),// wMaxPacketSize = Max packet size
    1,                  // bInterval = One frame interval BUGBUG what means??
};

static uint8_t PROGMEM device_qualifier_desc[] = {
    10,                  // bLength            
    USB_DT_DEVICE_QUALIFIER, // bDescriptorType    
    W_TO_B(0x0200),      // bcdUSB 
    239,                 // bDeviceClass 239 Miscellaneous Device
    2,                   // bDeviceSubClass     
    1,                   // bDeviceProtocol     Interface Association
    ENDPOINT0_SIZE,      // bMaxPacketSize0    
    1,                   // bNumConfigurations
    0                    // Device Status: 0x0000(Bus Powered)
};

static uint8_t PROGMEM other_speed_descriptor[] = {
    9,                  // bLength;
    USB_DT_OTHER_SPEED_CONFIGURATION,  // bDescriptorType;
    W_TO_B(9),          // wTotalLength
    1,                  // bNumInterfaces
    1,                  // bConfigurationValue
    0,                  // iConfiguration
    0xA0,               // bmAttributes
    50,                 // bMaxPower
};

struct usb_string_descriptor_struct {
    uint8_t bLength;
    uint8_t bDescriptorType;
    int16_t wString[];
};

static struct usb_string_descriptor_struct PROGMEM string0 = {
    4,
    USB_DT_STRING,
    {0x0409}
};

static struct usb_string_descriptor_struct PROGMEM string1 = {
    sizeof(STR_MANUFACTURER),
    USB_DT_STRING,
    STR_MANUFACTURER
};

static struct usb_string_descriptor_struct PROGMEM string2 = {
    sizeof(STR_PRODUCT),
    USB_DT_STRING,
    STR_PRODUCT
};

// This table defines which descriptor data is sent for each specific
// request from the host (in wValue and wIndex).
static struct descriptor_list_struct {
    uint16_t    wValue;
    uint16_t    wIndex;
    const uint8_t   *addr;
    uint8_t     length;
} PROGMEM descriptor_list[] = {
    {MKWORD(USB_DT_DEVICE, 0x00), 0x0000, device_descriptor, sizeof(device_descriptor)},
    {MKWORD(USB_DT_CONFIG, 0x00), 0x0000, config1_descriptor, sizeof(config1_descriptor)},
    {MKWORD(USB_DT_DEVICE_QUALIFIER, 0x00), 0x0000, device_qualifier_desc, sizeof(device_qualifier_desc)},
    {MKWORD(USB_DT_OTHER_SPEED_CONFIGURATION, 0x00), 0x0000, other_speed_descriptor, sizeof(other_speed_descriptor)},
    {MKWORD(USB_DT_STRING, 0x00), 0x0000, (const uint8_t *)&string0, 4},
    {MKWORD(USB_DT_STRING, STR_MANUFACTURER_I), 0x0409, (const uint8_t *)&string1, sizeof(STR_MANUFACTURER)},
    {MKWORD(USB_DT_STRING, STR_PRODUCT_I), 0x0409, (const uint8_t *)&string2, sizeof(STR_PRODUCT)}
};
#define NUM_DESC_LIST (sizeof(descriptor_list)/sizeof(struct descriptor_list_struct))

// zero when we are not configured, non-zero when enumerated
static volatile uint8_t usb_configuration=0;
static volatile uint8_t transmit_flush_timer=0;

// initialize USB
void usb_init(void)
{
    HW_CONFIG();
    USB_FREEZE();               // enable USB
    PLL_CONFIG();               // config PLL
    while (!(PLLCSR & (1<<PLOCK))) 
        ;   // wait for PLL lock
    USB_CONFIG();               // start USB clock
    UDCON = 0;              // enable attach resistor
    usb_configuration = 0;
    UDIEN = (1<<EORSTE)|(1<<SOFE);
    sei();
}

uint8_t usb_configured(void)
{
    return usb_configuration;
}

// Misc functions to wait for ready and send/receive packets
static inline void usb_wait_in_ready(void)
{
    while (!(UEINTX & (1<<TXINI))) ;
}
static inline void usb_send_in(void)
{
    UEINTX = ~(1<<TXINI);
}
static inline void usb_wait_receive_out(void)
{
    while (!(UEINTX & (1<<RXOUTI))) ;
}
static inline void usb_ack_out(void)
{
    UEINTX = ~(1<<RXOUTI);
}
static inline void usb_stall(void)
{
    UECONX = (1<<STALLRQ)|(1<<EPEN);
}

int main(void)
{
    CPU_PRESCALE(0);  // run at 16 MHz
    LED_CONFIG;

    uart_init(BAUD_RATE);
    
    // Initialize the USB, and then wait for the host to set configuration.
    usb_init();
    while (!usb_configured()) 
        /* wait */ ;

    // Wait an extra second for the PC's operating system to load drivers
    // and do whatever it does to actually be ready for input
    _delay_ms(1000);

    while (1) {
        
    }
}

// USB Device Interrupt - handle all device events
ISR(USB_GEN_vect)
{
    uint8_t intbits, t;
    
    intbits = UDINT;
    UDINT = 0;
    if (intbits & (1<<EORSTI)) {
        UENUM = 0;
        UECONX = 1;
        UECFG0X = EP_TYPE_CONTROL;
        UECFG1X = EP_SIZE(ENDPOINT0_SIZE) | EP_SINGLE_BUFFER;
        UEIENX = (1<<RXSTPE);
        usb_configuration = 0;
        LED_OFF;
    }
    if ((intbits & (1<<SOFI))) {
        if (usb_configuration) {
            t = transmit_flush_timer;
            if (t) {
                transmit_flush_timer = --t;
                if (!t) {
                    UENUM = UVC_TX_ENDPOINT;
                    UEINTX = 0x3A;
                }
            }
        }       
    }
}

// USB Endpoint Interrupt - endpoint 0 is handled here.  The
// other endpoints are manipulated by the user-callable
// functions, and the start-of-frame interrupt.
//
// USB Endpoint Interrupt - endpoint 0 is handled here.  The
// other endpoints are manipulated by the user-callable
// functions, and the start-of-frame interrupt.
//
ISR(USB_COM_vect)
{
    uint8_t intbits;
    const uint8_t *list;
    const uint8_t *cfg;
    uint8_t i, n, len, en;
    uint8_t bmRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
    uint16_t desc_val;
    const uint8_t *desc_addr;
    uint8_t desc_length;

    UENUM = 0;
    intbits = UEINTX;
    if (intbits & (1<<RXSTPI)) {
        bmRequestType = UEDATX;
        bRequest = UEDATX;
        wValue = UEDATX;
        wValue |= (UEDATX << 8);
        wIndex = UEDATX;
        wIndex |= (UEDATX << 8);
        wLength = UEDATX;
        wLength |= (UEDATX << 8);
        UEINTX = ~((1<<RXSTPI) | (1<<RXOUTI) | (1<<TXINI));
        if (bRequest == USB_REQ_GET_DESCRIPTOR) {
            list = (const uint8_t *)descriptor_list;
            for (i=0; ; i++) {
                if (i >= NUM_DESC_LIST) {
                    UECONX = (1<<STALLRQ)|(1<<EPEN);  //stall
                    DBG("Unk descriptor!\r\n");
                    DBGV("wV=%d wI=%d\r\n", wValue, wIndex);
                    return;
                }
                desc_val = pgm_read_word(list);
                if (desc_val != wValue) {
                    list += sizeof(struct descriptor_list_struct);
                    continue;
                }
                list += 2;
                desc_val = pgm_read_word(list);
                if (desc_val != wIndex) {
                    list += sizeof(struct descriptor_list_struct)-2;
                    continue;
                }
                list += 2;
                desc_addr = (const uint8_t *)pgm_read_word(list);
                list += 2;
                desc_length = pgm_read_byte(list);
                break;
            }
            len = (wLength < 256) ? wLength : 255;
            if (len > desc_length) len = desc_length;
            do {
                // wait for host ready for IN packet
                do {
                    i = UEINTX;
                } while (!(i & ((1<<TXINI)|(1<<RXOUTI))));
                if (i & (1<<RXOUTI)) return;    // abort
                // send IN packet
                n = len < ENDPOINT0_SIZE ? len : ENDPOINT0_SIZE;
                for (i = n; i; i--) {
                    UEDATX = pgm_read_byte(desc_addr++);
                }
                len -= n;
                usb_send_in();
            } while (len || n == ENDPOINT0_SIZE);
            return;
        }
        if (bRequest == USB_REQ_SET_ADDRESS) {
            usb_send_in();
            usb_wait_in_ready();
            UDADDR = wValue | (1<<ADDEN);
            return;
        }
        
        if (bRequest == USB_REQ_SET_CONFIGURATION && bmRequestType == 0) {
            usb_configuration = wValue;
            usb_send_in();
            cfg = endpoint_config_table;
            for (i=1; i<5; i++) {
                UENUM = i;
                en = pgm_read_byte(cfg++);
                UECONX = en;
                if (en) {
                    UECFG0X = pgm_read_byte(cfg++);
                    UECFG1X = pgm_read_byte(cfg++);
                }
            }
            UERST = 0x1E;
            UERST = 0;
            DBG("Configured!\r\n");
            return;
        }
        if (bRequest == USB_REQ_GET_CONFIGURATION && bmRequestType == 0x80) {
            usb_wait_in_ready();
            UEDATX = usb_configuration;
            usb_send_in();
            return;
        }
        if (bRequest == USB_REQ_GET_STATUS) {
            usb_wait_in_ready();
            i = 0;
#ifdef SUPPORT_ENDPOINT_HALT
            if (bmRequestType == 0x82) {
                UENUM = wIndex;
                if (UECONX & (1<<STALLRQ)) i = 1;
                UENUM = 0;
            }
#endif
            UEDATX = i;
            UEDATX = 0;
            usb_send_in();
            return;
        }
#ifdef SUPPORT_ENDPOINT_HALT
        if ((bRequest == USB_REQ_CLEAR_FEATURE || 
             bRequest == USB_REQ_SET_FEATURE) && 
             bmRequestType == 0x02 && wValue == 0) {
                i = wIndex & 0x7F;
                if (i >= 1 && i <= MAX_ENDPOINT) {
                    usb_send_in();
                    UENUM = i;
                    if (bRequest == SET_FEATURE) {
                        UECONX = (1<<STALLRQ)|(1<<EPEN);
                    } else {
                        UECONX = (1<<STALLRQC)|(1<<RSTDT)|(1<<EPEN);
                        UERST = (1 << i);
                        UERST = 0;
                    }
                    return;
                }
        }
#endif
        if (bRequest == USB_REQ_SET_INTERFACE && bmRequestType == 1) {
            usb_send_in();
            usb_wait_in_ready();
            return;
        }
    }
    DBGV("?Req %d\r\n", bRequest);
    UECONX = (1<<STALLRQ) | (1<<EPEN);  // stall
}
