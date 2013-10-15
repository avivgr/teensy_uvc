/*
    Author: Aviv Greenberg - www.linkedin.com/in/avivgr/
*/    
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h> 
#include "uart.h"
#include "uvc.h"

#define DBG(x) uart_print(x)
char _buff[64];
#define DBGV(format, ...) \
    snprintf_P(_buff, sizeof(_buff)-1, PSTR(format), __VA_ARGS__);\
    uart_print_S(_buff);

#define STR_MANUFACTURER L"Acme"
#define STR_MANUFACTURER_I 1
#define STR_PRODUCT L"Teensy UVC Camera"
#define STR_PRODUCT_I 2

#define VENDOR_ID       0x046e
#define PRODUCT_ID      0x375d

// Endpoint config
#define ENDPOINT0_SIZE  16
#define UVC_INTERFACE   0
#define UVC_TX_ENDPOINT 1
#define UVC_TX_BUFFER   EP_DOUBLE_BUFFER
#define UVC_TX_SIZE 256

#define HEIGHT 240L
#define WIDTH 320L
#define FRAME_SIZE (HEIGHT*WIDTH*2)

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

#define CONFIG1_DESC_SIZE (9+8+9+13+17+9+7+12+9+14+27+46+9+7)
#define VC_DESC_SIZE (13+17+9+7+12)
#define VS_DESC_SIZE (14+27+46)

/* Terminal IDs */
#define INPUT_TERMINAL_ID 0x01
#define OUTPUT_TERMINAL_ID 0x02
#define SU_TERMINAL_ID 0x03
#define PU_TERMINAL_ID 0x04

#define VIDEOC_IFACE 0x00
#define VIDEOS_IFACE 0x01

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
    CC_VIDEO,           // bFunctionClass = CC_VIDEO 
    UVC_SC_VIDEO_INTERFACE_COLLECTION,// bFunctionSubClass = SC_VIDEO_INTERFACE_COLLECTION 
    0x00,               // bFunctionProtocol = Not used. Must be set to PC_PROTOCOL_UNDEFINED. 
    STR_PRODUCT_I,      // iFunction = Index to product string descriptor

    // Standard VideoControl Interface Descriptor
    9,                  // bLength = Size of this descriptor, in bytes.
    USB_DT_INTERFACE,   // bDescriptorType = INTERFACE descriptor type
    VIDEOC_IFACE,       // bInterfaceNumber = Index of this interface
    0x00,               // bAlternateSetting = Index of this setting
    0x00,               // bNumEndpoints = 0 endpoints (NO interrupt endpoint)
    CC_VIDEO,           // bInterfaceClass = CC_VIDEO
    UVC_SC_VIDEOCONTROL,// bInterfaceSubClass = SC_VIDEOCONTROL
    0x00,               // bInterfaceProtocol = Not used. Must be set to PC_PROTOCOL_UNDEFINED.
    STR_PRODUCT_I,      // iInterface = Index to string descriptor that contains the product string 

    // Class-specific VideoControl Interface Descriptor
    13,                 // bLength = Size of this descriptor, in bytes.
    UVC_DT_CS_INTERFACE,// bDescriptorType = UVC_DT_CS_INTERFACE
    UVC_VC_HEADER,      // bDescriptorSubType = VC_HEADER subtype
    W_TO_B(0x0110),     // bcdUVC = 0x0110 version 1.1.
    W_TO_B(VC_DESC_SIZE),// wTotalLength = Total size of class-specific descriptors
    DW_TO_B(0x005B8D80),// dwClockFrequency = deprecated. 0x005B8D80 This device will provide timestamps and a device clock reference based on a 6MHz clock.
    0x01,               // bInCollection = Number of streaming interfaces.
    0x01,               // baInterfaceNr(1) = VideoStreaming interface 1 belongs to this VideoControl interface.

    // Input Terminal Descriptor (Camera)
    17,                 // bLength = Size of this descriptor, in bytes.
    UVC_DT_CS_INTERFACE,// bDescriptorType = UVC_DT_CS_INTERFACE
    UVC_VC_INPUT_TERMINAL,// bDescriptorSubtype = VC_INPUT_TERMINAL subtype
    INPUT_TERMINAL_ID,  // bTerminalID = ID of this input terminal
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
    UVC_DT_CS_INTERFACE,// bDescriptorType = UVC_DT_CS_INTERFACE
    UVC_VC_OUTPUT_TERMINAL, // bDescriptorSubtype = VC_OUTPUT_TERMINAL
    OUTPUT_TERMINAL_ID, // bTerminalID = ID of this terminal
    W_TO_B(UVC_TT_STREAMING),// wTerminalType = TT_STREAMING type. This terminal is a USB streaming terminal.
    0x00,               // bAssocTerminal = No association
    PU_TERMINAL_ID,     // bSourceID = The input pin of this unit is connected to the output pin of this unit.
    0x00,               // iTerminal = Unused
    
    // Selector Unit Terminal Descriptor
    7,                  // bLength                
    UVC_DT_CS_INTERFACE,// bDescriptorType        
    UVC_VC_SELECTOR_UNIT,// bDescriptorSubtype      (SELECTOR_UNIT)
    SU_TERMINAL_ID,     // bUnitID                
    1,                  // bNrInPins              
    INPUT_TERMINAL_ID,  // baSource( 0)           
    0,                  // iSelector

    // Processing Unit Descriptor
    12,                 // bLength = Size of this descriptor, in bytes.
    UVC_DT_CS_INTERFACE, // bDescriptorType = UVC_DT_CS_INTERFACE
    UVC_VC_PROCESSING_UNIT,// bDescriptorSubtype = VC_PROCESSING_UNIT
    PU_TERMINAL_ID,     // bUnitID = ID of this unit
    SU_TERMINAL_ID,     // bSourceID = This input pin of this unit is connected to the output pin of other unit.
    0x00, 0x00,         // wMaxMultiplier = unused
    0x02,               // bControlSize = Size of the bmControls field, in bytes.
    0x01, 0x00,         // bmControls = Brightness control supported
    0x00,               // iProcessing = Unused
    0x00,               // bmVideoStandards

    // Standard VideoStreaming Interface Descriptor - - Operational Alternate Setting 0
    9,                  // bLength = Size of this descriptor, in bytes.
    USB_DT_INTERFACE,   // bDescriptorType = INTERFACE descriptor type
    VIDEOS_IFACE,       // bInterfaceNumber = Index of this interface
    0x00,               // bAlternateSetting = Index of this alternate setting
    0x00,               // bNumEndpoints = 0 endpoints – no bandwidth used
    CC_VIDEO,           // bInterfaceClass = CC_VIDEO
    UVC_SC_VIDEOSTREAMING,// bInterfaceSubClass = SC_VIDEOSTREAMING
    0x00,               // bInterfaceProtocol = PC_PROTOCOL_UNDEFINED
    0x00,               // iInterface = Unused

    // Class-specific VideoStreaming Header Descriptor (Input)
    14,                 // bLength = Size of this descriptor, in bytes.
    UVC_DT_CS_INTERFACE,// bDescriptorType = UVC_DT_CS_INTERFACE
    UVC_VS_INPUT_HEADER,// bDescriptorSubtype = VS_INPUT_HEADER.
    0x01,               // bNumFormats = One format descriptor follows.
    W_TO_B(VS_DESC_SIZE),// wTotalLength = Total size of class-specific VideoStreaming interface descriptors
    UVC_TX_ENDPOINT|0x80, // bEndpointAddress = Address of the isochronous endpoint used for video data
    0x00,               // bmInfo = No dynamic format change supported
    OUTPUT_TERMINAL_ID, // bTerminalLink = This VideoStreaming interface supplies terminal ID 2 (Output Terminal).
    0x01,               // bStillCaptureMethod = Device supports still image capture method 1.
    0x00,               // bTriggerSupport = Hardware trigger supported for still image capture
    0x00,               // bTriggerUsage = Hardware trigger should initiate a still image capture.
    0x01,               // bControlSize = Size of the bmaControls field
    0x00,               // bmaControls = No VideoStreaming specific controls are supported.

    // Class-specific VideoStreaming Format Descriptor
    27,                 // bLength = Size of this descriptor, in bytes.
    UVC_DT_CS_INTERFACE,        // bDescriptorType = UVC_DT_CS_INTERFACE
    UVC_VS_FORMAT_UNCOMPRESSED, // bDescriptorSubtype = UVC_VS_FORMAT_UNCOMPRESSED
    1,                  // bFormatIndex         
    1,                  // bNumFrameDescriptors
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
    UVC_DT_CS_INTERFACE,// bDescriptorType        
    UVC_VS_FRAME_UNCOMPRESSED,// bDescriptorSubtype     
    1,                  // bFrameIndex            
    0x01,               // bmCapabilities
    W_TO_B(WIDTH),      // wWidth 
    W_TO_B(HEIGHT),     // wHeight  
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
    VIDEOS_IFACE,       // bInterfaceNumber = Index of this interface
    0x01,               // bAlternateSetting = Index of this alternate setting
    0x01,               // bNumEndpoints = 0 endpoints – no bandwidth used
    CC_VIDEO,           // bInterfaceClass = CC_VIDEO
    UVC_SC_VIDEOSTREAMING,// bInterfaceSubClass = SC_VIDEOSTREAMING
    0x00,               // bInterfaceProtocol = PC_PROTOCOL_UNDEFINED
    0x00,               // iInterface = Unused

    // Standard VS Isochronous Video Data Endpoint Descriptor
    7,                  // bLength = Size of this descriptor, in bytes.
    USB_DT_ENDPOINT,    // bDescriptorType = ENDPOINT
    UVC_TX_ENDPOINT|0x80,// bEndpointAddress = IN endpoint 2
    0x01,               // bmAttributes = Isochronous transfer type. 
    W_TO_B(UVC_TX_SIZE),// wMaxPacketSize = Max packet size
    10,                 // bInterval = One frame interval BUGBUG what means??
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
    W_TO_B(CONFIG1_DESC_SIZE),  // wTotalLength
    2,                  // bNumInterfaces
    1,                  // bConfigurationValue
    0,                  // iConfiguration
    0x80,               // bmAttributes  = Bus-powered device, no remote wakeup capability
    0xFA,               // bMaxPower
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
    {MKWORD(USB_DT_OTHER_SPEED_CONFIGURATION, 0x00), 0x0000, other_speed_descriptor, /* size ok */ sizeof(config1_descriptor)},
    {MKWORD(USB_DT_STRING, 0x00), 0x0000, (const uint8_t *)&string0, 4},
    {MKWORD(USB_DT_STRING, STR_MANUFACTURER_I), 0x0409, (const uint8_t *)&string1, sizeof(STR_MANUFACTURER)},
    {MKWORD(USB_DT_STRING, STR_PRODUCT_I), 0x0409, (const uint8_t *)&string2, sizeof(STR_PRODUCT)}
};
#define NUM_DESC_LIST (sizeof(descriptor_list)/sizeof(struct descriptor_list_struct))

#define VSPC_HINT_dwFrameInterval (1<<0)
#define VSPC_HINT_wKeyFrameRate (1<<1)
#define VSPC_HINT_wPFrameRate (1<<2)
#define VSPC_HINT_wCompQuality (1<<3)
#define VSPC_HINT_wCompWindowSize (1<<4)

#define VSPC_FINFO_FID_REQUIRED (1<<0)
#define VSPC_FINFO_EOF (1<<1)
/* 34 bytes */
struct vs_probe_commit
{
    uint16_t bmHint;            // Bitfield indicating what fields shall be kept fixed
    uint8_t  bFormatIndex;      // Video format index from a format descriptor.
    uint8_t  bFrameIndex;       // Video frame index from a frame descriptor.
    uint32_t dwFrameInterval;   // Frame interval in 100 ns units. 
    uint16_t wKeyFrameRate;     // Key frame rate in key-frame per video-frame units.
    uint16_t wPFrameRate;       // PFrame rate in PFrame/key frame units.
    uint16_t wCompQuality;      // Compression quality control in abstract units 0-10000 (highest).
    uint16_t wCompWindowSize;   // Window size for average bit rate control.
    uint16_t wDelay;            // Internal interface latency (ms) from data capture to presentation on USB.
    uint32_t dwMaxVideoFrameSize;// Maximum video frame or codec-specific segment size in bytes.   
    uint32_t dwMaxPayloadTransferSize;// maximum number of i/o bytes the device handle in a single payload 
    uint32_t dwClockFrequency;  // The device clock frequency in Hz
    uint8_t  bmFramingInfo;     // Bitfield control
    uint8_t  bPreferedVersion;  // The preferred payload format version 
    uint8_t  bMinVersion;       // The min payload format version
    uint8_t  bMaxVersion;       // The max payload format version
};
#define PC_INIT(p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15, p16)\
    {p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15, p16}

// zero when we are not configured, non-zero when enumerated
static volatile uint8_t usb_configuration = 0;
static volatile uint8_t transmit_flush_timer = 0;
static uint8_t last_error = UVC_ERR_SUCCESS;
static volatile uint8_t videos_alt_setting = 0;
static volatile uint8_t streaming = 0;
static volatile uint8_t fid = 0;

/* Helper macros for UVC controls.
   You can use DEFINE_UVC_CONTROL to define a uvc control and greatly simplify
   access through needed GET_XXX / SET_CUR uvc requests. The read-only fields
   (.e.g MIN/MAX etc) will be stored in program memory to save RAM.
   Use CTL_CALL to dispatch uvc requests for the control.
*/   
#define DEFINE_UVC_CONTROL(name, type, _min, _max, _res, _def, __len, _info_flags) \
type name = _def;\
const uint8_t PROGMEM name##_len = __len;\
static struct name##_ctl_info_ {\
    type min;\
    type max;\
    type res;\
    type def;\
    uint8_t info_flags;\
} PROGMEM name##_ctl_info = {\
    _min, _max, _res, _def, _info_flags \
}

#define CTL_CALL(name, _bRequest,  _wLength)\
    ctl_req(_bRequest, (uint8_t *)&name, (const uint8_t *)& name##_ctl_info, \
            name##_len, _wLength)

/* Control definitions */
DEFINE_UVC_CONTROL(brightness, int16_t, 0, 100, 1, 50, 2, GINFO_SUPPORT_GET | GINFO_SUPPORT_SET);
DEFINE_UVC_CONTROL(probe_commit, struct vs_probe_commit,
    /* min */ PC_INIT(0,1,1, 333333,0,0,0,0,0,FRAME_SIZE,FRAME_SIZE,16000000L,0,0,0,0),
    /* max */ PC_INIT(0,1,1,1333333,0,0,0,0,0,FRAME_SIZE,FRAME_SIZE,16000000L,0,0,0,0),
    /* res */ PC_INIT(0,1,1, 333333,0,0,0,0,0,         0,         0,        0,0,0,0,0),
    /* def */ PC_INIT(0,1,1, 333333,0,0,0,0,0,FRAME_SIZE,FRAME_SIZE,16000000L,0,0,0,0),
    34,
    GINFO_SUPPORT_GET | GINFO_SUPPORT_SET
    );

// initialize USB
void usb_init(void)
{
    HW_CONFIG();
    USB_CONFIG();               // enable USB
    PLL_CONFIG();               // config PLL
    while (!(PLLCSR & (1<<PLOCK))) 
        ;   // wait for PLL lock
    USB_CONFIG();               // start USB clock
    UDCON = 0;              // enable attach resistor
    usb_configuration = 0;
    UDIEN = (1<<EORSTE)|(1<<SOFE);
    sei();
}

static inline uint8_t usb_configured(void)
{
    return usb_configuration;
}
// Misc functions to wait for ready and send/receive packets
static inline void usb_wait_in_ready(void)
{
    while (!(UEINTX & (1<<TXINI))) ;
}
static inline void usb_ack_in(void)
{
    UEINTX &= ~(1<<TXINI);
}
/* Cuases hw to switch banks */
static inline void usb_ack_bank(void)
{
    //UEINTX = ~((int8_t)(1<<FIFOCON));
    UEINTX &= ~((1 << TXINI) | (1 << FIFOCON));
}
static inline uint8_t usb_rw_allowed(void)
{
    return UEINTX & (1<<RWAL);
}
static inline uint16_t usb_available(void)
{
    uint16_t ret = UEBCHX;
    ret = (ret << 8) | UEBCLX;
    return 240;
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

#define UVC_PHI_DEF (UVC_PHI_EOF | UVC_PHI_EOH | (fid&1))
uint8_t uggly = 0;
uint8_t uggly2 = 0;
static inline void _send_frame(void)
{
    uint16_t avail, obank;
    uint16_t h,w,n;

    if(!usb_rw_allowed())
        return;

    usb_wait_in_ready();
    avail = usb_available();
    if(avail < 2) {
        if(uggly++ == 0)
            DBG("Q??\r\n");
        return;
    }

    if(uggly2++ == 0)
        DBG("I??\r\n");

    /* Write header */
    UEDATX = 2; // write header len
    UEDATX = UVC_PHI_DEF; // write header bmHeaderInfo bitmap
    fid = !fid; // flip frame id bit
    avail -= 2;

    for(h=0; h < 240; h++) {
        w = 320*2;
        do {
            n=MIN(avail, w);
            if(n == 0) {
                DBG("W??\r\n");
                return;
            }
            while(n) {
                UEDATX=0xcc;                
                n--;
            }
            w-=n;
            avail-=n;
            if(!avail) {                
                usb_ack_bank();
                usb_wait_in_ready();
                obank = avail = usb_available();
            }            
        } while(w > 0);
    }

    if(obank != avail) {
        usb_ack_bank();
    }
}

void send_frame(void)
{
    uint8_t intr_state;

    intr_state = SREG;
    cli();
    UENUM = UVC_TX_ENDPOINT;
    _send_frame();
    SREG = intr_state;
    sei();
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
        if(streaming)
        {            
            send_frame();
        }
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

static uint8_t ctl_req(uint8_t bRequest, uint8_t *ctl, const uint8_t *ctl_info, uint8_t ctl_len, uint16_t wLength)
{
    uint8_t *wptr;
    const uint8_t *rptr;
    uint8_t len, i, n;
    uint8_t write = 0;
    uint8_t pgmem = 0;
    uint16_t ctl_len16 = ctl_len;

    len = (uint8_t)wLength;
    
    switch(bRequest) {
    case UVC_REQ_SET_CUR:
        wptr = ctl;
        write = 1;
        break;
    case UVC_REQ_GET_CUR:
        rptr = ctl;
        break;
    case UVC_REQ_GET_MIN:
        rptr = ctl_info;
        pgmem = 1;
        break;
    case UVC_REQ_GET_MAX:
        rptr = ctl_info + (1 * ctl_len);
        pgmem = 1;
        break;
    case UVC_REQ_GET_RES:
        rptr = ctl_info + (2 * ctl_len);
        pgmem = 1;
        break;
    case UVC_REQ_GET_DEF:
        rptr = ctl_info + (3 * ctl_len);
        pgmem = 1;
        break;
    case UVC_REQ_GET_LEN:
        switch(len) {
        case 1:
            rptr = &ctl_len;
            break;
        case 2:
            rptr = (const uint8_t *)&ctl_len16;
            break;
        default:
            return UVC_ERR_INVALID_REQUEST;
        }
        break;
    case UVC_REQ_GET_INFO:
        len = 1;
        rptr = ctl_info + (4 * ctl_len);
        pgmem = 1;
        break;
    default:
       return UVC_ERR_INVALID_REQUEST;
    }

    if(write) {
        //DBGV("\r\nwp=%p l=%x pg=%x\r\n", wptr, len, pgmem);
        do {
            usb_wait_receive_out();
            n = len < ENDPOINT0_SIZE ? len : ENDPOINT0_SIZE;
            for(i = 0; i < n; i++) {
                *wptr = UEDATX;
                wptr++;
            }
            len -= n;       
            usb_ack_out();
            usb_ack_in();
        }  while (len || n == ENDPOINT0_SIZE);    
    } else {
        //DBGV("\r\nrp=%p l=%x pg=%x\r\n", rptr, len, pgmem);
        do {
            usb_wait_in_ready();
            n = len < ENDPOINT0_SIZE ? len : ENDPOINT0_SIZE;
            for (i = n; i; i--) {
                if(pgmem)
                    UEDATX = pgm_read_byte(rptr++);
                else
                    UEDATX = *rptr++;
            }   
            len -= n;
            usb_ack_in();
        } while (len || n == ENDPOINT0_SIZE);
    }

    return UVC_ERR_SUCCESS;
}

static inline uint8_t su_term_req(uint8_t bRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength)
{
    uint8_t cs = MSB(wValue);
    uint8_t unit;
    uint8_t ret = UVC_ERR_SUCCESS;

    DBGV("su %x V=%x I=%x L=%x", bRequest, wValue, wIndex, wLength);
    
    if(cs != UVC_SU_INPUT_SELECT_CONTROL || wLength != 1) {        
        return UVC_ERR_INVALID_CONTROL;
    }
    
    switch(bRequest) {
        case UVC_REQ_SET_CUR:
            usb_wait_receive_out();
            unit = UEDATX;
            if(unit != 1)
                ret = UVC_ERR_OUT_OF_RANGE;
            usb_ack_out();
            usb_ack_in();
            break;
        case UVC_REQ_GET_CUR :
        case UVC_REQ_GET_MIN :
        case UVC_REQ_GET_MAX :
        case UVC_REQ_GET_RES :
            usb_wait_in_ready();
            UEDATX = 1;
            usb_ack_in();
            break;
        case UVC_REQ_GET_INFO:
            usb_wait_in_ready();
            UEDATX = GINFO_SUPPORT_GET | GINFO_SUPPORT_SET;
            usb_ack_in();            
            break;
        default:
            ret = UVC_ERR_INVALID_REQUEST;
    }

    return ret;    
}

static inline uint8_t pu_term_req(uint8_t bRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength)
{
    uint8_t cs = MSB(wValue);
    uint8_t ret = UVC_ERR_SUCCESS;
    
    DBGV("pu %x V=%x I=%x L=%x", bRequest, wValue, wIndex, wLength);
   
    switch(cs) {
    case UVC_PU_BRIGHTNESS_CONTROL:
        ret = CTL_CALL(brightness, bRequest, wLength);
        break;
    default:
        ret = UVC_ERR_INVALID_CONTROL;
    }

    return ret;      
}

static inline uint8_t in_term_req(uint8_t bRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength)
{
    //uint8_t cs = MSB(wValue);
    uint8_t ret = UVC_ERR_SUCCESS;
    
    DBGV("in %x V=%x I=%x L=%x", bRequest, wValue, wIndex, wLength);

    switch(bRequest) {
        case UVC_REQ_SET_CUR:
        case UVC_REQ_GET_CUR :
        case UVC_REQ_GET_MIN :
        case UVC_REQ_GET_MAX :
        case UVC_REQ_GET_RES :
        case UVC_REQ_GET_LEN:
        case UVC_REQ_GET_INFO:
        case UVC_REQ_GET_DEF:
        default:
            ret = UVC_ERR_INVALID_REQUEST;
    }    

    return ret;  
}

static inline uint8_t out_term_req(uint8_t bRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength)
{
    //uint8_t cs = MSB(wValue);
    uint8_t ret = UVC_ERR_SUCCESS;
    
    DBGV("out %x V=%x I=%x L=%x", bRequest, wValue, wIndex, wLength);

    switch(bRequest) {
        case UVC_REQ_SET_CUR:
        case UVC_REQ_GET_CUR :
        case UVC_REQ_GET_MIN :
        case UVC_REQ_GET_MAX :
        case UVC_REQ_GET_RES :
        case UVC_REQ_GET_LEN:
        case UVC_REQ_GET_INFO:
        case UVC_REQ_GET_DEF:
        default:
            ret = UVC_ERR_INVALID_REQUEST;
    }    

    return ret;  
}

static inline uint8_t videoc_req(uint8_t bRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength)
{
    uint8_t cs = MSB(wValue);
    uint8_t ret = UVC_ERR_SUCCESS;

    DBGV("vc %x V=%x I=%x L=%x", bRequest, wValue, wIndex, wLength);

    switch(cs) {
    case UVC_VC_REQUEST_ERROR_CODE_CONTROL:
        switch(bRequest) {
            case UVC_REQ_GET_CUR:
                usb_wait_in_ready();
                UEDATX = last_error;
                usb_ack_in();
                break;
            case UVC_REQ_GET_INFO:
                usb_wait_in_ready();
                UEDATX = GINFO_SUPPORT_GET;
                usb_ack_in();            
                break;            
            default:
                ret = UVC_ERR_INVALID_REQUEST;
        }
        break;
    default:
        ret = UVC_ERR_INVALID_CONTROL;
    } 

    return ret;  
}

static void adjust_after_probe(void)
{
    DBG("after probe\r\n");

}

static int verify_after_commit(void)
{
    DBG("commit!\r\n");
    return UVC_ERR_SUCCESS;
}

static inline uint8_t videos_req(uint8_t bRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength)
{
    uint8_t cs = MSB(wValue);
    uint8_t ret = UVC_ERR_SUCCESS;
    
    //DBGV("vs %x V=%x I=%x L=%x", bRequest, wValue, wIndex, wLength);

    switch(cs) {
    case UVC_VS_PROBE_CONTROL:
        ret = CTL_CALL(probe_commit, bRequest, wLength);
        if(bRequest == UVC_REQ_SET_CUR && ret == UVC_ERR_SUCCESS)
            adjust_after_probe();
        break;
    case UVC_VS_COMMIT_CONTROL:
        ret = CTL_CALL(probe_commit, bRequest, wLength);
        if(bRequest == UVC_REQ_SET_CUR && ret == UVC_ERR_SUCCESS)
            ret = verify_after_commit();
        break;
    default:
        ret = UVC_ERR_INVALID_REQUEST;
    }        

    return ret;  
}
// USB Endpoint Interrupt - endpoint 0 is handled here.  The
// other endpoints are manipulated by the user-callable
// functions, and the start-of-frame interrupt.
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
                    DBGV("Unk desc wV=%d wI=%d\r\n", wValue, wIndex);
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
                    /* Other speed config is just like config, so why waste pgm space ? */
                    if(wIndex == MKWORD(USB_DT_OTHER_SPEED_CONFIGURATION, 0) &&
                       desc_addr - other_speed_descriptor == sizeof(other_speed_descriptor))
                            desc_addr = config1_descriptor + sizeof(other_speed_descriptor);
 
                    UEDATX = pgm_read_byte(desc_addr++);
                }
                len -= n;
                usb_ack_in();
            } while (len || n == ENDPOINT0_SIZE);
            return;
        }
        if (bRequest == USB_REQ_SET_ADDRESS) {
            usb_ack_in();
            usb_wait_in_ready();
            UDADDR = wValue | (1<<ADDEN);
            return;
        }
        
        if (bRequest == USB_REQ_SET_CONFIGURATION && bmRequestType == 0) {
            usb_configuration = wValue;
            usb_ack_in();
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
            /* Reset all endpoints except ep 0 */
            UERST = 0x1E;
            UERST = 0;
            DBG("Configured!\r\n");
            return;
        }
        if (bRequest == USB_REQ_GET_CONFIGURATION && bmRequestType == 0x80) {
            usb_wait_in_ready();
            UEDATX = usb_configuration;
            usb_ack_in();
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
            usb_ack_in();
            return;
        }
#ifdef SUPPORT_ENDPOINT_HALT
        if ((bRequest == USB_REQ_CLEAR_FEATURE || 
             bRequest == USB_REQ_SET_FEATURE) && 
             bmRequestType == 0x02 && wValue == 0) {
                i = wIndex & 0x7F;
                if (i >= 1 && i <= MAX_ENDPOINT) {
                    usb_ack_in();
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
            if(wIndex == VIDEOS_IFACE) {
                videos_alt_setting = wValue;
                if(!streaming && videos_alt_setting > 0) {
                    /* start streaming */
                    fid = 0;
                    streaming = 1;
                }
                else if(streaming && videos_alt_setting == 0) {
                    /* stop streaming */                    
                    streaming = 0;
                }
                DBGV("strm=%d\r\n", streaming);
            }
            usb_wait_in_ready();
            usb_ack_in();
            return;
        }

        if((bRequest == UVC_REQ_SET_CUR && bmRequestType == 33) ||
           (bmRequestType == 161))
        {
            uint8_t unitid = MSB(wIndex);
            uint8_t iface = LSB(wIndex);

            switch(unitid)
            {
            case INPUT_TERMINAL_ID :
                last_error = in_term_req(bRequest, wValue, wIndex, wLength);
                break;
            case OUTPUT_TERMINAL_ID:
                last_error = out_term_req(bRequest, wValue, wIndex, wLength);
                break;
            case SU_TERMINAL_ID :
                last_error = su_term_req(bRequest, wValue, wIndex, wLength);
                break;
            case PU_TERMINAL_ID:
                last_error = pu_term_req(bRequest, wValue, wIndex, wLength);
                break;
            case 0:
                switch(iface) {
                    case VIDEOC_IFACE:
                        last_error = videoc_req(bRequest, wValue, wIndex, wLength);
                        break;
                    case VIDEOS_IFACE:
                        last_error = videos_req(bRequest, wValue, wIndex, wLength);
                        break;
                    default:
                        last_error = UVC_ERR_INVALID_UNIT;
                }
                break;
            default:
                last_error = UVC_ERR_INVALID_UNIT;
            }
            DBGV(" =%x\r\n", last_error);
            if(last_error == UVC_ERR_SUCCESS)
                return;
        }
        if(bRequest == UVC_REQ_SET_CUR && bmRequestType == 34) {
            DBG("set_cur_e\r\n");
        }
        if(bmRequestType == 162) {
            DBG("get_XXX_e\r\n");
        }
        DBGV("? %x V=%x I=%x L=%x\r\n", bRequest, wValue, wIndex, wLength);
    }

    usb_stall();
}
