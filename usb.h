#ifndef _USB_H_
#define _USB_H_

#include "stdint.h"

#define ENDPOINTS_N (3)
#define CONTROL_BUFFER_SIZE (64)

#define BDT_DESC_OWN ((uint32_t)1 << 7)
#define BDT_DESC_DATA1 ((uint32_t)1 << 6)
#define BDT_DESC_DTS ((uint32_t)1 << 3)
#define BDT_DESC_STALL ((uint32_t)1 << 2)
#define BDT_DESC_PID(n) ((n >> 2) & 0b1111)
#define BDT_DESC_BYTE_COUNT(n) ((n >> 16) & ((1 << 10) - 1))

#define BDT_PID_OUT 0x1
#define BDT_PID_IN 0x9
#define BDT_PID_SETUP 0xd

#define SETUP_DEVICE_GET_STATUS (0x0080)
#define SETUP_DEVICE_CLEAR_FEATURE (0x0100)
#define SETUP_DEVICE_SET_FEATURE (0x0300)
#define SETUP_DEVICE_SET_ADDRESS (0x0500)
#define SETUP_DEVICE_GET_DESCRIPTOR (0x0680)
#define SETUP_DEVICE_SET_DESCRIPTOR (0x0700)
#define SETUP_DEVICE_GET_CONFIGURATION (0x0880)
#define SETUP_DEVICE_SET_CONFIGURATION (0x0900)

#define SETUP_INTERFACE_GET_STATUS (0x0081)
#define SETUP_INTERFACE_CLEAR_FEATURE (0x0101)
#define SETUP_INTERFACE_SET_FEATURE (0x0301)
#define SETUP_INTERFACE_GET_INTERFACE (0x0a81)
#define SETUP_INTERFACE_SET_INTERFACE (0x1101)

#define SETUP_ENDPOINT_GET_STATUS (0x0082)
#define SETUP_ENDPOINT_CLEAR_FEATURE (0x0102)
#define SETUP_ENDPOINT_SET_FEATURE (0x0302)
#define SETUP_ENDPOINT_SYNCH_FRAME (0x1282)

#define SETUP_SET_LINE_CODING 0x2021
#define SETUP_GET_LINE_CODING 0x21a1
#define SETUP_SET_CONTROL_LINE_STATE 0x2221

#define LINE_STATE_DTR (1 << 0)
#define LINE_STATE_RTS (1 << 1)

#define DESCRIPTOR_TYPE(value) ((uint8_t)((value) >> 8))
#define DESCRIPTOR_TYPE_DEVICE 1
#define DESCRIPTOR_TYPE_DEVICE_CONFIGURATION 2
#define DESCRIPTOR_TYPE_DEVICE_QUALIFIER 6

#define ENDPOINT_DATA1 (1 << 0)
#define ENDPOINT_ODD (1 << 1)

#define PHASE_SETUP 0
#define PHASE_DATA_IN 1
#define PHASE_DATA_OUT 2
#define PHASE_STATUS 3

#define bdt_descriptor(count, data1)                                    \
    (((data1) ? BDT_DESC_DATA1 : 0) | BDT_DESC_DTS |                    \
     (((uint32_t)(count)) << 16))

#define bdt_entry(n, tx, odd) (&bdt[((n) << 2) | ((tx) << 1) | ((odd) != 0)])

#define LSB(s) ((s) & 0xff)
#define MSB(s) (((s) >> 8) & 0xff)

#define CONTROL_ENDPOINT 0
#define COM_ENDPOINT 1
#define DATA_ENDPOINT 2
#define COM_BUFFER_SIZE 16
#define DATA_BUFFER_SIZE 64

static uint8_t device_descriptor[] = {
    18,                         /* bLength */
    1,                          /* bDescriptorType */
    0x00, 0x02,                 /* bcdUSB */
    2,                          /* bDeviceClass (2 = CDC) */
    0,                          /* bDeviceSubClass */
    0,                          /* bDeviceProtocol */
    CONTROL_BUFFER_SIZE,        /* bMaxPacketSize0 */
    0xc0, 0x16,                 /* idVendor */
    0x86, 0x04,                 /* idProduct */
    0x00, 0x01,                 /* bcdDevice */
    0,                          /* iManufacturer */
    0,                          /* iProduct */
    0,                          /* iSerial */
    1,                          /* bNumConfigurations */
};

#define CONFIGURATION_DESCRIPTOR_SIZE (9+9+5+5+4+5+7+9+7+7)
static uint8_t configuration_descriptor[CONFIGURATION_DESCRIPTOR_SIZE] = {
    /* Configuration descriptor. */
    9,                                  /* bLength */
    2,                                  /* bDescriptorType */
    LSB(CONFIGURATION_DESCRIPTOR_SIZE), /* wTotalLength */
    MSB(CONFIGURATION_DESCRIPTOR_SIZE),
    1,					/* bNumInterfaces */
    1,					/* bConfigurationValue */
    0,					/* iConfiguration */
    0xC0,                               /* bmAttributes */
    50,					/* bMaxPower */
    
    /* Interface descriptor. */
    9,					/* bLength */
    4,					/* bDescriptorType */
    0,					/* bInterfaceNumber */
    0,					/* bAlternateSetting */
    1,					/* bNumEndpoints */
    0x02,       /* bInterfaceClass (2 = Communication Interface Class) */
    0x02,       /* bInterfaceSubClass (2 = ACM subclass) */
    0x01,       /* bInterfaceProtocol */
    0,          /* iInterface */
    
    /* CDC Header Functional Descriptor. */
    5,					/* bFunctionLength */
    0x24,                               /* bDescriptorType */
    0x00,                               /* bDescriptorSubtype */
    0x10, 0x01,				/* bcdCDC */
    
    /* Call Management Functional Descriptor/ */
    5,					/* bFunctionLength */
    0x24,                               /* bDescriptorType */
    0x01,                               /* bDescriptorSubtype */
    0x01,                               /* bmCapabilities */
    1,					/* bDataInterface */
    
    /* Abstract Control Management Functional Descriptor. */
    4,					/* bFunctionLength */
    0x24,                               /* bDescriptorType */
    0x02,                               /* bDescriptorSubtype */
    0x06,                               /* bmCapabilities */
    
    /* Union Functional Descriptor. */
    5,					/* bFunctionLength */
    0x24,                               /* bDescriptorType */
    0x06,                               /* bDescriptorSubtype */
    0,					/* bMasterInterface */
    1,					/* bSlaveInterface0 */
    
    /* Endpoint descriptor. */
    7,					/* bLength */
    5,                                  /* bDescriptorType */
    COM_ENDPOINT | 0x80,                /* bEndpointAddress */
    0x03,                               /* bmAttributes (0x03=intr) */
    COM_BUFFER_SIZE, 0,                 /* wMaxPacketSize */
    64,

    /* Interface descriptor. */
    9,               /* bLength */
    4,               /* bDescriptorType */
    1,               /* bInterfaceNumber */
    0,               /* bAlternateSetting */
    2,               /* bNumEndpoints */
    0x0A,            /* bInterfaceClass (0a = Data Interface Class) */
    0x00,            /* bInterfaceSubClass */
    0x00,            /* bInterfaceProtocol */
    0,

    /* Endpoint descriptor. */
    7,                          /* bLength */
    5,                          /* bDescriptorType */
    DATA_ENDPOINT,              /* bEndpointAddress */
    0x02,                       /* bmAttributes (0x02=bulk) */
    DATA_BUFFER_SIZE, 0,        /* wMaxPacketSize */
    0,

    /* Endpoint descriptor. */
    7,                          /* bLength */
    5,                          /* bDescriptorType */
    DATA_ENDPOINT | 0x80,       /* bEndpointAddress */
    0x02,                       /* bmAttributes (0x02=bulk) */
    DATA_BUFFER_SIZE, 0,        /* wMaxPacketSize */
    0                           /* bInterval */
};

typedef struct {
    uint32_t desc;
    void *buffer;
} bdtentry_t;

void initialize_usb();
int usb_initialized();

#endif
