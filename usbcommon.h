#ifndef _USBCOMMON_H_
#define _USBCOMMON_H_

#include <stdint.h>

#define COMMUNICATION_INTERFACE 0
#define DATA_INTERFACE 1

#define CONTROL_ENDPOINT 0
#define NOTIFICATION_ENDPOINT 1
#define DATA_ENDPOINT 2

typedef void (*usb_DataInCallback)(uint8_t *data, int length);
typedef void (*usb_LineStateCallback)(uint8_t new_line_state);
typedef void (*usb_LineCodingCallback)(uint8_t *new_line_coding);
typedef void (*usb_SendBreakCallback)(uint16_t duration, void *userdata);

void usb_initialize();
int usb_enumerated();
void usb_await_enumeration();
int usb_write(const char *s, int n, int flush);
int usb_interrupt(uint8_t *buffer, int n);
void usb_set_data_in_callback (usb_DataInCallback new_callback);
void usb_set_line_state_callback(usb_LineStateCallback new_callback);
void usb_set_line_coding_callback(usb_LineCodingCallback new_callback);
void usb_set_send_break_callback(usb_SendBreakCallback new_callback,
                                 void *new_userdata);

#endif
