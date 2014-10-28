#ifndef _USBCOMMON_H_
#define _USBCOMMON_H_

#include <stdint.h>

#define COMMUNICATION_INTERFACE 0
#define DATA_INTERFACE 1

#define CONTROL_ENDPOINT 0
#define NOTIFICATION_ENDPOINT 1
#define DATA_ENDPOINT 2

typedef void (*usb_data_in_callback)(uint8_t *data, int length);

void usb_initialize();
int usb_enumerated();
void usb_await_enumeration();
int usb_read(char **buffer, int *length);
int usb_write(const char *s, int n, int flush);
int usb_interrupt(uint8_t *buffer, int n);
void usb_set_data_in_callback (usb_data_in_callback new_callback);

#endif
