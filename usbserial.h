#ifndef _USBSERIAL_H_
#define _USBSERIAL_H_

#include <stdio.h>
#include "usbcommon.h"

#define NOTIFICATION_SERIALSTATE 0x20a1

#define LINE_STATE_DTR (1 << 0)
#define LINE_STATE_RTS (1 << 1)

#define SERIAL_STATE_DCD (1 << 0)
#define SERIAL_STATE_DSR (1 << 1)

void usbserial_initialize();

int usbserial_is_dtr();
void usbserial_await_dtr();

int usbserial_is_rts();
void usbserial_await_rts();

#define usbserial_flush() usbserial_write(0, 0, 1)

int usbserial_printf(const char *format, ...);
int usbserial_set_state(uint16_t state);

#define usbserial_trace(format, ...)                                    \
    usbserial_printf("%s: %d: " format, __FILE__, __LINE__, ##__VA_ARGS__);

#endif
