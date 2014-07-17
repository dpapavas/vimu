#ifndef _USBSERIAL_H_
#define _USBSERIAL_H_

#include <stdio.h>
#include "usbcommon.h"

int usbserial_is_dtr();
void usbserial_await_dtr();

int usbserial_is_rts();
void usbserial_await_rts();

int usbserial_write(const char *s, int n, int flush);
int usbserial_read(char *buffer);

#define usbserial_flush() usbserial_write(0, 0, 1)

int usbserial_printf(const char *format, ...);

#define usbserial_trace(format, ...)                                    \
    usbserial_printf("%s: %d: " format, __FILE__, __LINE__, ##__VA_ARGS__);

#define usbserial_assert(cond)                                  \
    if(!(cond)) {                                               \
        usbserial_trace("assertion failed: %s.\n", #cond);      \
        while(1) {                                              \
            delay_ms(50);                                       \
            toggle_led();                                       \
        }                                                       \
    }

#endif
