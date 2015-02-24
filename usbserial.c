#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>

#include "mk20dx128.h"
#include "usbcommon.h"
#include "usbserial.h"

/* Estimate the number of digits in an integer of type t.  The number
 * of digits is equal to log_radix (maxvalue(t)) = log_radix(2 ^
 * bits(t)) = bits(t) / log2(radix).  Add to this one for the lost
 * remainder due to integer division, one for the possible sign and 1
 * for the terminating \0. */

#define ITOSTR_BUFFER_BOUND(t, log2r) ((sizeof (t) * 8) / log2r + 3)

static void utostr(uint64_t n, int radix, int width)
{
    int log2r, r;

    if (width <= 0) {
        width = 1;
    }

    for(log2r = 0, r = radix ; r > 1 ; r /= 2, log2r += 1);

    {
        const int N = ITOSTR_BUFFER_BOUND(n, log2r);
        char buffer[N], *p;
        int i, k;

        p = buffer + N;

        for (; n > 0 ; *(p -= 1) = ((n % radix) +
                                    (n % radix < 10 ? '0' : 'a' - 10)),
                       n /= radix);

        k = buffer + N - p;

        for (i = k ; i < width ; i += 1) {
            usb_write("0", 1, 0);
        }

        if (k > 0) {
            usb_write(p, k, 0);
        }
    }
}

static void itostr(int64_t n, int radix, int width, int plus)
{
    if (n < 0) {
        usb_write("-", 1, 0);
        utostr((uint64_t)(-n), radix, width);
    } else {
        if (plus == 1) {
            usb_write("+", 1, 0);
        } else if ( plus == 2) {
            usb_write(" ", 1, 0);
        }

        utostr((uint64_t)n, radix, width);
    }
}

static void ftostr(float n, int width, int precision, int plus)
{
    if (n < 0) {
        usb_write("-", 1, 0);

        utostr((uint64_t)-n, 10, width - precision - 1 - (n < 0 || plus > 0));
    } else {
        if (plus == 1) {
            usb_write("+", 1, 0);
        } else if (plus == 2) {
            usb_write(" ", 1, 0);
        }

        utostr((uint64_t)n, 10, width - precision - 1 - (n < 0 || plus > 0));
    }

    usb_write(".", 1, 0);
    utostr((uint64_t)(fabs(fmodf(n, 1) * pow10(precision))), 10, precision);
}

int usbserial_printf(const char *format, ...)
{
    va_list ap;
    const char *c, *d;

    if (!usbserial_is_dtr()) {
        return -1;
    }

    va_start(ap, format);

    for (c = format ; *c != '\0' ;) {
        for (d = c; *d != '%' && *d != '\0' ; d += 1);

        if (d > c) {
            usb_write(c, d - c, 0);
        }

        c = d;

        if (*c == '%') {
            int size = 2, base = -1, sign = -1, width = -1, precision = -1;
            int plus = 0;

            for (c += 1 ; ; c += 1) {
                if (*c == '%') {
                    usb_write("%", 1, 0);
                    break;
                } else if (*c == '+') {
                    plus = 1;
                } else if (*c == ' ') {
                    plus = 2;
                } else if (((*c >= '0' && *c <= '9') || *c == '.') &&
                           width < 0) {
                    if (*c != '.') {
                        width = strtol(c, (char **)&c, 10);
                    }

                    if (*c == '.') {
                        precision = strtol(c + 1, (char **)&c, 10);
                    }

                    c -= 1;
                } else if (*c == 'l') {
                    size += 1;
                } else if (*c == 'h') {
                    size -= 1;
                } else {
                    if (*c == 's') {
                        char *s;

                        s = va_arg(ap, char *);
                        usb_write(s, width > 0 ? width : strlen(s), 0);
                        break;
                    } else if (*c == 'f') {
                        ftostr((float)va_arg(ap, double),
                               width, precision > 0 ? precision : 5, plus);
                        break;
                    } else if (*c == 'd') {
                        base = 10;
                        sign = 1;
                    } else if (*c == 'u') {
                        base = 10;
                        sign = 0;
                    } else if (*c == 'x') {
                        base = 16;
                        sign = 0;
                    } else if (*c == 'b') {
                        base = 2;
                        sign = 0;
                    } else {
                        break;
                    }

                    if (sign) {
                        switch (size) {
                        case 3:
                            itostr(va_arg(ap, int64_t), base, width, plus);
                            break;
                        case 2:
                            itostr(va_arg(ap, int32_t), base, width, plus);
                            break;
                        default:
                            itostr(va_arg(ap, int), base, width, plus);
                            break;
                        }
                    } else {
                        switch (size) {
                        case 3:
                            utostr(va_arg(ap, uint64_t), base, width);
                            break;
                        case 2:
                            utostr(va_arg(ap, uint32_t), base, width);
                            break;
                        default:
                            utostr(va_arg(ap, unsigned int), base, width);
                            break;
                        }
                    }

                    break;
                }
            }

            c += 1;
        }
    }

    va_end(ap);

    usb_write(NULL, 0, 1);

    return 0;
}

int usbserial_set_state(uint16_t state)
{
    uint16_t notification[5];

    notification[0] = NOTIFICATION_SERIALSTATE;
    notification[1] = 0;
    notification[2] = COMMUNICATION_INTERFACE;
    notification[3] = 2;
    notification[4] = state;

    usb_interrupt((uint8_t *)notification, 10);

    return 0;
}

void usbserial_await_dtr()
{
    sleep_while(!(line_state & LINE_STATE_DTR));
}

int usbserial_is_dtr()
{
    return line_state & LINE_STATE_DTR;
}

void usbserial_await_rts()
{
    sleep_while(!(line_state & LINE_STATE_RTS));
}

int usbserial_is_rts()
{
    return line_state & LINE_STATE_RTS;
}
