#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>

#include "mk20dx128.h"
#include "util.h"
#include "usb.h"

__attribute__ ((section(".usbdata.bdt")))
static bdtentry_t bdt[ENDPOINTS_N * 4];

__attribute__ ((aligned(4)))
static uint8_t control_rx[2][CONTROL_BUFFER_SIZE],
    ep2_rx[2][DATA_BUFFER_SIZE], ep2_tx[2][DATA_BUFFER_SIZE];


static struct {
    uint8_t *data;
    uint16_t length;
} outputs[ENDPOINTS_N], inputs[ENDPOINTS_N];

static uint8_t oddbits;
static uint8_t address, phase;
static volatile uint8_t configuration;
static volatile uint8_t line_state;
static volatile uint8_t line_coding[7];

static inline void process_setup_packet(uint16_t request, uint16_t value,
                                        uint16_t index, uint16_t length)
{
    outputs[CONTROL_ENDPOINT].data = NULL;
    outputs[CONTROL_ENDPOINT].length = 0;

    switch (request) {
    case SETUP_DEVICE_GET_DESCRIPTOR:
        /* GET_DESCRIPTOR request. */

        switch (DESCRIPTOR_TYPE(value)) {
        case DESCRIPTOR_TYPE_DEVICE:
            /* GET_DESCRIPTOR for DEVICE. */

            phase = PHASE_DATA_IN;
            outputs[CONTROL_ENDPOINT].data = device_descriptor;
            outputs[CONTROL_ENDPOINT].length = 18;
            break;

        case DESCRIPTOR_TYPE_DEVICE_QUALIFIER:
            /* GET_DESCRIPTOR for DEVICE QUALIFIER. */

            phase = PHASE_SETUP;

            USB0_ENDPT(0) |= USB_ENDPT_EPSTALL;
            break;

        case DESCRIPTOR_TYPE_DEVICE_CONFIGURATION:
            /* GET_DESCRIPTOR for DEVICE CONFIGURATION. */

            phase = PHASE_DATA_IN;
            outputs[CONTROL_ENDPOINT].data = configuration_descriptor;
            outputs[CONTROL_ENDPOINT].length = sizeof(configuration_descriptor);
            break;

        default:
            USB0_ENDPT(0) |= USB_ENDPT_EPSTALL;
        }
        break;

    case SETUP_DEVICE_GET_CONFIGURATION:
        /* GET_CONFIGURATION request. */

        phase = PHASE_DATA_IN;
        outputs[CONTROL_ENDPOINT].data = (uint8_t *)&configuration;
        outputs[CONTROL_ENDPOINT].length = sizeof(configuration);

        break;

    case SETUP_DEVICE_SET_CONFIGURATION:
        /* SET_CONFIGURATION request. */

        phase = PHASE_STATUS;

        if (value != 1) {
            USB0_ENDPT(0) |= USB_ENDPT_EPSTALL;
        } else {
            int i;

            /* Initialize endpoint 1. */

            USB0_ENDPT(1) = USB_ENDPT_EPTXEN | USB_ENDPT_EPHSHK;

            for (i = 0 ; i < 2 ; i += 1) {
                bdtentry_t *t;

                t = bdt_entry(COM_ENDPOINT, 1, i);
                t->buffer = NULL;
                t->desc = bdt_descriptor (0, 0);
            }

            /* Initialize endpoint 2. */

            USB0_ENDPT(2) = USB_ENDPT_EPRXEN | USB_ENDPT_EPTXEN |
                USB_ENDPT_EPHSHK;

            for (i = 0 ; i < 2 ; i += 1) {
                bdtentry_t *r, *t;

                r = bdt_entry(DATA_ENDPOINT, 0, i);
                t = bdt_entry(DATA_ENDPOINT, 1, i);

                r->buffer = ep2_rx[i];
                t->buffer = ep2_tx[i];

                r->desc = bdt_descriptor (DATA_BUFFER_SIZE, 0);
                t->desc = bdt_descriptor (0, 0);

                r->desc |= BDT_DESC_OWN;
            }

            configuration = (uint8_t)value;
        }
        break;

    case SETUP_DEVICE_SET_ADDRESS:
        /* SET_ADDRESS request. */

        phase = PHASE_STATUS;
        address = (uint8_t)value;
        break;

    case SETUP_SET_CONTROL_LINE_STATE:
        /* SET_CONTROL_LINE_STATE request. */

        phase = PHASE_STATUS;

        line_state = (uint8_t)value;
        break;

    case SETUP_SET_LINE_CODING:
        /* SET_LINE_CODING request. */

        phase = PHASE_DATA_OUT;
        inputs[CONTROL_ENDPOINT].data = (uint8_t *)line_coding;
        inputs[CONTROL_ENDPOINT].length =
            length < sizeof(line_coding) ?
            length : sizeof(line_coding);
        break;

    default:
        USB0_ENDPT(0) |= USB_ENDPT_EPSTALL;
    }
}

static void handle_data_transfer (uint8_t status)
{
    bdtentry_t *entry;

    entry = &(bdt[status >> 2]);

    switch (BDT_DESC_PID(entry->desc)) {
    case BDT_PID_IN:
        break;
    case BDT_PID_OUT:
        /* n = BDT_DESC_BYTE_COUNT(entry->desc); */

        entry->desc = bdt_descriptor (DATA_BUFFER_SIZE, 1);
        entry->desc |= BDT_DESC_OWN;
        break;
    default:
        USB0_ENDPT(2) |= USB_ENDPT_EPSTALL;
    }
}

static void handle_interrupt_transfer (uint8_t status)
{
    bdtentry_t *entry;

    entry = &(bdt[status >> 2]);

    switch (BDT_DESC_PID(entry->desc)) {
    case BDT_PID_IN:
        assert(0);
        break;
    case BDT_PID_OUT:
        assert(0);
        break;
    default:
        USB0_ENDPT(1) |= USB_ENDPT_EPSTALL;
    }
}

static void handle_control_transfer (uint8_t status)
{
    bdtentry_t *entry, *out;
    uint16_t *buffer, length, value, request, index;
    int n, data1;

    entry = &(bdt[status >> 2]);

    out = bdt_entry(CONTROL_ENDPOINT, 1, oddbits & (1 << CONTROL_ENDPOINT));
    data1 = !(entry->desc & BDT_DESC_DATA1);

    switch (BDT_DESC_PID(entry->desc)) {
    case BDT_PID_SETUP:
        /* We should be in the setup phase but we could also be in the
         * data in stage if the. */

        if (phase != PHASE_SETUP) {
            /* while (1) {delay_ms(500); toggle_led()}; */
            USB0_ENDPT(0) |= USB_ENDPT_EPSTALL;
            break;
        }

        /* Read the payload and release the buffer. */

        buffer = (uint16_t *)entry->buffer;
        request = buffer[0];
        value = buffer[1];
        index = buffer[2];
        length = buffer[3];

        assert(out != entry);
        entry->desc = bdt_descriptor (CONTROL_BUFFER_SIZE, 1);
        entry->desc |= BDT_DESC_OWN;

        process_setup_packet(request, value, index, length);

        /* Adjust the output to the maximum byte count
         * specified by the host. */

        if (outputs[CONTROL_ENDPOINT].length > length) {
            outputs[CONTROL_ENDPOINT].length = length;
        }

        if (phase == PHASE_DATA_IN ||
            phase == PHASE_STATUS) {
            /* If the next stage requires data output prepare the
             * buffers. */

            n = outputs[CONTROL_ENDPOINT].length < CONTROL_BUFFER_SIZE ?
                outputs[CONTROL_ENDPOINT].length : CONTROL_BUFFER_SIZE;

            assert (!(out->desc & BDT_DESC_OWN));
            out->buffer = (void *)outputs[CONTROL_ENDPOINT].data;
            out->desc = bdt_descriptor (n, data1);
            out->desc |= BDT_DESC_OWN;

            outputs[CONTROL_ENDPOINT].data += n;
            outputs[CONTROL_ENDPOINT].length -= n;
            oddbits ^= (1 << CONTROL_ENDPOINT);
        } else if (phase == PHASE_DATA_OUT) {
            /* If the request contained data read it. */

            n = BDT_DESC_BYTE_COUNT(entry->desc);

            assert (n > 0);

            memcpy(inputs[CONTROL_ENDPOINT].data, entry->buffer,
                   (inputs[CONTROL_ENDPOINT].length < n ?
                    inputs[CONTROL_ENDPOINT].length : n));

            inputs[CONTROL_ENDPOINT].data += n;
            inputs[CONTROL_ENDPOINT].length -= n;

            /* Prepare to send a zero-length data packet to
             * aknowledge proper reception during the
             * preceding OUT phase. */

            assert (!(out->desc & BDT_DESC_OWN));
            out->buffer = NULL;
            out->desc = bdt_descriptor (0, data1);
            out->desc |= BDT_DESC_OWN;

            oddbits ^= (1 << CONTROL_ENDPOINT);
        }

        USB0_CTL &= ~USB_CTL_TXSUSPENDTOKENBUSY;

        break;

    case BDT_PID_IN:
        if (phase == PHASE_DATA_IN) {
            /* Determine whether there's anything else to send
             * and prepare the buffer, otherwise transition to
             * the status phase. */

            n = outputs[CONTROL_ENDPOINT].length < CONTROL_BUFFER_SIZE ?
                outputs[CONTROL_ENDPOINT].length : CONTROL_BUFFER_SIZE;

            if (n > 0) {
                /* assert (out == entry); */
                assert (!(out->desc & BDT_DESC_OWN));
                out->buffer = (void *)outputs[CONTROL_ENDPOINT].data;
                out->desc = bdt_descriptor (n, data1);
                out->desc |= BDT_DESC_OWN;

                outputs[CONTROL_ENDPOINT].data += n;
                outputs[CONTROL_ENDPOINT].length -= n;

                oddbits ^= (1 << CONTROL_ENDPOINT);
            }
        } else if (phase == PHASE_STATUS ||
                   phase == PHASE_DATA_OUT) {
            phase = PHASE_SETUP;
        } else {
            USB0_ENDPT(0) |= USB_ENDPT_EPSTALL;
            break;
        }

        break;

    case BDT_PID_OUT:
        if (phase == PHASE_DATA_OUT) {
            /* Read the data. */

            n = BDT_DESC_BYTE_COUNT(entry->desc);

            memcpy(inputs[CONTROL_ENDPOINT].data, entry->buffer,
                   (inputs[CONTROL_ENDPOINT].length < n ?
                    inputs[CONTROL_ENDPOINT].length : n));

            inputs[CONTROL_ENDPOINT].data += n;
            inputs[CONTROL_ENDPOINT].length -= n;
        } else if (phase == PHASE_STATUS ||
                   phase == PHASE_DATA_IN) {
            phase = PHASE_SETUP;
        } else {
            USB0_ENDPT(0) |= USB_ENDPT_EPSTALL;
            break;
        }

        entry->desc = bdt_descriptor (CONTROL_BUFFER_SIZE, 1);
        entry->desc |= BDT_DESC_OWN;

        break;

    default:
        USB0_ENDPT(0) |= USB_ENDPT_EPSTALL;
    }

    /* Set address. */

    if (phase == PHASE_SETUP && USB0_ADDR != address) {
        USB0_ADDR = address;
    }
}

__attribute__((interrupt ("IRQ"))) void usb_isr()
{
    if (USB0_ISTAT & USB_ISTAT_USBRST) {
        int i;

        phase = PHASE_SETUP;

        /* Reset the even/odd BDT toggle bits. */

        USB0_CTL |= USB_CTL_ODDRST;
        oddbits = 0;

        /* Initialize endpoint 0 receive buffers and keep ownership
         * of transmission buffers. */

        for (i = 0 ; i < 2 ; i += 1) {
            bdtentry_t *r, *t;

            r = bdt_entry(CONTROL_ENDPOINT, 0, i);
            t = bdt_entry(CONTROL_ENDPOINT, 1, i);

            r->buffer = control_rx[i];
            t->buffer = NULL;

            r->desc = bdt_descriptor (CONTROL_BUFFER_SIZE, 0);
            t->desc = bdt_descriptor (0, 0);

            r->desc |= BDT_DESC_OWN;
        }

        /* Activate endpoint 0, reset all error flags and reset
         * address to 0. */

        USB0_ENDPT(0) = USB_ENDPT_EPRXEN | USB_ENDPT_EPTXEN | USB_ENDPT_EPHSHK;
        USB0_ERRSTAT = ~0;
        USB0_ADDR = 0;

        /* Enable interesting interrupts. */

        USB0_ERREN = ~0;
        USB0_INTEN = (USB_INTEN_USBRSTEN |
                      USB_INTEN_ERROREN |
                      USB_INTEN_TOKDNEEN |
                      USB_INTEN_SLEEPEN |
                      USB_INTEN_STALLEN);

        USB0_CTL &= ~(USB_CTL_ODDRST | USB_CTL_TXSUSPENDTOKENBUSY);

        USB0_ISTAT |= USB_ISTAT_USBRST;
    } else if (USB0_ISTAT & USB_ISTAT_ERROR) {
        USB0_ERRSTAT = ~0;
        USB0_ISTAT |= USB_ISTAT_ERROR;
    } else if (USB0_ISTAT & USB_ISTAT_STALL) {
        int i;

        for (i = 0 ; i < ENDPOINTS_N ; i += 1) {
            USB0_ENDPT(i) &= ~USB_ENDPT_EPSTALL;
        }

        USB0_ISTAT |= USB_ISTAT_STALL;
    } else if (USB0_ISTAT & USB_ISTAT_SLEEP) {
        USB0_ISTAT |= USB_ISTAT_SLEEP;
    } else while (USB0_ISTAT & USB_ISTAT_TOKDNE) {
        uint8_t status;
        int n;

        /* A new token has been received. */

        status = USB0_STAT;
        n = USB_STAT_ENDP(status);

        switch (n) {
        case 0: handle_control_transfer(status); break;
        case 1: handle_interrupt_transfer(status); break;
        case 2: handle_data_transfer(status); break;
        default:
            USB0_ENDPT(n) |= USB_ENDPT_EPSTALL;
        }

        USB0_ISTAT |= USB_ISTAT_TOKDNE;
    }
}

void usb_initialize()
{
    /* Enable the USB interrupt. */

    enable_interrupt(35);

    /* Set the USB clock source to MGCPLL with a divider for 48Mhz and
     * enable the USB clock. */

    SIM_CLKDIV2 |= SIM_CLKDIV2_USBDIV(1);
    SIM_SOPT2 |= SIM_SOPT2_USBSRC | SIM_SOPT2_PLLFLLSEL;
    SIM_SCGC4 |= SIM_SCGC4_USBOTG;

    /* Reset the USB module. */

    USB0_USBTRC0 |= USB_USBTRC0_USBRESET;
    while (USB0_USBTRC0 & USB_USBTRC0_USBRESET);

    /* Set the BDT base address which is aligned on a 512-byte
     * boundary and hence its first 9 bits are identically zero. */

    USB0_BDTPAGE1 = (uint8_t)((((uint32_t)bdt) >> 8) & 0xfe);
    USB0_BDTPAGE2 = (uint8_t)(((uint32_t)bdt) >> 16);
    USB0_BDTPAGE3 = (uint8_t)(((uint32_t)bdt) >> 24);

    /* Clear all USB ISR flags and enable weak pull-downs. */

    USB0_ISTAT = ~0;
    USB0_ERRSTAT = ~0;
    USB0_OTGISTAT = ~0;
    USB0_USBCTRL = USB_USBCTRL_PDE;

    /* Enable the USB module. */

    USB0_CTL = USB_CTL_USBENSOFEN;

    /* Enable USB reset interrupt. */

    USB0_INTEN = USB_INTEN_USBRSTEN;

    /* Enable d+ pull-up. */

    USB0_CONTROL |= USB_CONTROL_DPPULLUPNONOTG;
}

int usb_enumerated()
{
    return configuration;
}

void usb_await_enumeration()
{
    while(configuration == 0) {
        wfi();
    }
}

void usbserial_await_dtr()
{
    while(!(line_state & LINE_STATE_DTR)) {
        wfi();
    }
}

int usbserial_is_dtr()
{
    return line_state & LINE_STATE_DTR;
}

void usbserial_await_rts()
{
    while(!(line_state & LINE_STATE_RTS)) {
        wfi();
    }
}

int usbserial_is_rts()
{
    return line_state & LINE_STATE_RTS;
}

int usbserial_write(const char *s, int n, int flush)
{
    static uint8_t fill[2];
    volatile bdtentry_t *out;
    int i, o, m;

    if (configuration == 0 || !(line_state & LINE_STATE_DTR)) {
        return -1;
    }

    i = oddbits & (1 << DATA_ENDPOINT);

    if (n > 0) {
        o = fill[i];
        m = DATA_BUFFER_SIZE - o < n ? DATA_BUFFER_SIZE - o : n;
    } else {
        m = 0;
    }

    out = bdt_entry(DATA_ENDPOINT, 1, i);

    while (out->desc & BDT_DESC_OWN) {
        wfi();
    }

    if (m > 0) {
        memcpy(out->buffer + o, s, m);
        fill[i] += m;
    }

    if (flush || fill[i] == DATA_BUFFER_SIZE) {
        out->desc = bdt_descriptor (fill[i], i);
        out->desc |= BDT_DESC_OWN;

        oddbits ^= (1 << DATA_ENDPOINT);
        fill[i] = 0;
    }

    if (n > m) {
        return usbserial_write(s + m, n - m, flush);
    } else {
        return 0;
    }
}

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
            usbserial_write("0", 1, 0);
        }

        if (k > 0) {
            usbserial_write(p, k, 0);
        }
    }
}

static void itostr(int64_t n, int radix, int width, int plus)
{
    if (n < 0) {
        usbserial_write("-", 1, 0);
        utostr((uint64_t)(-n), radix, width);
    } else {
        if (plus == 1) {
            usbserial_write("+", 1, 0);
        } else if (plus == 1) {
            usbserial_write(" ", 1, 0);
        }

        utostr((uint64_t)n, radix, width);
    }
}

static void ftostr(float n, int width, int precision, int plus)
{
    itostr((int64_t)n, 10, width, plus);
    usbserial_write(".", 1, 0);
    utostr((int64_t)(fabs(fmodf(n, 1) * 100000000)), 10, width);
}

int usbserial_printf(const char *format, ...)
{
    va_list ap;
    const char *c, *d;

    va_start(ap, format);

    for (c = format ; *c != '\0' ;) {
        for (d = c; *d != '%' && *d != '\0' ; d += 1);

        if (d > c) {
            usbserial_write(c, d - c, 0);
        }

        c = d;

        if (*c == '%') {
            int size = 2, base = -1, sign = -1, width = -1, precision = 5;
            int plus = 0;

            for (c += 1 ; ; c += 1) {
                if (*c == '%') {
                    usbserial_write("%", 1, 0);
                    break;
                } else if (*c == '+') {
                    plus = 1;
                } else if (*c == ' ') {
                    plus = 2;
                } else if (*c >= '0' && *c <= '9' && width < 0) {
                    width = strtol(c, (char **)&c, 10);

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
                        usbserial_write(s, strlen(s), 0);
                        break;
                    } else if (*c == 'f') {
                        ftostr((float)va_arg(ap, double),
                               width, precision, plus);
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
                            utostr(va_arg(ap, int), base, width);
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

    usbserial_write(NULL, 0, 1);

    return 0;
}
