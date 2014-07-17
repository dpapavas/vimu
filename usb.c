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
static uint8_t ep0_setup[8], ep2_rx[2][DATA_BUFFER_SIZE], ep2_tx[2][DATA_BUFFER_SIZE];

static struct {
    uint8_t *data;
    uint16_t length;
} pending;

static uint8_t oddbits;
static uint8_t address, phase;
static volatile uint8_t configuration;
static volatile uint8_t line_state;
static volatile uint8_t line_coding[7];

static inline int process_setup_packet(uint16_t request, uint16_t value,
                                       uint16_t index, uint16_t length)
{
    switch (request) {
    case SETUP_DEVICE_GET_DESCRIPTOR:
        /* GET_DESCRIPTOR request. */

        switch (DESCRIPTOR_TYPE(value)) {
        case DESCRIPTOR_TYPE_DEVICE:
            /* GET_DESCRIPTOR for DEVICE. */

            pending.data = device_descriptor;
            pending.length = 18;

            return PHASE_DATA_IN;

        case DESCRIPTOR_TYPE_DEVICE_QUALIFIER:
            /* GET_DESCRIPTOR for DEVICE QUALIFIER. */

            USB0_ENDPT(0) |= USB_ENDPT_EPSTALL;

            return PHASE_COMPLETE;

        case DESCRIPTOR_TYPE_DEVICE_CONFIGURATION:
            /* GET_DESCRIPTOR for DEVICE CONFIGURATION. */

            pending.data = configuration_descriptor;
            pending.length = sizeof(configuration_descriptor);

            return PHASE_DATA_IN;
        }

        /* We've received a request for an unimplemented device
         * descriptor, so we just stall. */

        USB0_ENDPT(0) |= USB_ENDPT_EPSTALL;

        return PHASE_COMPLETE;

    case SETUP_DEVICE_GET_CONFIGURATION:
        /* GET_CONFIGURATION request. */

        pending.data = (uint8_t *)&configuration;
        pending.length = sizeof(configuration);

        return PHASE_DATA_IN;

    case SETUP_DEVICE_SET_CONFIGURATION:
        /* SET_CONFIGURATION request. */

        pending.data = NULL;
        pending.length = 0;

        if (value != 1) {
            USB0_ENDPT(0) |= USB_ENDPT_EPSTALL;

            return PHASE_COMPLETE;
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

            USB0_ENDPT(2) = (USB_ENDPT_EPRXEN | USB_ENDPT_EPTXEN |
                             USB_ENDPT_EPHSHK);

            for (i = 0 ; i < 2 ; i += 1) {
                bdtentry_t *r, *t;

                r = bdt_entry(DATA_ENDPOINT, 0, i);
                t = bdt_entry(DATA_ENDPOINT, 1, i);

                r->buffer = ep2_rx[i];
                t->buffer = ep2_tx[i];

                r->desc = bdt_descriptor (0, 0);
                t->desc = bdt_descriptor (0, 0);
            }

            configuration = (uint8_t)value;
        }

        return PHASE_DATA_OUT;

    case SETUP_DEVICE_SET_ADDRESS:
        /* SET_ADDRESS request. */

        pending.data = NULL;
        pending.length = 0;

        address = (uint8_t)value;

        return PHASE_DATA_OUT;

    case SETUP_SET_CONTROL_LINE_STATE:
        /* SET_CONTROL_LINE_STATE request. */

        pending.data = NULL;
        pending.length = 0;

        line_state = (uint8_t)value;

        return PHASE_DATA_OUT;

    case SETUP_SET_LINE_CODING:
        /* SET_LINE_CODING request. */

        pending.data = (uint8_t *)line_coding;
        pending.length = (length < sizeof(line_coding) ?
                                length : sizeof(line_coding));

        return PHASE_DATA_OUT;
    }

    /* We've received an unimplemented request, so we
     * just stall. */

    USB0_ENDPT(0) |= USB_ENDPT_EPSTALL;

    return PHASE_COMPLETE;
}

static void handle_data_transfer (uint8_t status)
{
    bdtentry_t *entry;

    entry = &(bdt[status >> 2]);

    switch (BDT_DESC_PID(entry->desc)) {
    case BDT_PID_IN:
        break;
    case BDT_PID_OUT:
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
    bdtentry_t *entry;
    uint16_t *buffer, length, value, request, index;
    int n, pid;

    /* Some notes:

      * Double-buffering of BDT entries works on the level of
        *endpoints* (tx and rx channels are considered separate
        endpoints).

      * Each time we receieve a TOKDNE interrupt a token has already
        been processed.  In order for that to happen we need to
        prepare and release a BDT entry, in the correct (odd/even)
        bank and with the correct data toggle bit (DATA0/1), for the
        next token during during each service routine call.

      * Each control transaction consists of up to 3 stages: SETUP, DATA
        and STATUS.

      * Each stage has a data packet.

      * The SETUP data packet is always DATA0 and the request determines
        if the DATA stage exists and what direction it has.

      * The DATA stage data packets alternate between DATA0 and DATA1
        and must transfer the amount of data specified in the SETUP
        phase.

      * The STATUS phase data packet is always of the opposite direction
        with respect to the preceding stage and always DATA1. */

    entry = &(bdt[status >> 2]);
    pid = BDT_DESC_PID(entry->desc);

    if (phase == PHASE_SETUP && pid == BDT_PID_SETUP) {
        /* Read the payload and release the buffer. */

        buffer = (uint16_t *)entry->buffer;
        request = buffer[0];
        value = buffer[1];
        index = buffer[2];
        length = buffer[3];

        phase = process_setup_packet(request, value, index, length);

        /* Make sure we don't return more than the host asked
         * for. */

        if (phase == PHASE_DATA_IN && length < pending.length) {
            pending.length = length;
        }
    }

    if (phase == PHASE_DATA_IN &&
        (pid == BDT_PID_IN || pid == BDT_PID_SETUP)) {

        /* Determine whether there's anything else to send
         * and prepare the buffer, otherwise transition to
         * the status phase. */

        n = pending.length < CONTROL_BUFFER_SIZE ?
            pending.length : CONTROL_BUFFER_SIZE;

        if (n > 0) {
            bdtentry_t *out;
            int data1;

            data1 = !(entry->desc & BDT_DESC_DATA1);
            out = bdt_entry(CONTROL_ENDPOINT, 1,
                            oddbit(CONTROL_ENDPOINT, 1));
            assert (!(out->desc & BDT_DESC_OWN));

            out->buffer = (void *)pending.data;
            out->desc = bdt_descriptor (n, data1);
            out->desc |= BDT_DESC_OWN;

            pending.data += n;
            pending.length -= n;

            toggle_oddbit (CONTROL_ENDPOINT, 1);
        } else {
            bdtentry_t *in;

            /* Prepare to receive a zero-length DATA1 packet to
             * aknowledge proper reception during the preceding
             * IN phase. */

            in = bdt_entry(CONTROL_ENDPOINT, 0,
                           oddbit(CONTROL_ENDPOINT, 0));
            assert (!(in->desc & BDT_DESC_OWN));

            in->buffer = NULL;
            in->desc = bdt_descriptor (0, 1);
            in->desc |= BDT_DESC_OWN;

            toggle_oddbit (CONTROL_ENDPOINT, 0);
            phase = PHASE_STATUS_IN;
        }
    }

    if (phase == PHASE_DATA_OUT &&
        (pid == BDT_PID_OUT || pid == BDT_PID_SETUP)) {

        /* Determine whether there's anything else to receive and
         * prepare the buffer, otherwise transition to the status
         * phase. */

        n = pending.length < CONTROL_BUFFER_SIZE ?
            pending.length : CONTROL_BUFFER_SIZE;

        if (n > 0) {
            bdtentry_t *in;
            int data1;

            data1 = !(entry->desc & BDT_DESC_DATA1);
            in = bdt_entry(CONTROL_ENDPOINT, 0, oddbit(CONTROL_ENDPOINT, 0));
            assert (!(in->desc & BDT_DESC_OWN));

            in->buffer = (void *)pending.data;
            in->desc = bdt_descriptor (n, data1);
            in->desc |= BDT_DESC_OWN;

            pending.data += n;
            pending.length -= n;

            toggle_oddbit (CONTROL_ENDPOINT, 0);
        } else {
            bdtentry_t *out;

            /* Prepare to send a zero-length DATA1 packet to
             * aknowledge proper reception during the preceding
             * OUT phase. */

            out = bdt_entry(CONTROL_ENDPOINT, 1, oddbit(CONTROL_ENDPOINT, 1));
            assert (!(out->desc & BDT_DESC_OWN));

            out->buffer = NULL;
            out->desc = bdt_descriptor (0, 1);
            out->desc |= BDT_DESC_OWN;

            toggle_oddbit (CONTROL_ENDPOINT, 1);
            phase = PHASE_STATUS_OUT;
        }
    }

    if (pid == BDT_PID_SETUP) {
        USB0_CTL &= ~USB_CTL_TXSUSPENDTOKENBUSY;
    }

    if ((phase == PHASE_STATUS_IN && pid == BDT_PID_OUT) ||
        (phase == PHASE_STATUS_OUT && pid == BDT_PID_IN)) {

        /* Set the address. */

        if (USB0_ADDR != address) {
            assert(phase == PHASE_STATUS_OUT && pid == BDT_PID_IN);
            USB0_ADDR = address;
        }

        phase = PHASE_COMPLETE;
    }

    if (phase == PHASE_COMPLETE) {
        bdtentry_t *in;

        /* Prepare an input buffer for the next setup token. */

        in = bdt_entry(CONTROL_ENDPOINT, 0, oddbit(CONTROL_ENDPOINT, 0));
        assert (!(in->desc & BDT_DESC_OWN));

        in->buffer = (void *)ep0_setup;
        in->desc = bdt_descriptor (8, 0);
        in->desc |= BDT_DESC_OWN;

        toggle_oddbit (CONTROL_ENDPOINT, 0);

        phase = PHASE_SETUP;
    }
}

__attribute__((interrupt ("IRQ"))) void usb_isr()
{
    if (USB0_ISTAT & USB_ISTAT_USBRST) {
        int i;

        /* Reset the even/odd BDT toggle bits. */

        USB0_CTL |= USB_CTL_ODDRST;
        phase = PHASE_SETUP;
        oddbits = 0;
        address = 0;

        /* Initialize the BDT entries for endpoint 0. */

        for (i = 0 ; i < 2 ; i += 1) {
            bdtentry_t *r, *t;

            r = bdt_entry(CONTROL_ENDPOINT, 0, i);
            t = bdt_entry(CONTROL_ENDPOINT, 1, i);

            if (i == 0) {
                /* Prepare a buffer for the first setup packet (setup
                 * packets are always DATA0). */

                r->buffer = ep0_setup;
                r->desc = bdt_descriptor (8, 0);
                r->desc |= BDT_DESC_OWN;

                toggle_oddbit(CONTROL_ENDPOINT, 0);
            } else {
                r->buffer = NULL;
                r->desc = bdt_descriptor (0, 0);
            }

            t->buffer = NULL;
            t->desc = bdt_descriptor (0, 0);
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
        assert(0);
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
    prioritize_interrupt(35, 3);

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

    /* { */
    /*     usbserial_await_rts(); */
    /*     usbserial_trace("%x %x %x %x %x %x %x\n", */
    /*                     line_coding[0], line_coding[1], line_coding[2], */
    /*                     line_coding[3], line_coding[4], line_coding[5], */
    /*                     line_coding[6]); */
    /* } */
}

int usb_enumerated()
{
    return configuration;
}

void usb_await_enumeration()
{
    sleep_while(configuration == 0);
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

int usbserial_read(char *buffer)
{
    volatile bdtentry_t *in;

    if (configuration == 0) {
        return -1;
    }

    in = bdt_entry(DATA_ENDPOINT, 0, oddbit(DATA_ENDPOINT, 0));
    assert (!(in->desc & BDT_DESC_OWN));

    in->desc = bdt_descriptor (DATA_BUFFER_SIZE, oddbit(DATA_ENDPOINT, 0));
    in->desc |= BDT_DESC_OWN;

    toggle_oddbit(DATA_ENDPOINT, 0);

    sleep_while (in->desc & BDT_DESC_OWN);

    assert (BDT_DESC_BYTE_COUNT(in->desc) > 0);

    usbserial_write(in->buffer, BDT_DESC_BYTE_COUNT(in->desc), 1);

    return 0;
}

int usbserial_write(const char *s, int n, int flush)
{
    static uint8_t fill[2];
    volatile bdtentry_t *out;
    int i, o, m;

    if (configuration == 0 || !(line_state & LINE_STATE_DTR)) {
        return -1;
    }

    i = oddbit (DATA_ENDPOINT, 1);

    if (n > 0) {
        o = fill[i];
        m = DATA_BUFFER_SIZE - o < n ? DATA_BUFFER_SIZE - o : n;
    } else {
        m = 0;
    }

    out = bdt_entry(DATA_ENDPOINT, 1, i);
    sleep_while (out->desc & BDT_DESC_OWN);

    if (m > 0) {
        memcpy(out->buffer + o, s, m);
        fill[i] += m;
    }

    if (flush || fill[i] == DATA_BUFFER_SIZE) {
        out->desc = bdt_descriptor (fill[i], i);
        out->desc |= BDT_DESC_OWN;

        toggle_oddbit (DATA_ENDPOINT, 1);
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

    usbserial_write(NULL, 0, 1);

    return 0;
}
