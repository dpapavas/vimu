#include <stdio.h>
#include <string.h>

#include "mk20dx128.h"
#include "util.h"
#include "usb.h"
#include "usbserial.h"

__attribute__ ((section(".usbdata.bdt")))
static bdtentry_t bdt[ENDPOINTS_N * 4];

static struct {
    __attribute__ ((aligned(4)))
    uint8_t setup[8];

    __attribute__ ((aligned(4)))
    uint8_t notification[2][NOTIFICATION_BUFFER_SIZE];

    __attribute__ ((aligned(4)))
    uint8_t serial[4][DATA_BUFFER_SIZE];
}  buffers;

static struct {
    uint8_t *data;
    uint16_t length;
} pending, buffered[2];

static uint8_t oddbits;
static uint8_t address, phase;
static volatile uint8_t configuration;

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

            USB0_ENDPT(CONTROL_ENDPOINT) |= USB_ENDPT_EPSTALL;

            return PHASE_COMPLETE;

        case DESCRIPTOR_TYPE_DEVICE_CONFIGURATION:
            /* GET_DESCRIPTOR for DEVICE CONFIGURATION. */

            pending.data = configuration_descriptor;
            pending.length = sizeof(configuration_descriptor);

            return PHASE_DATA_IN;
        }

        /* We've received a request for an unimplemented device
         * descriptor, so we just stall. */

        USB0_ENDPT(CONTROL_ENDPOINT) |= USB_ENDPT_EPSTALL;

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
            USB0_ENDPT(CONTROL_ENDPOINT) |= USB_ENDPT_EPSTALL;

            return PHASE_COMPLETE;
        } else {
            int i;

            /* Initialize endpoint 1. */

            USB0_ENDPT(NOTIFICATION_ENDPOINT) = (USB_ENDPT_EPTXEN |
                                                 USB_ENDPT_EPHSHK);

            for (i = 0 ; i < 2 ; i += 1) {
                bdtentry_t *t;

                t = bdt_entry(NOTIFICATION_ENDPOINT, 1, i);
                t->buffer = buffers.notification[i];
                t->desc = bdt_descriptor (0, 0);
            }

            reset_oddbit(NOTIFICATION_ENDPOINT, 0);
            reset_oddbit(NOTIFICATION_ENDPOINT, 1);

            /* Initialize endpoint 2. */

            USB0_ENDPT(DATA_ENDPOINT) = (USB_ENDPT_EPRXEN | USB_ENDPT_EPTXEN |
                                         USB_ENDPT_EPHSHK);

            for (i = 0 ; i < 2 ; i += 1) {
                bdtentry_t *r, *t;

                r = bdt_entry(DATA_ENDPOINT, 0, i);
                t = bdt_entry(DATA_ENDPOINT, 1, i);

                r->buffer = buffers.serial[i];
                t->buffer = buffers.serial[2 + i];

                /* Prime the first input BDT entry. In the case of
                 * data endpont input transmissions we use the oddbits
                 * to keep track of the BDT entry we should read next,
                 * so don't toggle here. */

                r->desc = bdt_descriptor (DATA_BUFFER_SIZE, i);

                if (i == 0) {
                    r->desc |= BDT_DESC_OWN;
                }

                t->desc = bdt_descriptor (0, 0);
            }

            /* Initialize the buffer pointers. */

            reset_oddbit(DATA_ENDPOINT, 0);
            reset_oddbit(DATA_ENDPOINT, 1);

            buffered[1].data = NULL;
            buffered[1].length = DATA_BUFFER_SIZE;

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

    USB0_ENDPT(CONTROL_ENDPOINT) |= USB_ENDPT_EPSTALL;

    return PHASE_COMPLETE;
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

        in->buffer = (void *)buffers.setup;
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
        address = 0;

        pending.data = NULL;
        pending.length = 0;

        reset_oddbit(CONTROL_ENDPOINT, 0);
        reset_oddbit(CONTROL_ENDPOINT, 1);

        /* Initialize the BDT entries for endpoint 0. */

        for (i = 0 ; i < 2 ; i += 1) {
            bdtentry_t *r, *t;

            r = bdt_entry(CONTROL_ENDPOINT, 0, i);
            t = bdt_entry(CONTROL_ENDPOINT, 1, i);

            if (i == 0) {
                /* Prepare a buffer for the first setup packet (setup
                 * packets are always DATA0). */

                r->buffer = buffers.setup;
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

        USB0_ENDPT(CONTROL_ENDPOINT) = (USB_ENDPT_EPRXEN | USB_ENDPT_EPTXEN |
                                        USB_ENDPT_EPHSHK);
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
        case CONTROL_ENDPOINT: handle_control_transfer(status); break;
        case NOTIFICATION_ENDPOINT: case DATA_ENDPOINT: break;
        default:
            USB0_ENDPT(n) |= USB_ENDPT_EPSTALL;
        }

        USB0_ISTAT |= USB_ISTAT_TOKDNE;
    }
}

void usb_initialize()
{
    /* Enable the USB interrupt but give it a low priority. */

    enable_interrupt(35);
    prioritize_interrupt(35, 14);

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

int usb_interrupt(uint8_t *buffer, int n)
{
    bdtentry_t *out;

    out = bdt_entry(NOTIFICATION_ENDPOINT, 1,
                    oddbit(NOTIFICATION_ENDPOINT, 1));
    sleep_while (out->desc & BDT_DESC_OWN);

    memcpy(out->buffer, buffer, n);

    out->desc = bdt_descriptor (n, oddbit(NOTIFICATION_ENDPOINT, 1));
    out->desc |= BDT_DESC_OWN;

    toggle_oddbit (NOTIFICATION_ENDPOINT, 1);

    return 0;
}

int usb_read(char **buffer, int *length)
{
    volatile bdtentry_t *in;

    if (configuration == 0) {
        return 0;
    }

    /* Fetch more data if needed. */

    in = bdt_entry(DATA_ENDPOINT, 0, oddbit(DATA_ENDPOINT, 0));
    if (in->desc & BDT_DESC_OWN) {
        return 0;
    }

    *buffer = in->buffer;
    *length = BDT_DESC_BYTE_COUNT(in->desc);

    /* Release the other BDT entry. */

    toggle_oddbit(DATA_ENDPOINT, 0);

    in = bdt_entry(DATA_ENDPOINT, 0, oddbit(DATA_ENDPOINT, 0));
    assert(!(in->desc & BDT_DESC_OWN));

    in->desc = bdt_descriptor (DATA_BUFFER_SIZE, oddbit(DATA_ENDPOINT, 0));
    in->desc |= BDT_DESC_OWN;

    return 1;
}

int usb_write(const char *s, int n, int flush)
{
    volatile bdtentry_t *out;
    int m;

    if (configuration == 0) {
        return -1;
    }

    /* If the current buffer has been filled and flushed to the USB
     * module fetch a new one. */

    if (buffered[1].length == DATA_BUFFER_SIZE) {
        out = bdt_entry(DATA_ENDPOINT, 1, oddbit (DATA_ENDPOINT, 1));

        buffered[1].data = out->buffer;
        buffered[1].length = 0;

        sleep_while (out->desc & BDT_DESC_OWN);
    }

    /* Break the write up if it's too large to fit in the current
     * buffer. */

    if (n > 0) {
        m = DATA_BUFFER_SIZE - buffered[1].length < n ?
            DATA_BUFFER_SIZE - buffered[1].length : n;
    } else {
        m = 0;
    }

    assert(m > 0 || n == 0);

    if (m > 0) {
        memcpy(buffered[1].data, s, m);

        buffered[1].data += m;
        buffered[1].length += m;
    }

    /* Send the current buffer to the USB module. */

    if (flush || buffered[1].length == DATA_BUFFER_SIZE) {
        out = bdt_entry(DATA_ENDPOINT, 1, oddbit (DATA_ENDPOINT, 1));
        assert(!(out->desc & BDT_DESC_OWN));

        out->desc = bdt_descriptor (buffered[1].length,
                                    oddbit (DATA_ENDPOINT, 1));
        out->desc |= BDT_DESC_OWN;

        toggle_oddbit (DATA_ENDPOINT, 1);
    }

    if (flush) {
        buffered[1].length = DATA_BUFFER_SIZE;
    }

    if (n > m) {
        return usb_write(s + m, n - m, flush);
    } else {
        return 0;
    }
}
