#define _XOPEN_SOURCE

#include <string.h>

#include "mk20dx128.h"
#include "usbcommon.h"
#include "usbserial.h"
#include "sensors.h"
#include "fusion.h"
#include "console.h"
#include "button.h"
#include "sdio.h"
#include "log.h"
#include "util.h"
#include "i2c.h"

static void fusion_data_ready(float *line)
{
    const int N = 1000;
    const float c = 1e-3;
    static int n;
    static float sumy, sumxy, sumw[3];
    float y, *w;
    int i;

    y = line[9];
    w = line + 3;

    for (i = 0 ; i < 3 ; i += 1) {
        sumw[i] += w[i];
    }

    n += 1;
    sumy += y;
    sumxy += n * y;

    /* usbserial_printf("%f\n", y); */

    if (n == N) {
        float a, lambda;

        a = (2.0 / (float)N / (float)(N - 1)) *
            ((2 * N + 1) * sumy - 3.0 * sumxy);
        lambda = (2.0 / (float)N / (float)(N - 1) / c) *
            (-3.0 * sumy + 6.0 / (float)(N + 1) * sumxy);

        usbserial_printf("%f, %f, %f, %f, %f\n",
                         sumw[0] / N, sumw[1] / N, sumw[2] / N, a, lambda);

        sumxy = sumy = sumw[0] = sumw[1] = sumw[2] = 0;
        n = 0;
    }
}

static void foo()
{
    /* int i; */
    /* const char c[] = {0xa0, 0xa1, 0x00, 0x02, 0x02, 0x01, */
    /*                   0x02 ^ 0x01, 0x0D, 0x0A}; */

    /* for (i = 0 ; i < sizeof(c) ; i += 1) { */
    /*     while(!(UART0_S1 & UART_S1_TDRE)); */

    /*     UART0_D = (uint8_t)c[i]; */
    /* } */

    /* usb_write("\n\n", 2, 1); */

    /* GPIOD_PTOR = (1ul << 3); */
    usbserial_printf("click!\n");
}

int main()
{
    usb_initialize();
    usbserial_await_rts();

    sdio_initialize();
    i2c_initialize();
    button_initialize();
    button_set_callback(foo);
    console_initialize();

    console_enter();

#if 0
    /* button_set_callback(callback); */

    /* while(1) { */
    /*      if (usbserial_is_rts()) { */
    /*         console_cycle(); */
    /*     } */
    /* } */

    usbserial_await_rts();
    button_set_callback(foo);
    assert(0);

    SIM_SCGC4 |= SIM_SCGC4_UART0;
    SIM_SCGC5 |= SIM_SCGC5_PORTB;
    PORTB_PCR16 = PORT_PCR_MUX(3) | PORT_PCR_PFE;
    PORTB_PCR17 = PORT_PCR_MUX(3) | PORT_PCR_PFE;

    set_uart_baud_rate(0, 9600);

    UART0_C2 &= ~(UART_C2_RE | UART_C2_TE);
    /* UART0_PFIFO = UART_PFIFO_RXFE; */
    /* UART0_CFIFO = UART_CFIFO_RXOFE; */
    UART0_RWFIFO = 1;
    /* UART0_C3 = UART_C3_ORIE | UART_C3_NEIE | UART_C3_FEIE; */
    UART0_C2 = /* UART_C2_RIE |  */UART_C2_RE | UART_C2_TE;

    while(1) {
        if (UART0_S1 & UART_S1_RDRF) {
            volatile uint8_t d = UART0_D;
            usb_write((char *)&d, 1, 1);
            /* usbserial_printf("%2x ", d); */
            toggle_led();
        }

        /* usbserial_printf("%b\n", UART0_S1); */
    }
#endif

    return 0;
}
