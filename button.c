#include "mk20dx128.h"
#include "usbserial.h"
#include "button.h"

static uint8_t down;
static button_callback callback;

__attribute__((interrupt ("IRQ"))) void lptmr_isr(void)
{
    if (!down) {
        down = 1;

        /* Set the lop power timer for active-low to catch the button
         * release. */

        LPTMR0_CSR &= ~LPTMR_CSR_TEN;
        LPTMR0_CSR |= LPTMR_CSR_TPP;
        LPTMR0_CSR |= LPTMR_CSR_TEN;
    } else {
        down = 0;

        if (callback) {
            callback();
        }

        /* Set the lop power timer for active-low to catch the next
         * button press. */

        LPTMR0_CSR &= ~LPTMR_CSR_TEN;
        LPTMR0_CSR &= ~LPTMR_CSR_TPP;
        LPTMR0_CSR |= LPTMR_CSR_TEN;
    }

    LPTMR0_CSR |= LPTMR_CSR_TCF;
}

void button_set_callback(button_callback new_callback)
{
    callback = new_callback;
}

void button_initialize()
{
    /* Enable the low power timer and port C modules. */

    SIM_SCGC5 |= SIM_SCGC5_PORTC | SIM_SCGC5_LPTIMER;
    PORTC_PCR5 = PORT_PCR_MUX(3) | PORT_PCR_PFE | PORT_PCR_PE;

    /* Reset the low power timer module. */

    LPTMR0_CSR = 0;

    /* Configure for single pulse counting, clocked by the 1kHz LPO
     * clock and set the glitch filter for 16 rising clock edges. */

    LPTMR0_CSR = LPTMR_CSR_TPS(2) | LPTMR_CSR_TMS | LPTMR_CSR_TIE;
    LPTMR0_CMR = 0;
    LPTMR0_PSR = LPTMR_PSR_PCS(1) | LPTMR_PSR_PRESCALE(4);

    /* Enable the timer and the corresponding interrupt. */

    LPTMR0_CSR |= LPTMR_CSR_TEN;

    /* Enable the button interrupt and set its priority as low as it
     * gets. */

    enable_interrupt(39);
    prioritize_interrupt(39, 15)
}
