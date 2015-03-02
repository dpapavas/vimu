#include "mk20dx128.h"
#include "sdio.h"
#include "usbserial.h"
#include "util.h"

/* #define DEBUG */
#define MAX_RETRIES 5

#define INITIAL 0
#define TRANSMIT_ARGUMENT 1
#define TRANSMIT_CRC7 5
#define AWAIT_RESPONSE 6
#define RECEIVE_PAYLOAD 7
#define RECEIVE_DATA_RESPONSE_TOKEN 12
#define WAIT_WHILE_BUSY 13
#define STOP_TRANSMISSION 14
#define HANDLE_DATA_BLOCK 15

#define RESET_PIT(timeout)                              \
    PIT_TFLG(1) |= PIT_TFLG_TIF;                        \
    PIT_TCTRL(1) = 0;                                   \
    PIT_LDVAL(1) = (F_BUS / 1000000) * timeout - 1;     \
    PIT_TCTRL(1) |= PIT_TCTRL_TEN;

#define DISABLE_PIT()                                   \
    PIT_TFLG(1) |= PIT_TFLG_TIF;                        \
    PIT_TCTRL(1) = 0;

#define PIT_TIMED_OUT() (PIT_TFLG(1) & PIT_TFLG_TIF)

#define SPI_PUSHR_HEADER (SPI_PUSHR_PCS(0) | SPI_PUSHR_CTAS(0))

static int8_t ccs;

static struct {
    sdio_TransactionCallback callback;
    void *userdata;
    uint32_t argument;
    uint8_t command, response, *buffer;
    int phase, tries;
    volatile int busy;
} context;

static void reset_command()
{
    /* Clear the FIFOs. */

    SPI0_MCR |= (SPI_MCR_CLR_RXF | SPI_MCR_CLR_TXF |
                 SPI_MCR_DIS_RXF | SPI_MCR_DIS_TXF);

    /* Enable the interrupt. */

    SPI0_SR |= SPI_SR_TCF;
    SPI0_RSER = SPI_RSER_TCF_RE;

    /* Set up the command. */

    context.phase = 0;
    context.tries += 1;
    assert(context.tries <= MAX_RETRIES);

    pend_interrupt(12);
    DISABLE_PIT();
}

static void finalize_command()
{
    /* Disable the interrupt. */

    SPI0_RSER = 0;
    context.busy = 0;
    /* turn_off_led(); */
}

static void initialize_command(uint8_t cmd, uint32_t arg,
                               uint8_t *buffer,
                               sdio_TransactionCallback callback,
                               void *userdata)
{
    /* Set up the command. */

    context.command = cmd;
    context.argument = arg;
    context.buffer = buffer;
    context.callback = callback;
    context.userdata = userdata;
    context.tries = 0;
    context.busy = 1;
    /* turn_on_led(); */

    reset_command();
}

__attribute__((interrupt ("IRQ"))) void spi0_isr(void)
{
    uint8_t i;
    int j;

    SPI0_SR |= SPI_SR_TCF;

    switch(context.phase) {
    case INITIAL:
        /* Set up the CRC module to calculate the CRC7
         * checksum. */

        CRC_GPOLY = 0x0900 << 1;
        CRC_CTRL = CRC_CTRL_WAS;
        CRC_CRC16 = 0;
        CRC_CTRL = 0;

        /* Push the command code. */

        SPI0_PUSHR = context.command | 0x40 | SPI_PUSHR_HEADER;
        CRC_CRC8 = context.command | 0x40;
        context.phase = TRANSMIT_ARGUMENT;

        break;
    case TRANSMIT_ARGUMENT ... TRANSMIT_ARGUMENT + 3:
        j = context.phase - TRANSMIT_ARGUMENT;

        /* Transmit the argument. */

        SPI0_PUSHR = ((uint8_t *)&context.argument)[3 - j] | SPI_PUSHR_HEADER;
        CRC_CRC8 = ((uint8_t *)&context.argument)[3 - j];

        if (j < 3) {
            context.phase += 1;
        } else {
            context.phase = TRANSMIT_CRC7;
        }

        break;
    case TRANSMIT_CRC7:
        /* Transmit the CRC7 checksum. */

        SPI0_PUSHR = ((CRC_CRC16 >> 8) | 1) | SPI_PUSHR_HEADER;
        context.phase = AWAIT_RESPONSE;

        /* Set a 10 ms timeout for the card response.  (64 cycles are
         * specified but in we're doing byte-by-byte, interrupt based
         * tranfer, so we cycle irregularly due to software
         * latency). */

        RESET_PIT(10000);

        break;
    case AWAIT_RESPONSE:
        if ((i = SPI0_POPR) != 0xff) {
            /* usbserial_trace("resp = %x\n", i); */

            context.response = i;

            if (context.command == 17 ||
                context.command == 18 ||
                context.command == 24 ||
                context.command == 25) {
                /* Keep the timer running for read commands as the
                 * timeout is measured from the last bit of the
                 * request till the data block start token. */

                if (context.command == 17 ||
                    context.command == 18) {
                    /* Set a timeout of 100ms for the start of the
                     * block. */

                    RESET_PIT(100000);
                } else {
                    DISABLE_PIT();
                }

                if (i != 0x0) {
                    reset_command();
                    break;
                } else {
                    /* Set up the CRC module to calculate the CRC16
                     * checksum. */

                    CRC_GPOLY = 0x1021;
                    CRC_CTRL = CRC_CTRL_WAS;
                    CRC_CRC16 = 0;
                    CRC_CTRL = 0;

                    SPI0_MCR &= ~(context.command == 17 ||
                                  context.command == 18 ?
                                  SPI_MCR_DIS_TXF : SPI_MCR_DIS_RXF);

                    context.phase = HANDLE_DATA_BLOCK;
                }
            } else if (context.command == 0 ||
                       context.command == 16 ||
                       context.command == 41 ||
                       context.command == 55 ||
                       context.command == 59) {
                DISABLE_PIT();

                /* Nothing more to do for these commands. */

                if (i != 0x0 && i != 0x1) {
                    reset_command();
                } else {
                    finalize_command();
                }

                break;
            } else if (context.command == 8 ||
                       context.command == 13 ||
                       context.command == 58) {
                DISABLE_PIT();

                /* These commands have 4 more bytes of payload. */

                if (i != 0x0 && i != 0x1) {
                    reset_command();
                    break;
                } else {
                    context.phase = RECEIVE_PAYLOAD;
                }
            } else if (context.command == 12) {
                DISABLE_PIT();

                context.phase = STOP_TRANSMISSION;
            }
        } else if (PIT_TIMED_OUT()) {
            reset_command();
            break;
        }

        SPI0_PUSHR = 0xff | SPI_PUSHR_HEADER;

        break;
    case RECEIVE_PAYLOAD ... RECEIVE_PAYLOAD + 3:
        j = context.phase - RECEIVE_PAYLOAD;

        /* We've already received the R1 header, now receive the rest. */

        if (context.command == 8) {
            /* Receive an R7 response and check its validity. */

            if (((uint8_t *)&context.argument)[3 - j] != SPI0_POPR) {
                reset_command();
                break;
            }

            if (j == 3) {
                finalize_command();
                break;
            } else {
                context.phase += 1;
            }
        } else if (context.command == 58) {
            /* Receive an R3 response. */

            if (j == 0) {
                ccs = (SPI0_POPR & (1 << 6)) >> 6;
            }

            if (j == 3) {
                finalize_command();
                break;
            } else {
                context.phase += 1;
            }
        } else if (context.command == 13) {
            /* Receive an R2 response. */

            *context.buffer = SPI0_POPR;

            finalize_command();
            break;
        }

        SPI0_PUSHR = 0xff | SPI_PUSHR_HEADER;

        break;
    case HANDLE_DATA_BLOCK:
        if(context.command == 24 ||
           context.command == 25) {

            if (context.command == 25 && !context.buffer) {
                assert(context.callback);
                context.buffer = context.callback(SDIO_IN_PROGRESS,
                                                  context.buffer,
                                                  context.userdata);
                assert(context.buffer);
            }

            /* Send the start block token. */

            SPI0_PUSHR = ((context.command == 24 ? 0xfe : 0xfc) |
                          SPI_PUSHR_HEADER);

            /* Start listening to the TFFF interrupt. */

            SPI0_SR |= SPI_SR_TFFF;
            SPI0_RSER = SPI_RSER_TFFF_RE;

            context.phase += 1;
        } else {
            assert(context.command == 17 || context.command == 18);

            if (context.command == 18 && !context.buffer) {
                assert(context.callback);
                context.buffer = context.callback(SDIO_IN_PROGRESS,
                                                  context.buffer,
                                                  context.userdata);
                assert(context.buffer);
            }

            /* Wait for the start block token. */

            if ((i = SPI0_POPR) != 0xff) {
                DISABLE_PIT();

                /* usbserial_trace("token = %x\n", i); */

                if(i == 0xfe) {
                    /* Start listening to the RFDF interrupt. */

                    SPI0_SR |= SPI_SR_RFDF;
                    SPI0_RSER = SPI_RSER_RFDF_RE;

                    context.phase += 1;
                } else {
                    if (context.command == 17) {
                        reset_command();
                    } else {
                        assert(context.callback);
                        context.callback(SDIO_FAILED,
                                         context.buffer,
                                         context.userdata);
                    }

                    break;
                }
            } else if (PIT_TIMED_OUT()) {
                if (context.command == 17) {
                    reset_command();
                } else {
                    assert(context.callback);
                    context.callback(SDIO_FAILED,
                                     context.buffer,
                                     context.userdata);
                }

                break;
            }

            SPI0_PUSHR = 0xff | SPI_PUSHR_HEADER;
        }

        break;
    case HANDLE_DATA_BLOCK + 1 ... HANDLE_DATA_BLOCK + 512:
        if (context.command == 24 ||
            context.command == 25) {
            /* Transmit the data in the buffer. */

            while ((SPI0_SR & SPI_SR_TFFF) &&
                   (j = context.phase - HANDLE_DATA_BLOCK - 1) < 512) {
                SPI0_PUSHR = context.buffer[j] | SPI_PUSHR_HEADER;
                CRC_CRC8 = context.buffer[j];

                context.phase += 1;
            }
        } else {
            assert(context.command == 17 || context.command == 18);

            /* Read the data into the buffer. */

            while ((SPI0_SR & SPI_SR_RFDF) &&
                   (j = context.phase - HANDLE_DATA_BLOCK - 1) < 512) {
                SPI0_SR |= SPI_SR_RFDF;
                CRC_CRC8 = context.buffer[j] = SPI0_POPR;
                context.phase += 1;

                assert(SPI0_SR & SPI_SR_TFFF);
                SPI0_PUSHR = 0xff | SPI_PUSHR_HEADER;
            }
        }

        break;
    case HANDLE_DATA_BLOCK + 513 ... HANDLE_DATA_BLOCK + 514:
        j = context.phase - HANDLE_DATA_BLOCK - 513;

        if (context.command == 24 ||
            context.command == 25) {
            /* Transmit the checksum. */

            SPI0_PUSHR = ((CRC_CRC16 >> (!j << 3)) & 0xff) | SPI_PUSHR_HEADER;

            if (j < 1) {
                context.phase += 1;
            } else {
                context.phase = RECEIVE_DATA_RESPONSE_TOKEN;

                /* Set a busy timeout of 1 s until end of card busy
                 * (250ms is specified). */

                RESET_PIT(1000000);
            }
        } else {
            assert(context.command == 17 || context.command == 18);

            /* Receive the checksum and check it against the one we
             * calculated. */

            if(((CRC_CRC16 >> (!j << 3)) & 0xff) != SPI0_POPR) {
                if (context.command == 17) {
                    reset_command();
                } else {
                    assert(context.callback);
                    context.callback(SDIO_FAILED,
                                     context.buffer,
                                     context.userdata);
                }

                break;
            } else {
                if (j < 1) {
                    context.phase += 1;
                } else {
                    if (context.command == 17) {
                        finalize_command();
                        break;
                    } else {
                        assert(context.callback);
                        context.buffer = context.callback(SDIO_IN_PROGRESS,
                                                          context.buffer,
                                                          context.userdata);

                        if (!context.buffer) {
                            /* Send the stop transmission command. */

                            initialize_command(12, 0, NULL, NULL, NULL);
                            break;
                        }

                        /* Prepare for the next block. */

                        CRC_GPOLY = 0x1021;
                        CRC_CTRL = CRC_CTRL_WAS;
                        CRC_CRC16 = 0;
                        CRC_CTRL = 0;

                        context.phase = HANDLE_DATA_BLOCK;
                    }
                }
            }

            SPI0_PUSHR = 0xff | SPI_PUSHR_HEADER;
        }

        break;
    case RECEIVE_DATA_RESPONSE_TOKEN:
        if (context.command == 24 ||
            context.command == 25) {
            /* Wait for the data response token. */

            if ((i = SPI0_POPR) != 0xff) {
                /* usbserial_trace("token = %x\n", i); */

                if((i & 0xf) == 0x5) {
                    context.phase = WAIT_WHILE_BUSY;
                } else {
                    if (context.command == 24) {
                        reset_command();
                    } else {
                        assert(context.callback);
                        context.callback(SDIO_FAILED,
                                         context.buffer,
                                         context.userdata);
                    }
                    break;
                }
            } else if (PIT_TIMED_OUT()) {
                if (context.command == 24) {
                    reset_command();
                } else {
                    assert(context.callback);
                    context.callback(SDIO_FAILED,
                                     context.buffer,
                                     context.userdata);
                }
                break;
            }

            SPI0_PUSHR = 0xff | SPI_PUSHR_HEADER;
        }

        break;
    case WAIT_WHILE_BUSY:
        if (context.command == 24 ||
            context.command == 25 ||
            context.command == 12) {

            if (SPI0_POPR != 0x0) {
                DISABLE_PIT();

                if (context.command == 12 ||
                    context.command == 24 ||
                    (context.command == 25 && context.buffer == NULL)) {
                    finalize_command();

                    break;
                } else {
                    assert(context.callback);
                    context.buffer = context.callback(SDIO_IN_PROGRESS,
                                                      context.buffer,
                                                      context.userdata);

                    if (context.buffer) {
                        /* Prepare for the next block. */

                        CRC_GPOLY = 0x1021;
                        CRC_CTRL = CRC_CTRL_WAS;
                        CRC_CRC16 = 0;
                        CRC_CTRL = 0;

                        context.phase = HANDLE_DATA_BLOCK;
                    } else {
                        SPI0_PUSHR = 0xfd | SPI_PUSHR_HEADER;

                        context.phase = STOP_TRANSMISSION;
                        break;
                    }
                }
            } else if (PIT_TIMED_OUT()) {
                if (context.command == 24) {
                    reset_command();
                } else {
                    assert(context.callback);
                    context.callback(SDIO_FAILED,
                                     context.buffer,
                                     context.userdata);
                }
                break;
            }

            SPI0_PUSHR = 0xff | SPI_PUSHR_HEADER;
        }

        break;

    case STOP_TRANSMISSION:
        /* Wait for card to get busy. */

        if (SPI0_POPR == 0x0) {
            context.phase = WAIT_WHILE_BUSY;

            /* Set a busy timeout of 1 s until end of card busy
             * (250ms is specified). */

            RESET_PIT(1000000);
        }

        SPI0_PUSHR = 0xff | SPI_PUSHR_HEADER;

        break;
    }
}

static uint8_t send_command(uint8_t cmd, uint32_t arg)
{
#ifdef DEBUG
    usbserial_trace("cmd = %d, %x\n", cmd, arg);
#endif

    sleep_while (context.busy);
    initialize_command(cmd, arg, NULL, NULL, NULL);

    /* Wait for command to finish. */

    sleep_while (context.busy);

#ifdef DEBUG
    usbserial_trace("resp = %x\n", context.response);
#endif

    return context.response;
}

void sdio_initialize()
{
    int i;

    enable_interrupt(12);
    prioritize_interrupt(12, 5);

    /* Enable the CRC module. */

    SIM_SCGC6 |= SIM_SCGC6_CRC;

    /* Enable the clock to the SPI module and the associated ports. */

    SIM_SCGC5 |= SIM_SCGC5_PORTD | SIM_SCGC5_PORTC;
    SIM_SCGC6 |= SIM_SCGC6_SPI0;

    /* Configure the pins needed by the SPI module. */

    PORTC_PCR4 = PORT_PCR_MUX(2);
    PORTC_PCR6 = PORT_PCR_MUX(2);
    PORTC_PCR7 = PORT_PCR_MUX(2) | PORT_PCR_PE | PORT_PCR_PS;
    PORTD_PCR1 = PORT_PCR_MUX(2);

    /* Enable the SPI0 module as a master. */

    SPI0_MCR = (SPI_MCR_HALT | SPI_MCR_MSTR | SPI_MCR_DCONF(0) |
                SPI_MCR_CLR_RXF | SPI_MCR_CLR_TXF | SPI_MCR_ROOE);

    SPI0_TCR = SPI_TCR_TCNT(0);

    /* Set clock to run at 375kHz during initialization. */

    SPI0_CTAR0 = SPI_CTAR_FMSZ(7) | SPI_CTAR_PBR(0) | SPI_CTAR_BR(5);

    SPI0_MCR &= ~SPI_MCR_HALT;

    delay_ms(250);

    for (i = 0 ; i < 10 ; i += 1) {
        SPI0_PUSHR = (uint32_t)0xff | SPI_PUSHR_PCS_MASK | SPI_PUSHR_CTAS(0);
    }

    delay_ms(10);

    send_command(0, 0);  /* Reset the card. */
    send_command(59, 1); /* Turn CRC on. */

    send_command(8, 0x1aa); /* This is needed for SDHC card */
    send_command(58, 0);    /* support. */

    /* Tell the card to initialize and wait until it does so.  The
     * specs require a 100-400kHz clock signal to be present during
     * this time so we wait until after we're initilized to speed up
     * the clock. */

    RESET_PIT(3000000);
    while(send_command(55, 0), send_command(41, (1 << 30)) != 0x0) {
        assert (!PIT_TIMED_OUT());
    }
    DISABLE_PIT();

    /* Clock the card at 24Mhz. */

    SPI0_CTAR0 = (SPI_CTAR_FMSZ(7) | SPI_CTAR_DBR | SPI_CTAR_PBR(0) |
                  SPI_CTAR_BR(0));

    /* Issue a CMD58 to see if we have an HC card.  The CCS bit in the
     * OCR is only valid after ACMD41.  */

    send_command(58, 0);
    send_command(16, 512);
}

void sdio_read_single_block(int32_t addr, uint8_t *buffer)
{
#ifdef DEBUG
    usbserial_trace("cmd = 17, %x\n", addr);
#endif

    sleep_while (context.busy);
    initialize_command(17, ccs ? addr : addr << 9, buffer, NULL, NULL);
}

void sdio_read_multiple_blocks(int32_t addr, uint8_t *buffer,
                               sdio_TransactionCallback callback,
                               void *userdata)
{
#ifdef DEBUG
    usbserial_trace("cmd = 18, %x\n", addr);
#endif

    sleep_while (context.busy);
    initialize_command(18, ccs ? addr : addr << 9, buffer, callback, userdata);
}

void sdio_write_single_block(int32_t addr, uint8_t *buffer)
{
#ifdef DEBUG
    usbserial_trace("cmd = 24, %x\n", addr);
#endif

    sleep_while (context.busy);
    initialize_command(24, ccs ? addr : addr << 9, buffer, NULL, NULL);
}

void sdio_write_multiple_blocks(int32_t addr, uint8_t *buffer,
                                sdio_TransactionCallback callback,
                                void *userdata)
{
#ifdef DEBUG
    usbserial_trace("cmd = 25, %x\n", addr);
#endif

    sleep_while (context.busy);
    initialize_command(25, ccs ? addr : addr << 9, buffer, callback, userdata);
}

uint8_t sdio_get_status()
{
    uint8_t status;

#ifdef DEBUG
    usbserial_trace("cmd = 13\n");
#endif

    sleep_while (context.busy);
    initialize_command(13, 0, &status, NULL, NULL);
    sleep_while (context.busy);

#ifdef DEBUG
    usbserial_trace("resp = %x, status = %x\n", context.response, status);
#endif

    return status;
}

volatile int sdio_is_busy()
{
    return context.busy;
}
