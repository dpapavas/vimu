#include <stdio.h>
#include <stdlib.h>

#include "mk20dx128.h"
#include "i2c.h"
#include "usbserial.h"
#include "util.h"

#define TIMEOUT 1000

#define RESET_PIT()                                     \
    unpend_interrupt(30);                               \
    PIT_TFLG(0) |= PIT_TFLG_TIF;                        \
    PIT_LDVAL(0) = F_BUS / 1000000 * TIMEOUT - 1;       \
    PIT_TCTRL(0) = PIT_TCTRL_TIE;                       \
    PIT_TCTRL(0) |= PIT_TCTRL_TEN;

#define DISABLE_PIT()                                   \
    unpend_interrupt(30);                               \
    PIT_TFLG(0) |= PIT_TFLG_TIF;                        \
    PIT_TCTRL(0) = 0;

#define RESET_PIT_NOINT()                               \
    PIT_LDVAL(0) = F_BUS / 1000000 * TIMEOUT - 1;       \
    PIT_TCTRL(0) |= PIT_TCTRL_TEN;

#define DISABLE_PIT_NOINT()                             \
    PIT_TFLG(0) |= PIT_TFLG_TIF;                        \
    PIT_TCTRL(0) = 0;

#define BUSY_WAIT(cond)                                         \
    {                                                           \
        RESET_PIT_NOINT();                                      \
                                                                \
        while(!(cond)) {                                        \
            if (PIT_TFLG(0) & PIT_TFLG_TIF) {                   \
                errors += 1;                                    \
                                                                \
                DISABLE_PIT_NOINT();                            \
                STOP();                                         \
                                                                \
                return -1;                                      \
            }                                                   \
        }                                                       \
                                                                \
        DISABLE_PIT_NOINT();                                    \
    }

#define BLOCK()                                 \
    {                                           \
        BUSY_WAIT(I2C0_S & I2C_S_IICIF);        \
                                                \
        if(I2C0_S & I2C_S_ARBL) {               \
            STOP();                             \
            return -1;                          \
        }                                       \
                                                \
        I2C0_S |= I2C_S_IICIF;                  \
    }

#define RECEIVE() I2C0_C1 &= ~I2C_C1_TX
#define TRANSMIT() I2C0_C1 |= I2C_C1_TX
#define START() assert(!(I2C0_S & I2C_S_BUSY)); I2C0_C1 |= I2C_C1_MST; while(!(I2C0_S & I2C_S_BUSY))
#define STOP() assert(I2C0_S & I2C_S_BUSY); I2C0_C1 &= ~I2C_C1_MST; while(I2C0_S & I2C_S_BUSY)
#define REPEATED_START() I2C0_C1 |= I2C_C1_RSTA

#define WRITE(b)                                \
    {                                           \
        I2C0_D = (uint8_t)(b);                  \
        BLOCK();                                \
                                                \
        if (I2C0_S & I2C_S_RXAK) {              \
            errors += 1;                        \
                                                \
            STOP();                             \
                                                \
            return -1;                          \
        }                                       \
    }

#define READ()                                  \
    ({                                          \
        uint8_t i;                              \
                                                \
        i = I2C0_D;                             \
        BLOCK();                                \
        i;                                      \
    })

#define NACK() I2C0_C1 |= I2C_C1_TXAK;
#define ACK() I2C0_C1 &= ~I2C_C1_TXAK;

#define WRITE_NOBLOCK(b) I2C0_D = (uint8_t)(b)
#define READ_NOBLOCK(b) I2C0_D

static uint32_t errors;

static struct {
    uint8_t *buffer;
    uint32_t length, read;
    uint8_t phase, slave, reg;
    i2c_data_ready_callback callback;
} context;

__attribute__((interrupt ("IRQ"))) void pit0_isr(void)
{
    errors += 1;

    STOP();

    DISABLE_PIT();
    PIT_TFLG(0) |= PIT_TFLG_TIF;
}

__attribute__((interrupt ("IRQ"))) void i2c0_isr()
{
    if (I2C0_S & I2C_S_ARBL) {
        goto error;
    } else if (I2C0_S & I2C_S_TCF) {
        switch (context.phase) {
        case 0:
            RESET_PIT();

            if (I2C0_S & I2C_S_RXAK) {
                goto error;
            }

            WRITE_NOBLOCK(context.reg);

            context.phase += 1;
            break;

        case 1:
            RESET_PIT();

            if (I2C0_S & I2C_S_RXAK) {
                goto error;
            }

            if (context.length == 1) {
                NACK();
            }

            REPEATED_START();
            WRITE_NOBLOCK((context.slave << 1) | 1);

            context.phase += 1;
            break;

        case 2:
            RESET_PIT();

            if (I2C0_S & I2C_S_RXAK) {
                goto error;
            }

            RECEIVE();
            READ_NOBLOCK();

            context.phase += 1;
            break;

        case 3:
            if (context.read < context.length - 1) {
                RESET_PIT();

                if (context.read == context.length - 2) {
                    NACK();
                }

                context.buffer[context.read ^ 1] = READ_NOBLOCK();
                context.read += 1;
            } else {
                DISABLE_PIT();

                /* We're done, stop the i2c module. */

                STOP();
                TRANSMIT();

                /* usbserial_printf("* %8x, %d, %d\n",
                   context.buffer, context.length, context.read); */

                /* Write the last byte and call the callback. */

                context.buffer[context.read ^ (!(context.length & 1))] = I2C0_D;
                context.read += 1;

                context.callback(I2C_READ_SUCCESS, context.read);
            }
        }
    } else {
        usbserial_printf("%b\n", I2C0_S);
        assert(0);
    }

    I2C0_S |= I2C_S_IICIF;

    return;

error:
    /* There was an error, stop the i2c module. */

    DISABLE_PIT();
    STOP();

    errors += 1;

    /* Call the callback. */

    context.callback(I2C_READ_ERROR, context.read);

    I2C0_S |= I2C_S_IICIF;

    return;
}

unsigned int i2c_bytes_read()
{
    return context.read;
}

int i2c_read_noblock(uint8_t slave, uint8_t reg, uint8_t *buffer, size_t n,
                     i2c_data_ready_callback callback)
{
    if (I2C0_S & I2C_S_BUSY) {
        return 1;
    }

    if (n == 0) {
        return 0;
    }

    /* Enable interrupts. */

    I2C0_C1 |= I2C_C1_IICIE;
    I2C0_S |= I2C_S_IICIF;

    context.phase = 0;
    context.slave = slave;
    context.reg = reg;
    context.buffer = buffer;
    context.length = n;
    context.read = 0;
    context.callback = callback;

    START();
    TRANSMIT();
    ACK();

    WRITE_NOBLOCK((slave << 1) | 0);

    /* Start the timeout timer. */

    RESET_PIT();

    return 0;
}

int i2c_read(uint8_t slave, uint8_t reg, uint8_t *buffer, size_t n)
{
    int i;

    if (I2C0_S & I2C_S_BUSY) {
        return -1;
    }

    I2C0_C1 &= ~I2C_C1_IICIE;
    I2C0_S |= I2C_S_IICIF;

    /* Start the transmission. */

    TRANSMIT();
    START();
    ACK();

    /* Select the slave and register to read. */

    WRITE((slave << 1) | 0);
    WRITE(reg);

    /* Do a repeated start to switch to reception. */

    if (n == 1) {
        NACK();
    }

    REPEATED_START();
    WRITE((slave << 1) | 1);

    /* Switch to reception and prime the data register. */

    RECEIVE();
    READ();

    /* Read all but the last bytes and store with proper
     * endianness. */

    for (i = 0 ; i < n - 1 ; i += 1) {
        if (i == n - 2) {
            NACK();
        }

        buffer[i ^ 1] = READ();
    }

    /* Stop transaction and read the remaining byte. */

    STOP();
    TRANSMIT();

    buffer[i ^ (!(n & 1))] = I2C0_D;

    return 0;
}

int i2c_write(uint8_t slave, uint8_t reg, uint8_t value)
{
    if (I2C0_S & I2C_S_BUSY) {
        return -1;
    }

    I2C0_C1 &= ~I2C_C1_IICIE;
    I2C0_S |= I2C_S_IICIF;

    /* Start the transmission. */

    START();
    TRANSMIT();
    NACK();

    /* Select the slave and register to write. */

    WRITE((slave << 1) | 0);
    WRITE(reg);

    /* Write value and stop the transaction. */

    WRITE(value);

    STOP();

    return 0;
}

void i2c_initialize()
{
    /* Set up the required interrupts:
       I2C0 */

    enable_interrupt(11);
    prioritize_interrupt(11, 1);

    /* PIT0 */

    enable_interrupt(30);
    prioritize_interrupt(30, 1);

    /* Enable the i2c and PORTB module clocks. */

    SIM_SCGC4 |= SIM_SCGC4_I2C0;
    SIM_SCGC5 |= SIM_SCGC5_PORTB;

    /* Enable i2c function for PTB2, PTB3. */

    PORTB_PCR2 = PORTB_PCR3 = (PORT_PCR_MUX(2) | PORT_PCR_ODE |
                               PORT_PCR_DSE | PORT_PCR_PFE |
                               PORT_PCR_SRE);

    /* Enable the i2c module and set the bus speed to 400Khz with a
     * SDA hold time of 0.75us, a SCL start hold time of 0.92us and a
     * SCL stop hold time of 1.33us. */

    I2C0_F = I2C_F_MULT(2) | I2C_F_ICR(5);
    I2C0_FLT = I2C_FLT_FLT(4);
    I2C0_C1 |= I2C_C1_IICEN;
    I2C0_C2 |= I2C_C2_HDRS;
}
