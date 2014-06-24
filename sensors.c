#include <unistd.h>
#include <string.h>

#include "mk20dx128.h"
#include "i2c.h"
#include "sensors.h"
#include "util.h"

#define LINE_WIDTH (10 * sizeof(uint16_t))
#define RING_CAPACITY (3 * LINE_WIDTH)

static struct {
    uint8_t buffer[RING_CAPACITY];
    uint8_t read, write;
} ring;

static uint8_t online;
static uint16_t pending, transferred;

static void configure_register(uint8_t slave, uint8_t reg, uint8_t value)
{
    uint8_t r;

    /* Write a register and read it back to validate the
     * transmission.  Retry if necessary. */

    for(;;) {
        if (i2c_write(slave, reg, value) == 0 &&
            i2c_read(slave, reg, &r, 1) == 0 &&
            r == value) {
            break;
        }
    }
}

void power_sensors_down()
{
    /* Pull PB1 low to power down the sensors. */

    GPIOB_PCOR |= ((uint32_t)1 << 1);

    online = 0;
}

void power_sensors_up()
{
    /* Configure PB1 and and PD5 as outputs and pull them high and low
     * respectively to power the sensors. */

    PORTB_PCR1 = PORT_PCR_MUX(1) | PORT_PCR_DSE;

    GPIOB_PDDR |= ((uint32_t)1 << 1);
    GPIOB_PSOR |= ((uint32_t)1 << 1);

    PORTD_PCR5 = PORT_PCR_MUX(1) | PORT_PCR_DSE;
    GPIOD_PDDR = ((uint32_t)1 << 5);
    GPIOD_PCOR |= ((uint32_t)1 << 1);

    /* Wait for the sensor board to power-on. */

    delay_ms(100);

    configure_register(MPU6050, WHO_AM_I, 0x68);

    /* Use the gyroscope as clock source and wake up. */

    configure_register (MPU6050, PWR_MGMT_1, PWR_MGMT_1_CLKSEL(1));

    /* Set sample rate divider. */

    configure_register (MPU6050, SMPRT_DIV, SAMPLE_RATE_DIVIDER);

    /* Set low pass filter. */

    configure_register (MPU6050, CONFIG, CONFIG_DLPF_CFG(DLPF_BANDWIDTH));

    /* Set gyro full-scale range. */

    configure_register (MPU6050, GYRO_CONFIG, GYRO_CONFIG_FS_SEL(GYRO_FSR));

    /* Set accelerometer full-scale range. */

    configure_register (MPU6050, ACCEL_CONFIG, ACCEL_CONFIG_AFS_SEL(ACCEL_FSR));

    /* Set the i2c clock to 400Khz. */

    configure_register (MPU6050, I2C_MST_CTRL,
                        I2C_MST_CTRL_I2C_MST_CLK(13) |
                        I2C_MST_CTRL_WAIT_FOR_ES);

    /* Enable i2c bypass mode. */

    configure_register (MPU6050, USER_CTRL, 0);
    configure_register (MPU6050, INT_PIN_CFG, INT_PIN_CFG_I2C_BYPASS_EN);

    /* Configure the magnetometer. */

    configure_register (HMC5883L, IRA, 'H');
    configure_register (HMC5883L, IRB, '4');
    configure_register (HMC5883L, IRC, '3');

    /* Set output rate to 75Hz, set the gain and enter continuous
     * measurement mode. */

    configure_register (HMC5883L, CRA, CRA_DO(6) | CRA_MS(0) | CRA_MA(0));
    configure_register (HMC5883L, CRB, CRB_GN(MAG_GAIN));
    configure_register (HMC5883L, MR, MR_MD(0));

    /* Configure slave 0 to read the magnetometer values. */

    configure_register (MPU6050, I2C_SLV0_ADDR,
                        I2C_SLV0_ADDR_I2C_SLV0_ADDR(HMC5883L) |
                        I2C_SLV0_ADDR_I2C_SLV0_RW);

    configure_register (MPU6050, I2C_SLV0_REG, 3);
    configure_register (MPU6050, I2C_SLV0_CTRL,
                        I2C_SLV0_CTRL_I2C_SLV0_LEN(6) |
                        I2C_SLV0_I2C_SLV0_EN);

    /* Disable bypass mode and start writing to the FIFO.*/

    configure_register (MPU6050, INT_PIN_CFG, 0);

    configure_register (MPU6050, FIFO_EN,
                        FIFO_EN_XG_FIFO_EN |
                        FIFO_EN_YG_FIFO_EN |
                        FIFO_EN_ZG_FIFO_EN |
                        FIFO_EN_ACCEL_FIFO_EN |
                        FIFO_EN_TEMP_FIFO_EN |
                        FIFO_EN_SLV0_FIFO_EN);

    configure_register (MPU6050, USER_CTRL,
                        USER_CTRL_I2C_MST_EN |
                        USER_CTRL_FIFO_EN);

    online = 1;
    transferred = pending = 0;
}

int sensors_are_online()
{
    return online;
}

int read_sensor_values(int16_t *line)
{
    /* Check whether we can start a new transfer. */

    if (!i2c_busy()) {
        int n;

        /* If a transmission was in progress check to see if we
         * got what we asked for. */

        if (pending > 0) {
            n = i2c_bytes_read();

            assert(pending >= n);

            transferred += n;
            pending -= n;
        }

        /* If fewer than requested bytes were read due to a
         * transmission error, schedule a read for the rest
         * otherwise try to schedule a read for the next line. */

        if (pending > 0) {
            assert(!(i2c_read_noblock(MPU6050, FIFO_R_W,
                                      ring.buffer +
                                      (ring.write - pending +
                                       RING_CAPACITY) % RING_CAPACITY,
                                      pending)));
        } else {
            if (RING_CAPACITY - (transferred + pending) >= LINE_WIDTH) {
                int k;

                /* Try to fill as much of the ring buffer as possible in
                 * one go to save on overhead. */

                while ((k = i2c_read_register_word(MPU6050, FIFO_COUNT)) < 0);
                assert(k >= 0 && k < 1024);

                if (k >= LINE_WIDTH) {
                    /* usbserial_trace("t: %8x, %d\n", 100, 100); */

                    assert(RING_CAPACITY - ring.write >= LINE_WIDTH);

                    pending = LINE_WIDTH;
                    assert(!(i2c_read_noblock(MPU6050, FIFO_R_W,
                                              ring.buffer + ring.write,
                                              pending)));

                    ring.write = (ring.write + pending) % RING_CAPACITY;
                }
            }
        }
    }

    if (transferred >= LINE_WIDTH) {
        memcpy(line, (int16_t *)(ring.buffer + ring.read), LINE_WIDTH);
        ring.read = (ring.read + LINE_WIDTH) % RING_CAPACITY;
        transferred -= LINE_WIDTH;

        return 0;
    } else {
        return -1;
    }
}
