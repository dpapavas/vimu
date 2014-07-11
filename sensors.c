#include <unistd.h>
#include <string.h>

#include "mk20dx128.h"
#include "i2c.h"
#include "sensors.h"
#include "util.h"

/* #undef assert */
/* #define assert usbserial_assert */

#define MAX_RETRIES 5
#define LINE_WIDTH 10

static int16_t buffers[2][LINE_WIDTH];
static uint8_t online, r = 0, w = 0;
static volatile uint8_t fetched, available;

static void fetch_more_data();

static void data_ready(int status, int count)
{
    assert(status == I2C_READ_SUCCESS);

    fetched += 1;
    available -= 1;
    r = !r;

    if (available > 0) {
        fetch_more_data();
    }
}

static void fetch_more_data()
{
    /* usbserial_trace("%d, %d\n", */
    /*                 available, i2c_read_register_word(MPU6050, */
    /*                                                   FIFO_COUNT)); */

    assert(fetched < 2);
    assert(i2c_read_noblock(MPU6050, FIFO_R_W, (uint8_t *)(buffers[w]),
                            sizeof(buffers[0]), data_ready) == 0);

    w = !w;
}

__attribute__((interrupt ("IRQ"))) void portb_isr(void)
{
    if (PORTB_PCR0 & PORT_PCR_ISF) {
        PORTB_PCR0 |= PORT_PCR_ISF;

        available += 1;

        /* Make sure the FIFO hasn't overflown. */

        assert(available * sizeof(buffers[0]) < 1024);

        /* { */
        /*     static uint64_t c; */
        /*     uint64_t t; */

        /*     t = cycles(); */
        /*     usbserial_trace("%lu\n", t - c); */
        /*     c = t; */
        /* } */

        if (available == 1) {
            fetch_more_data();
        }
    }
}

static void configure_register(uint8_t slave, uint8_t reg, uint8_t value)
{
    uint8_t r;
    int i;

    /* Write a register and read it back to validate the
     * transmission.  Retry if necessary. */

    for(i = 0 ; i < MAX_RETRIES ; i += 1) {
        if (i2c_write(slave, reg, value) == 0 &&
            i2c_read(slave, reg, &r, 1) == 0 &&
            r == value) {
            break;
        }
    }

    assert(i < MAX_RETRIES);
}

void power_sensors_down()
{
    /* Pull PB0 low to power down the sensors. */

    GPIOD_PCOR |= ((uint32_t)1 << 6);

    online = 0;
}

void power_sensors_up()
{
    SIM_SCGC5 |= SIM_SCGC5_PORTD | SIM_SCGC5_PORTC;

    /* Configure PB1 and and PD5 as outputs and pull them high and low
     * respectively to power the sensors. */

    PORTD_PCR5 = PORT_PCR_MUX(1) | PORT_PCR_DSE;
    PORTC_PCR1 = PORT_PCR_MUX(1) | PORT_PCR_DSE;

    GPIOD_PDDR |= ((uint32_t)1 << 5);
    GPIOD_PCOR |= ((uint32_t)1 << 5);

    GPIOC_PDDR |= ((uint32_t)1 << 1);
    GPIOC_PSOR |= ((uint32_t)1 << 1);

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

    /* Disable bypass mode and enable interrupts. */

    configure_register (MPU6050, INT_PIN_CFG,
                        INT_PIN_CFG_INT_RD_CLEAR);

    configure_register (MPU6050, INT_ENABLE, INT_ENABLE_DATA_RDY_EN);

    /* Start writing to the FIFO. */

    configure_register (MPU6050, FIFO_EN,
                        FIFO_EN_XG_FIFO_EN |
                        FIFO_EN_YG_FIFO_EN |
                        FIFO_EN_ZG_FIFO_EN |
                        FIFO_EN_ACCEL_FIFO_EN |
                        FIFO_EN_TEMP_FIFO_EN |
                        FIFO_EN_SLV0_FIFO_EN);

    configure_register (MPU6050, USER_CTRL,
                        USER_CTRL_I2C_MST_EN | USER_CTRL_FIFO_EN);

    /* Configure PB0 (connected to INTA) as an interrupt source. */

    SIM_SCGC5 |= SIM_SCGC5_PORTB;
    PORTB_PCR0 = PORT_PCR_MUX(1) | PORT_PCR_IRQC(9);

    enable_interrupt(41);
    prioritize_interrupt(41, 0);

    online = 1;
}

int sensors_are_online()
{
    return online;
}

void read_sensor_values(int16_t *line)
{
    sleep_while (fetched == 0);

    memcpy (line, buffers[r], sizeof(buffers[0]));
    fetched -= 1;
}
