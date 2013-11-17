#define _XOPEN_SOURCE
#include <unistd.h>

#include "mk20dx128.h"
#include "usbserial.h"
#include "util.h"
#include "i2c.h"

#define MPU6050 0x68
#define HMC5883L 0x1e

#define SAMPLE_RATE_DIVIDER 0
#define DLPF_BANDWIDTH 1
#define GYRO_FSR 1
#define ACCEL_FSR 0
#define MAG_GAIN 1

#define WHO_AM_I 117
#define PWR_MGMT_1 107
#define PWR_MGMT_1_DEVICE_RESET ((uint8_t)1 << 7)
#define PWR_MGMT_1_CLKSEL(n) ((uint8_t)(n) & 0b111)
#define SMPRT_DIV 25

#define CONFIG 26
#define CONFIG_DLPF_CFG(n) ((uint8_t)(n) & 0b111)

#define GYRO_CONFIG 27
#define GYRO_CONFIG_FS_SEL(n) ((uint8_t)((n & 0b11)) << 3)

#define ACCEL_CONFIG 28
#define ACCEL_CONFIG_AFS_SEL(n) ((uint8_t)((n & 0b11)) << 3)

#define FIFO_EN 35
#define FIFO_EN_TEMP_FIFO_EN ((uint8_t)1 << 7)
#define FIFO_EN_XG_FIFO_EN ((uint8_t)1 << 6)
#define FIFO_EN_YG_FIFO_EN ((uint8_t)1 << 5)
#define FIFO_EN_ZG_FIFO_EN ((uint8_t)1 << 4)
#define FIFO_EN_ACCEL_FIFO_EN ((uint8_t)1 << 3)
#define FIFO_EN_SLV2_FIFO_EN ((uint8_t)1 << 2)
#define FIFO_EN_SLV1_FIFO_EN ((uint8_t)1 << 1)
#define FIFO_EN_SLV0_FIFO_EN ((uint8_t)1 << 0)

#define I2C_MST_CTRL 36
#define I2C_MST_CTRL_I2C_MST_CLK(n) ((uint8_t)(n) & 0b1111)
#define I2C_MST_CTRL_WAIT_FOR_ES ((uint8_t)1 << 6)

#define USER_CTRL 106
#define USER_CTRL_FIFO_EN ((uint8_t)1 << 6)
#define USER_CTRL_I2C_MST_EN ((uint8_t)1 << 5)
#define USER_CTRL_FIFO_RESET ((uint8_t)1 << 2)
#define USER_CTRL_I2C_MST_RESET ((uint8_t)1 << 1)
#define USER_CTRL_SIG_COND_RESET ((uint8_t)1 << 0)

#define INT_PIN_CFG 55 
#define INT_PIN_CFG_I2C_BYPASS_EN  ((uint8_t)1 << 1)

#define I2C_SLV0_ADDR 37
#define I2C_SLV0_ADDR_I2C_SLV0_ADDR(n) ((uint8_t)(n) & 0b111111)
#define I2C_SLV0_ADDR_I2C_SLV0_RW ((uint8_t)1 << 7)

#define I2C_SLV0_REG 38
#define I2C_SLV0_CTRL 39
#define I2C_SLV0_CTRL_I2C_SLV0_LEN(n) ((uint8_t)(n) & 0b1111)
#define I2C_SLV0_I2C_SLV0_EN ((uint8_t)1 << 7)

#define FIFO_COUNT 114
#define FIFO_R_W 116

#define CRA 0
#define CRA_MS(n) ((uint8_t)((n & 0b11)) << 0)
#define CRA_DO(n) ((uint8_t)((n & 0b111)) << 2)
#define CRA_MA(n) ((uint8_t)((n & 0b11)) << 5)

#define CRB 1
#define CRB_GN(n) ((uint8_t)((n & 0b111)) << 5)

#define MR 2
#define MR_MD(n) ((uint8_t)((n & 0b11)) << 0)
#define IRA 10
#define IRB 11
#define IRC 12

/* Always make sure a byte is left unused in the ring buffer so that
 * we can distinguish between empty and full. */

#define LINE_WIDTH (9 * sizeof(uint16_t))
#define RING_CAPACITY (3 * LINE_WIDTH)

#define RETRIES 10
#define RETRY_DELAY_MS 1

static struct {
    uint8_t buffer[RING_CAPACITY];
    uint8_t read, write;
} ring;

static void configure_register(uint8_t slave, uint8_t reg, uint8_t value)
{
    int i;

    /* Write a register and read it back to validate the
     * transmission.  Retry if necessary. */
    
    for (i = 0 ; i < RETRIES ; i += 1) {
        uint8_t r = ~value;
        
        if (i2c_write(slave, reg, value) == 0 &&
            i2c_read(slave, reg, &r, 1) == 0 &&
            r == value) {
            break;
        }

        delay_ms(RETRY_DELAY_MS);
    }

    assert(i < RETRIES);
}    

void _initialize()
{
    /* Configure PB1 as an output and pull it high to power the
     * sensors. */
    
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
                        FIFO_EN_SLV0_FIFO_EN);

    configure_register (MPU6050, USER_CTRL,
                        USER_CTRL_I2C_MST_EN |
                        USER_CTRL_FIFO_EN);
}

int main()
{
    usb_await_enumeration();
    usbserial_await_dtr();

    _initialize();

    while(1) {
        static uint16_t pending, transferred;
        int i;

        /* if (i2c_busy()) { */
        /*     turn_on_led(); */
        /* } else { */
        /*     turn_off_led(); */
        /* } */
        
        /* { */
        /*     static int j; */

        /*     if ((j % 100) == 0) { */
        /*         turn_on_led(); */
        /*     } else { */
        /*         turn_off_led(); */
        /*     } */

        /*     j += 1; */
        /* } */

        /* usbserial_trace("* spc = %d, rd = %d, wr = %d, transferred = %d, pending = %d, busy=%d\n", RING_SPACE, ring.read, ring.write, transferred, pending, i2c_busy()); */
        
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
                assert(i2c_read_noblock(MPU6050, FIFO_R_W,
                                        ring.buffer + (ring.write - pending + RING_CAPACITY) % RING_CAPACITY,
                                        pending) == 1);
            } else {
                if (RING_CAPACITY - (transferred + pending) >= LINE_WIDTH) {
                    int k;
                    
                    /* Try to fill as much of the ring buffer as possible in
                     * one go to save on overhead. */

                    for (i = 0;
                         (k = i2c_read_register_word(MPU6050, FIFO_COUNT)) < 0 &&
                             i < RETRIES;
                         i += 1);
            
                    assert(i < RETRIES);
                    assert(k >= 0 && k <= 1024);

                    if (k >= LINE_WIDTH) {
                        /* usbserial_trace("t: %8x, %d\n", 100, 100); */

                        assert(RING_CAPACITY - ring.write >= LINE_WIDTH);
                        
                        pending = LINE_WIDTH;                    
                        assert(i2c_read_noblock(MPU6050, FIFO_R_W,
                                                ring.buffer + ring.write,
                                                pending) == 1);

                        ring.write = (ring.write + pending) % RING_CAPACITY;
                    }
                }
            }
        }

        if(transferred >= LINE_WIDTH) {
            int16_t *line;
    
            line = (int16_t *)(ring.buffer + ring.read);
            ring.read = (ring.read + LINE_WIDTH) % RING_CAPACITY;
            transferred -= LINE_WIDTH;

            usbserial_trace("%hd, %hd, %hd, %hd, %hd, %hd, %hd, %hd, %hd\n",
                             line[0], line[1], line[2],
                             line[3], line[4], line[5],
                             line[6], line[7], line[8]);
        }

        /* usbserial_trace("p = %d, t = %d\n", pending, transferred); */
    }
    
    return 0;
}
