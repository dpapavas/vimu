#ifndef _I2C_H_
#define _I2C_H_

#include <stdio.h>

#define I2C_READ_SUCCESS 0
#define I2C_READ_ERROR 1

typedef void (*i2c_data_ready_callback)(int status, int count);

void i2c_initialize();
int i2c_read(uint8_t slave, uint8_t reg, uint8_t *buffer, size_t n);
int i2c_write(uint8_t slave, uint8_t reg, uint8_t value);

int i2c_read_noblock(uint8_t slave, uint8_t reg, uint8_t *buffer, size_t n,
                     i2c_data_ready_callback callback);

#define i2c_is_busy() (I2C0_S & I2C_S_BUSY)

unsigned int i2c_bytes_read();

#define i2c_read_register_byte(slave, reg)                      \
    ({                                                          \
        uint8_t i;                                              \
        i2c_read(slave, reg, &i, 1) < 0 ? -1 : i;               \
    })

#define i2c_read_register_word(slave, reg)                      \
    ({                                                          \
        uint16_t i;                                             \
        (i2c_read(slave, reg, (uint8_t *)&i, 2) < 0 ? - 1 : i); \
    })

#endif
