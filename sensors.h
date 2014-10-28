#ifndef _SENSORS_H_
#define _SENSORS_H_

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
#define INT_PIN_CFG_INT_RD_CLEAR  ((uint8_t)1 << 4)
#define INT_PIN_CFG_LATCH_INT_EN  ((uint8_t)1 << 5)

#define INT_ENABLE 56
#define INT_ENABLE_DATA_RDY_EN  ((uint8_t)1 << 0)

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

#define LINE_WIDTH 10

typedef void (*sensors_data_ready_callback)(int16_t *line);

int sensors_are_online();
void read_sensor_values(int16_t *line);
void sensors_set_callback(sensors_data_ready_callback new_callback);

#endif
