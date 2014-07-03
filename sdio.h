#ifndef _SDIO_H_
#define _SDIO_H_

#include <stdint.h>

void sdio_initialize();
void sdio_read_single_block(int32_t addr, uint8_t *buffer);
void sdio_write_single_block(int32_t addr, uint8_t *buffer);
uint8_t sdio_get_status();
volatile int sdio_is_busy();

#endif
