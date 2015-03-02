#ifndef _SDIO_H_
#define _SDIO_H_

#include <stdint.h>

typedef enum {
    SDIO_IN_PROGRESS,
    SDIO_FAILED,
} SDTransactionStatus;

typedef uint8_t *(*sdio_TransactionCallback)(SDTransactionStatus status,
                                              uint8_t *buffer,
                                              void *userdata);

void sdio_initialize();
void sdio_read_single_block(int32_t addr, uint8_t *buffer);
void sdio_read_multiple_blocks(int32_t addr, uint8_t *buffer,
                               sdio_TransactionCallback callback,
                               void *userdata);
void sdio_write_single_block(int32_t addr, uint8_t *buffer);
void sdio_write_multiple_blocks(int32_t addr, uint8_t *buffer,
                                sdio_TransactionCallback callback,
                                void *userdata);
uint8_t sdio_get_status();
volatile int sdio_is_busy();

#endif
