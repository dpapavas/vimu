#ifndef _LOG_H_
#define _LOG_H_

#include <stdint.h>

void log_initialize();
void log_record(uint32_t data, int rate, int count);
void log_write_block(uint32_t addr, uint8_t *buffer);
void log_list();
void log_replay(uint32_t key, int lines);

#endif
