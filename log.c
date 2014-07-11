#include <string.h>

#include "mk20dx128.h"
#include "sdio.h"
#include "fusion.h"
#include "usbserial.h"
#include "util.h"

#define INDEX_BLOCKS 2
#define SAMPLE_SIZE sizeof(float)
#define ENTRIES_PER_INDEX_BLOCK (512 / sizeof(context.entry))

/************************************************
 * Block 0: Master index block.                 *
 *                                              *
 * +--------------------------------+           *
 * | Count/32 |                     |           *
 * +----------+---------------------+           *
 * |                                |           *
 * .                                .           *
 * .                                .           *
 * .                                .           *
 * +--------------------------------+           *
 *                                              *
 *                                              *
 * Blocks 1 - INDEX_BLOCKS:  Index blocks       *
 *                                              *
 * +------------------------------------------+ *
 * | ID/32 | DATA/32  | BLOCK/32 | SAMPLES/32 | *
 * +-------+----------+----------+------------+ *
 * .       .          .          .            . *
 * .       .          .          .            . *
 *         ENTRIES_PER_INDEX_BLOCK rows         *
 * .       .          .          .            . *
 * +-------+----------+----------+------------+ *
 *                                              *
 ************************************************/

static struct {
    uint32_t entry[4];
    uint16_t block, fill;
    uint8_t offset, width;
    uint8_t buffers[2][512];
} context;

static uint32_t data = (1 << 10) - 1;

static void fusion_data_ready(uint8_t *line)
{
    int spillover;

    spillover = context.width - (sizeof(context.buffers[0]) - context.fill);

    if (spillover < 0) {
        /* Write entire lines to the buffer if they fit. */

        memcpy (context.buffers[context.entry[3] % 2] + context.fill,
                line, context.width);
        context.fill += context.width;
    } else {
        toggle_led();

        /* If the line doesn't fit entirely, write the beginning of
         * the next line. */

        memcpy (context.buffers[context.entry[3] % 2] + context.fill,
                line, context.width - spillover);

        /* Flush the buffer into the SD card.  */

        sleep_while(sdio_is_busy());
        sdio_write_single_block(context.entry[2] + context.entry[3],
                                context.buffers[context.entry[3] % 2]);

        /* usbserial_trace("%d\n", */
        /*                 context.entry[2] + context.entry[3]); */

        context.entry[3] += 1;
        context.fill = spillover;

        /* If there's left-over data from the last line copy it to the
         * start of the new buffer. */

        if (spillover > 0) {
            memcpy(context.buffers[context.entry[3] % 2],
                   line + context.width - spillover, spillover);
        }
    }
}

static uint32_t calculate_next_free_block(uint32_t *entry)
{
    uint32_t f;
    int n;

    /* Count the number of channels in the log by counting the
     * number of bits set in the entry's data field. */

    for (n = 0, f = entry[1] ; f ; n += 1) {
        f &= f - 1;
    }

    /* Add the number of sample data blocks of the last entry to
     * its starting block and round upwards. */

    return entry[2] + (entry[3] * n * SAMPLE_SIZE + 511) / 512;
}

void log_initialize()
{
    uint8_t buffer[512];
    int i;

    /* Just clear the index blocks. */

    memset(buffer, 0, sizeof(buffer));

    for (i = 0 ; i < INDEX_BLOCKS + 1 ; i += 1) {
        sdio_write_single_block(i, buffer);
        sleep_while(sdio_is_busy());
    }
}

void log_begin()
{
    uint32_t *header, f;
    uint8_t buffer[512];
    int m, n;

    sdio_read_single_block(0, buffer);
    sleep_while(sdio_is_busy());

    header = (uint32_t *)buffer;

    n = header[0];
    context.block = n / ENTRIES_PER_INDEX_BLOCK + 1;
    context.offset = n % ENTRIES_PER_INDEX_BLOCK;

    if (context.block == 1 && context.offset == 0) {
        /* This is the first entry so we only need to skip the index
         * blocks. */

        m = 1 + INDEX_BLOCKS;
    } else if (context.offset == 0) {
        uint32_t *entry;

        /* The last entry is stored in the previous block so we need
         * to fetch it. */

        sdio_read_single_block(context.block - 1, buffer);
        sleep_while(sdio_is_busy());

        entry = (uint32_t *)(buffer + (ENTRIES_PER_INDEX_BLOCK - 1) *
                             sizeof(context.entry));
        m = calculate_next_free_block(entry);
    } else {
        uint32_t *entry;

        sdio_read_single_block(context.block, buffer);
        sleep_while(sdio_is_busy());

        entry = (uint32_t *)(buffer + (context.offset - 1) *
                             sizeof(context.entry));
        m = calculate_next_free_block(entry);
    }

    context.entry[0] = n + 1;
    context.entry[1] = data;
    context.entry[2] = m;
    context.entry[3] = 0;

    /* Count the number of channels in the log by counting the
     * number of bits set in the entry's data field. */

    for (context.width = 0, f = data ; f ; context.width += SAMPLE_SIZE) {
        f &= f - 1;
    }

    context.fill = 0;

    fusion_set_callback(fusion_data_ready);
    fusion_start();
}

void log_end()
{
    uint8_t buffer[512];
    uint32_t *n;

    fusion_set_callback(NULL);

    /* usb_write("+++\n", 4, 1); */

    /* Fetch the last index block, add the entry and write it back. */

    sdio_read_single_block(context.block, buffer);
    sleep_while(sdio_is_busy());

    memcpy(buffer + context.offset * sizeof(context.entry),
           context.entry, sizeof(context.entry));

    sdio_write_single_block(context.block, buffer);
    sleep_while(sdio_is_busy());

    /* Fetch the master index block, increment the number of entries
     * and write it back. */

    sdio_read_single_block(0, buffer);
    sleep_while(sdio_is_busy());

    n = (uint32_t *)(buffer + 0);
    *n += 1;

    sdio_write_single_block(0, buffer);
    sleep_while(sdio_is_busy());
}

void log_list()
{
    uint8_t buffer[512];
    uint32_t *n_p, n;
    int i;

    /* Fetch the master index block and read the number of logs. */

    sdio_read_single_block(0, buffer);
    sleep_while(sdio_is_busy());

    n_p = (uint32_t *)(buffer + 0);
    n = *n_p;

    for (i = 0 ; i < n ; i += 1) {
        uint32_t *entry;

        if (i % ENTRIES_PER_INDEX_BLOCK == 0) {
            sdio_read_single_block(i / ENTRIES_PER_INDEX_BLOCK + 1, buffer);
            sleep_while(sdio_is_busy());
        }

        entry = (uint32_t *)(buffer + (i % ENTRIES_PER_INDEX_BLOCK) *
                             sizeof(context.entry));

        usbserial_printf("%d %x %d %d\n",
                         entry[0], entry[1], entry[2], entry[3]);
    }
}

void log_set_data(uint32_t new_data)
{
    data = new_data;
    usbserial_printf("%d\n", new_data);
}

uint32_t log_get_data()
{
    return data;
}
