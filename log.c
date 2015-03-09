#include <string.h>

#include "mk20dx128.h"
#include "sdio.h"
#include "fusion.h"
#include "usbserial.h"
#include "util.h"

#define RING_SIZE 16
#define CONFIGURATION_BLOCKS 1
#define INDEX_BLOCKS 16
#define ENTRY_SIZE (4 * sizeof(uint32_t))
#define SAMPLE_SIZE sizeof(float)
#define ENTRIES_PER_INDEX_BLOCK (512 / ENTRY_SIZE)
#define SAMPLE_DATA_BLOCKS(LINES, WIDTH) ((LINES * WIDTH + 511) / 512)

/*****************************************************
 * Block 0: Master index block.                      *
 *                                                   *
 * +----------+------------------------------+-...-+ *
 * | Count/32 | First key of ith index block |     | *
 * +----------+------------------------------+-...-+ *
 *                                                   *
 * Blocks 1 - INDEX_BLOCKS:  Index blocks            *
 *                                                   *
 * +--------+----------+----------+----------+       *
 * | KEY/32 | DATA/32  | BLOCK/32 | LINES/32 |       *
 * +--------+----------+----------+----------+       *
 * .        .          .          .          .       *
 * .        .          .          .          .       *
 * .       ENTRIES_PER_INDEX_BLOCK rows      .       *
 * .        .          .          .          .       *
 * +--------+----------+----------+----------+       *
 *                                                   *
 ****************************************************/

static struct {
    volatile uint32_t lines;    /* Total number of lines recorded. */
    volatile uint32_t filling;  /* Total block currently being filled. */
    volatile uint32_t writing;  /* Total block currently being written
                                 * out. */
    volatile uint16_t fill;     /* The fill, in bytes, of the current
                                 * buffer. */
    uint32_t key, data, block;  /* Entry details. */
    uint32_t target;            /* Number of entries to write. */
    uint8_t width;              /* The witdth, in bytes, of each
                                 * line. */
    uint8_t buffers[RING_SIZE][512];
} context;

static uint8_t *flush_filled_buffers(SDTransactionStatus status,
                                     uint8_t *buffer,
                                     void *userdata)
{
    uint8_t *b;

    assert(status != SDIO_FAILED);

    if (context.writing == context.filling) {
        if (fusion_in_progress()) {
            sleep_while(context.writing == context.filling);
        } else {
            return NULL;
        }
    }

    /* usbserial_trace("< %f\n", (float)cycles() / cycles_in_ms(1)); */

    b = context.buffers[context.writing % RING_SIZE];
    context.writing += 1;

    return b;
}

static int fusion_data_ready(float *samples, void *userdata)
{
    uint8_t *line = (uint8_t *)samples;
    int spillover;

    /* This takes account of the fact that context.writing points to
     * the _next_ page to be written. */

    assert (context.filling - context.writing < RING_SIZE - 1);

    spillover = context.width - (sizeof(context.buffers[0]) - context.fill);
    assert(spillover < 0 || ((context.lines + 1) * context.width) % 512 == spillover);

    if (spillover < 0) {
        /* Write entire lines to the buffer if they fit. */

        memcpy (context.buffers[context.filling % RING_SIZE] + context.fill,
                line, context.width);

        context.fill += context.width;
    } else {
        /* If the line doesn't fit entirely, write the beginning of
         * the next line. */

        memcpy (context.buffers[context.filling % RING_SIZE] + context.fill,
                line, context.width - spillover);

        /* memset(context.buffers[(context.filling + 1) % RING_SIZE], */
        /*        0xFA, sizeof(context.buffers[0])); */

        /* If there's left-over data from the last line copy it to the
         * start of the new buffer. */

        if (spillover > 0) {
            /* Make sure we're not spilling into the page currently
             * being written out. */

            assert (context.filling - context.writing < RING_SIZE - 2);

            memcpy(context.buffers[(context.filling + 1) % RING_SIZE],
                   line + context.width - spillover, spillover);
        }

        context.filling += 1;
        context.fill = spillover;

        /* If this is the first completed block, start writing to the
         * card. */

        if (context.filling == 1) {
            assert(!sdio_is_busy());
            sdio_write_multiple_blocks(context.block, NULL,
                                       flush_filled_buffers, NULL);
        }

        /* usbserial_trace("%d\n", */
        /*                 context.block + context.filling); */
    }

    /* Increment the number of entries written. */

    context.lines += 1;

    if (context.lines == context.target) {
        /* Force the last, half-finished page to be flushed, if
         * needed. */

        if(context.fill > 0) {
            context.filling += 1;

            if (context.filling == 1) {
                assert(!sdio_is_busy());
                sdio_write_multiple_blocks(context.block, NULL,
                                           flush_filled_buffers, NULL);
            }
        }

        return 0;
    } else {
        return 1;
    }

    assert((context.lines * context.width) % 512 == context.fill);
}

static void dump_data_line(uint32_t data, float *line)
{
    int j;

    j = 0;

    /* Acceleration. */

    if (data & (1 << FUSION_RAW_ACCELERATION)) {
        usbserial_printf("%+1.2f %+1.2f %+1.2f ",
                         line[j + 0], line[j + 1], line[j + 2]);

        j += 3;
    }

    /* Angular rate. */

    if (data & (1 << FUSION_RAW_ANGULAR_SPEED)) {
        usbserial_printf("%+1.2f %+1.2f %+1.2f ",
                         line[j + 0], line[j + 1], line[j + 2]);

        j += 3;
    }

    /* Magnetic field. */

    if (data & (1 << FUSION_RAW_MAGNETIC_FIELD)) {
        usbserial_printf("%+1.2f %+1.2f %+1.2f ",
                         line[j + 0], line[j + 1], line[j + 2]);

        j += 3;
    }

    /* Sensor temperature. */

    if (data & (1 << FUSION_SENSOR_TEMPERATURE)) {
        usbserial_printf("%+1.2f", line[j + 0]);

        j += 1;
    }

    usbserial_printf("\n");
}

void log_initialize()
{
    uint8_t buffer[512];
    int i;

    /* Just clear the index blocks. */

    memset(buffer, 0, sizeof(buffer));

    for (i = 0 ; i < INDEX_BLOCKS + 1 ; i += 1) {
        sdio_write_single_block(CONFIGURATION_BLOCKS + i, buffer);
        sleep_while(sdio_is_busy());
    }
}

void log_record(uint32_t data, int rate, int count)
{
    uint8_t buffer[512];
    uint16_t index;
    uint8_t offset;
    uint32_t *b;
    int m, n;

    /* We're not currently logging so start now. */

    sdio_read_single_block(0 + CONFIGURATION_BLOCKS, buffer);
    sleep_while(sdio_is_busy());

    b = (uint32_t *)buffer;
    n = b[0];

    index = n / ENTRIES_PER_INDEX_BLOCK + CONFIGURATION_BLOCKS + 1;
    offset = n % ENTRIES_PER_INDEX_BLOCK;
    context.width = fusion_samples_per_line(data) * SAMPLE_SIZE;

    if (index == CONFIGURATION_BLOCKS + 1 && offset == 0) {
        /* This is the first entry so we only need to skip the index
         * blocks. */

        m = CONFIGURATION_BLOCKS + 1 + INDEX_BLOCKS;
    } else if (offset == 0) {
        uint32_t *entry;

        /* The last entry is stored in the previous block so we need
         * to fetch it. */

        sdio_read_single_block(index - 1, buffer);
        sleep_while(sdio_is_busy());

        entry = (uint32_t *)(buffer + (ENTRIES_PER_INDEX_BLOCK - 1) *
                             ENTRY_SIZE);
        m = entry[2] + SAMPLE_DATA_BLOCKS(entry[3], context.width);
    } else {
        uint32_t *entry;

        sdio_read_single_block(index, buffer);
        sleep_while(sdio_is_busy());

        entry = (uint32_t *)(buffer + (offset - 1) *
                             ENTRY_SIZE);
        m = entry[2] + SAMPLE_DATA_BLOCKS(entry[3], context.width);
    }

    context.key = n + 1;
    context.data = data;
    context.block = m;
    context.target = count;
    context.lines = context.fill = context.filling = context.writing = 0;

    turn_on_led();
    fusion_start(data, rate, fusion_data_ready, NULL);
    sleep_while(sdio_is_busy());
    turn_off_led();

    /* Stop logging. */

    assert(!(fusion_in_progress()));
    assert(context.writing == context.filling);

    /* Fetch the master index block. */

    sdio_read_single_block(0 + CONFIGURATION_BLOCKS, buffer);
    sleep_while(sdio_is_busy());

    b = (uint32_t *)buffer;

    /* Increment the number of entries. */

    b[0] += 1;

    /* Update the master index, if we're starting a new block. */

    if (offset == 0) {
        b[index + 1] = context.key;
    }

    /* Write it back. */

    sdio_write_single_block(0 + CONFIGURATION_BLOCKS, buffer);
    sleep_while(sdio_is_busy());

    /* Fetch the last index block, add the entry and write it back. */

    sdio_read_single_block(index, buffer);
    sleep_while(sdio_is_busy());

    {
        uint32_t *entry = (uint32_t *)(buffer + offset * ENTRY_SIZE);

        entry[0] = context.key;
        entry[1] = context.data;
        entry[2] = context.block;
        entry[3] = context.lines;
    }

    sdio_write_single_block(index, buffer);
    sleep_while(sdio_is_busy());

    usbserial_printf("Recorded log %d.\n", context.key);
}

void log_list()
{
    uint8_t buffer[512];
    uint32_t *n_p, n;
    int i;

    /* Fetch the master index block and read the number of logs. */

    sdio_read_single_block(0 + CONFIGURATION_BLOCKS, buffer);
    sleep_while(sdio_is_busy());

    /* This is necessary so as not to break aliasing rules. */

    n_p = (uint32_t *)(buffer + 0);
    n = *n_p;

    if (n == 0) {
        usbserial_printf("No logs.\n");
    } else {
        for (i = 0 ; i < n ; i += 1) {
            uint32_t *entry;

            if (i % ENTRIES_PER_INDEX_BLOCK == 0) {
                sdio_read_single_block((i / ENTRIES_PER_INDEX_BLOCK +
                                        CONFIGURATION_BLOCKS + 1),
                                       buffer);
                sleep_while(sdio_is_busy());
            }

            entry = (uint32_t *)(buffer + (i % ENTRIES_PER_INDEX_BLOCK) *
                                 ENTRY_SIZE);

            usbserial_printf("%d %x %d %d\n",
                             entry[0], entry[1], entry[2], entry[3]);
        }
    }
}

void log_replay(uint32_t key, int lines)
{
    uint8_t buffer[512];
    uint32_t entry[4], *b, n;
    int i, m, width;

    /* Fetch the master index block and read the number of logs. */

    sdio_read_single_block(0 + CONFIGURATION_BLOCKS, buffer);
    sleep_while(sdio_is_busy());

    /* This is necessary so as not to break aliasing rules. */

    b = (uint32_t *)(buffer + 0);
    n = b[0];

    for (i = 0 ; i <= (n - 1) / ENTRIES_PER_INDEX_BLOCK ; i += 1) {
        if (key < ((uint32_t *)buffer)[i + 1]) {
            break;
        }
    }

    if (i == 0) {
        usbserial_printf("No such log.\n");
        return;
    }

    /* Fetch the appropriate index block. */

    sdio_read_single_block(CONFIGURATION_BLOCKS + i, buffer);
    sleep_while(sdio_is_busy());

    if (i - 1 == n / ENTRIES_PER_INDEX_BLOCK) {
        m = n - (i - 1) * ENTRIES_PER_INDEX_BLOCK;
    } else {
        m = ENTRIES_PER_INDEX_BLOCK;
    }

    /* usbserial_trace ("n = %d, i = %d, m = %d\n", n, i, m); */

    for (i = 0 ; i < m ; i += 1) {
        memcpy(entry, buffer + i * sizeof(entry), sizeof(entry));

        if (key == entry[0]) {
            break;
        }
    }

    if (lines > 0 && lines < entry[3]) {
        entry[3] = lines;
    }

    if (i == m) {
        usbserial_printf("No such log.\n");
        return;
    }

    width = fusion_samples_per_line(entry[1]) * SAMPLE_SIZE;

    {
        uint8_t scratch[width];
        int j, blocks, spillover = 0;

        blocks = SAMPLE_DATA_BLOCKS (entry[3], width);

        turn_on_led();

        for (j = 0 ; j < blocks ; j += 1) {
            int k;

            /* Fetch a new block. */

            sdio_read_single_block(entry[2] + j, buffer);
            while(sdio_is_busy());

            if (spillover > 0) {
                float *line;

                memcpy (scratch + spillover, buffer, width - spillover);
                line = (float *)scratch;

                dump_data_line(entry[1], line);
            }

            /* If it's the last block, it's probably half-full. */

            if (j == blocks - 1) {
                k = 512 - (blocks * 512 - entry[3] * width);
            } else {
                k = sizeof(buffer);
            }

            /* Print all whole lines in the block. */

            for (i = (width - spillover) % width;
                 i <= k - width;
                 i += width) {
                float *line;

                line = (float *)(buffer + i);
                dump_data_line(entry[1], line);
            }

            /* Save the broken line at the end of the block, if
             * necessary. */

            if (j < blocks - 1) {
                spillover = sizeof(buffer) - i;
                assert (spillover < width);
            } else {
                spillover = 0;
            }

            if (spillover > 0) {
                memcpy (scratch, buffer + i, spillover);
            }
        }

        turn_off_led();
    }
}
