#include <string.h>

#include "mk20dx128.h"
#include "sdio.h"
#include "fusion.h"
#include "usbserial.h"
#include "util.h"

#define RING_SIZE 16
#define INDEX_BLOCKS 16
#define ENTRY_SIZE (4 * sizeof(uint32_t))
#define SAMPLE_SIZE sizeof(float)
#define ENTRIES_PER_INDEX_BLOCK (512 / ENTRY_SIZE)

/*****************************************************
 * Block 0: Master index block.                      *
 *                                                   *
 * +-----------------------------------------+-...-+ *
 * | Count/32 | First key of ith index block |     | *
 * +----------+------------------------------+-...-+ *
 *                                                   *
 * Blocks 1 - INDEX_BLOCKS:  Index blocks            *
 *                                                   *
 * +-----------------------------------------+       *
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

static uint32_t data = (1 << 10) - 1;

static uint8_t *flush_filled_buffers(SDTransactionStatus status,
                                     uint8_t *buffer,
                                     void *userdata)
{
    uint8_t *b;

    assert(status != SDIO_FAILED);
    toggle_led();

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

static void fusion_data_ready(float *samples)
{
    uint8_t *line = (uint8_t *)samples;
    int spillover;

    /* This takes account of the fact that context.writing points to
     * the _next_ page to be written. */

    assert (context.filling - context.writing < RING_SIZE - 1);

    spillover = context.width - (sizeof(context.buffers[0]) - context.fill);
    assert(spillover < 0 || ((context.lines + 1) * context.width) % 512 == spillover);

    assert(((float *)line)[9] > 10.0);

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

        /* usbserial_trace("%d\n", */
        /*                 context.block + context.filling); */
    }

    /* Increment the number of entries written. */

    context.lines += 1;

    if (context.lines == context.target) {
        fusion_set_callback(NULL);

        /* Force the last, half-finished page to be flushed, if
         * needed. */

        if(context.fill > 0) {
            context.filling += 1;
        }
    }

    assert((context.lines * context.width) % 512 == context.fill);
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

static inline int sample_line_width(uint32_t data)
{
    uint32_t f;
    int width;

    /* Count the number of channels in the log by counting the
     * number of bits set in the entry's data field. */

    for (width = 0, f = data ; f ; width += SAMPLE_SIZE) {
        f &= f - 1;
    }

    return width;
}

static inline int sample_data_blocks(lines, width)
{
    return ((lines * width + 511) / 512);
}

void log_record(int count)
{
    uint8_t buffer[512];
    uint16_t index;
    uint8_t offset;

    {
        uint32_t *header;
        int m, n;

        /* We're not currently logging so start now. */

        sdio_read_single_block(0, buffer);
        sleep_while(sdio_is_busy());

        header = (uint32_t *)buffer;

        n = header[0];
        index = n / ENTRIES_PER_INDEX_BLOCK + 1;
        offset = n % ENTRIES_PER_INDEX_BLOCK;
        context.width = sample_line_width(data);

        if (index == 1 && offset == 0) {
            /* This is the first entry so we only need to skip the index
             * blocks. */

            m = 1 + INDEX_BLOCKS;
        } else if (offset == 0) {
            uint32_t *entry;

            /* The last entry is stored in the previous block so we need
             * to fetch it. */

            sdio_read_single_block(index - 1, buffer);
            sleep_while(sdio_is_busy());

            entry = (uint32_t *)(buffer + (ENTRIES_PER_INDEX_BLOCK - 1) *
                                ENTRY_SIZE);
            m = entry[2] + sample_data_blocks(entry[3], context.width);
        } else {
            uint32_t *entry;

            sdio_read_single_block(index, buffer);
            sleep_while(sdio_is_busy());

            entry = (uint32_t *)(buffer + (offset - 1) *
                                ENTRY_SIZE);
            m = entry[2] + sample_data_blocks(entry[3], context.width);
        }

        context.key = n + 1;
        context.data = data;
        context.block = m;
        context.target = count;
        context.lines = context.fill = context.filling = context.writing = 0;

        fusion_set_callback(fusion_data_ready);
        sdio_write_multiple_blocks(context.block, NULL,
                                   flush_filled_buffers, NULL);

        sleep_while(sdio_is_busy());
    }

    {
        uint32_t *n;

        /* Stop logging. */

        assert(!(fusion_in_progress()));
        assert(context.writing == context.filling);

        /* Fetch the master index block. */

        sdio_read_single_block(0, buffer);
        sleep_while(sdio_is_busy());

        /* Increment the number of entries. */

        n = (uint32_t *)(buffer + 0);
        index = *n / ENTRIES_PER_INDEX_BLOCK + 1;
        offset = *n % ENTRIES_PER_INDEX_BLOCK;
        *n += 1;

        /* Update the master index, if we're starting a new block. */

        if (offset == 0) {
            n = (uint32_t *)buffer + offset + 1;
            *n = context.key;
        }

        /* Write it back. */

        sdio_write_single_block(0, buffer);
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
    }
}

void log_list()
{
    uint8_t buffer[512];
    uint32_t *n_p, n;
    int i;

    /* Fetch the master index block and read the number of logs. */

    sdio_read_single_block(0, buffer);
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
                sdio_read_single_block(i / ENTRIES_PER_INDEX_BLOCK + 1, buffer);
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
    uint32_t entry[4], *n_p, n;
    int i, m, width;

    /* Fetch the master index block and read the number of logs. */

    sdio_read_single_block(0, buffer);
    sleep_while(sdio_is_busy());

    /* This is necessary so as not to break aliasing rules. */

    n_p = (uint32_t *)(buffer + 0);
    n = *n_p;

    for (i = 0 ; i <= n / ENTRIES_PER_INDEX_BLOCK ; i += 1) {
        if (key < ((uint32_t *)buffer)[i + 1]) {
            break;
        }
    }

    if (i == 0) {
        usbserial_printf("No such log.\n");
        return;
    }

    /* Fetch the appropriate index block. */

    sdio_read_single_block(i, buffer);
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

    width = sample_line_width(entry[1]);

    {
        uint8_t scratch[width];
        int j, m, spillover = 0;

        m = sample_data_blocks (entry[3], width);

        for (j = 0 ; j < m ; j += 1) {
            int k;

            /* Fetch a new block. */

            sdio_read_single_block(entry[2] + j, buffer);
            while(sdio_is_busy());

            if (spillover > 0) {
                float *line;

                memcpy (scratch + spillover, buffer, width - spillover);
                line = (float *)scratch;

                usbserial_printf("%+1.2f %+1.2f %+1.2f "
                                 "%+1.2f %+1.2f %+1.2f "
                                 "%+.2f\n",
                                 line[0], line[1], line[2],
                                 line[3], line[4], line[5],
                                 line[9]);
            }

            if (j == m - 1) {
                k = 512 - (m * 512 - entry[3] * width);
            } else {
                k = sizeof(buffer);
            }

            for (i = (width - spillover) % width;
                 i <= k - width;
                 i += width) {
                float *line;

                line = (float *)(buffer + i);
                usbserial_printf("%+1.2f %+1.2f %+1.2f "
                                 "%+1.2f %+1.2f %+1.2f "
                                 "%+.2f\n",
                                 line[0], line[1], line[2],
                                 line[3], line[4], line[5],
                                 line[9]);
            }

            if (j < m - 1) {
                spillover = sizeof(buffer) - i;
                assert (spillover < width);
            } else {
                spillover = 0;
            }

            if (spillover > 0) {
                memcpy (scratch, buffer + i, spillover);
            }
        }
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
