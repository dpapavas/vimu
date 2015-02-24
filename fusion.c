#include <string.h>

#include "mk20dx128.h"
#include "util.h"
#include "fusion.h"
#include "sensors.h"

#define RING_SIZE 5
#define LINE_SIZE 10

static volatile fusion_data_ready_callback callback;
static int16_t lines[RING_SIZE][LINE_SIZE];
static uint8_t r = 0, w = 0;
static volatile uint8_t available;

static void sensor_data_ready(int16_t *line)
{
    assert(callback);
    assert (available < RING_SIZE);

    memcpy (lines[w], line, sizeof(lines[0]));

    available += 1;
    w = (w + 1) % RING_SIZE;

    pend_interrupt(45);
}

__attribute__((interrupt ("IRQ"))) void software_isr(void)
{
    int i;

    unpend_interrupt (45);
    assert(callback);

    for (i = 0 ; i < available ; i += 1) {
        float line[LINE_SIZE];
        int16_t *l;

        l = lines[r];

        /* Acceleration. */

        {
            static const float gain[] = {1.0 / 16384, 1.0 / 8192,
                                         1.0 / 4096, 1.0 / 2048};

            line[0] = (float)l[0] * gain[ACCEL_FSR];
            line[1] = (float)l[1] * gain[ACCEL_FSR];
            line[2] = (float)l[2] * gain[ACCEL_FSR];
        }

        /* Angular rate. */

        {
            static const float gain[] = {1.0 / 131, 1.0 / 65.5,
                                         1.0 / 32.8, 1.0 / 16.4};

            line[3] = (float)l[4] * gain[GYRO_FSR];
            line[4] = (float)l[5] * gain[GYRO_FSR];
            line[5] = (float)l[6] * gain[GYRO_FSR];
        }

        {
            line[6] = (float)l[7] * 1.0 / 16384;
            line[7] = (float)l[8] * 1.0 / 16384;
            line[8] = (float)l[9] * 1.0 / 16384;
        }

        line[9] = (float)l[3] / 340.0 + 36.53;

        callback(line);

        r = (r + 1) % RING_SIZE;
        available -= 1;
    }
}

void fusion_set_callback(fusion_data_ready_callback new)
{
    callback = new;

    if(new) {
        enable_interrupt(45);
        prioritize_interrupt(45, 3);

        sensors_set_callback(sensor_data_ready);
    } else {
        sensors_set_callback(NULL);
        disable_interrupt(45);
    }
}

int fusion_in_progress()
{
    return callback != NULL;
}
