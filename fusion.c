#include <string.h>

#include "mk20dx128.h"
#include "util.h"
#include "fusion.h"
#include "sensors.h"

#define RING_SIZE 2

static struct {
    fusion_DataReadyCallback callback;
    int16_t lines[RING_SIZE][SENSORS_SAMPLES_PER_LINE];
    float *line;
    uint32_t data;
    volatile uint16_t period;
    uint16_t count;
    uint8_t read, write, width;
    volatile uint8_t available;
} context;

static void sensor_data_ready(int16_t *line)
{
    if (context.period == 0) {
        return;
    }

    assert (context.callback);
    assert (context.available < RING_SIZE);

    memcpy (context.lines[context.write], line, sizeof(context.lines[0]));

    context.available += 1;
    context.write = (context.write + 1) % RING_SIZE;

    pend_software_interrupt();
}

static void fuse_sensor_data()
{
    int i;

    unpend_software_interrupt();

    /* Check if we're shutting down. */

    if (context.period == 0) {
        return;
    }

    assert (context.callback);
    assert (context.count < context.period);

    for (i = 0 ; i < context.available ; i += 1) {
        int16_t *l;
        int j = 0;

        l = context.lines[context.read];

        /* Acceleration. */

        if (context.data & (1 << FUSION_RAW_ACCELERATION)) {
            static const float gain[] = {1.0 / 16384, 1.0 / 8192,
                                         1.0 / 4096, 1.0 / 2048};

            context.line[j + 0] += (float)l[0] * gain[ACCEL_FSR];
            context.line[j + 1] += (float)l[1] * gain[ACCEL_FSR];
            context.line[j + 2] += (float)l[2] * gain[ACCEL_FSR];

            j += 3;
        }

        /* Angular rate. */

        if (context.data & (1 << FUSION_RAW_ANGULAR_SPEED)) {
            static const float gain[] = {1.0 / 131, 1.0 / 65.5,
                                         1.0 / 32.8, 1.0 / 16.4};

            context.line[j + 0] += (float)l[4] * gain[GYRO_FSR];
            context.line[j + 1] += (float)l[5] * gain[GYRO_FSR];
            context.line[j + 2] += (float)l[6] * gain[GYRO_FSR];

            j += 3;
        }

        /* Magnetic field. */

        if (context.data & (1 << FUSION_RAW_MAGNETIC_FIELD)) {
            context.line[j + 0] += (float)l[7] * 1.0 / 16384;
            context.line[j + 1] += (float)l[8] * 1.0 / 16384;
            context.line[j + 2] += (float)l[9] * 1.0 / 16384;

            j += 3;
        }

        /* Sensor temperature. */

        if (context.data & (1 << FUSION_SENSOR_TEMPERATURE)) {
            context.line[j] += (float)l[3] / 340.0 + 36.53;

            j += 1;
        }

        if ((context.count += 1) == context.period) {
            if (context.period > 1) {
                int k;

                for (k = 0 ; k < j ; k += 1) {
                    context.line[k] /= context.period;
                }
            }

            if (!context.callback(context.line)) {
                context.period = 0;
            } else {
                memset(context.line, 0, context.width * sizeof(float));
                context.count = 0;
            }
        }

        context.read = (context.read + 1) % RING_SIZE;
        context.available -= 1;
    }
}

int fusion_samples_per_line(uint32_t data)
{
    int i, n, widths[] = {
        [FUSION_RAW_ACCELERATION] = 3,
        [FUSION_RAW_ANGULAR_SPEED] = 3,
        [FUSION_RAW_MAGNETIC_FIELD] = 3,
        [FUSION_SENSOR_TEMPERATURE] = 1
    };

    for (i = 0, n = 0 ; i < sizeof(widths) / sizeof(widths[0]) ; i += 1) {
        if (data & (1 << i)) {
            n += widths[i];
        }
    }

    return n;
}

void fusion_start(uint32_t data, int rate, fusion_DataReadyCallback callback)
{
    int w = fusion_samples_per_line(data);
    float line[w];

    /* Initialize the parameters. */

    context.read = context.write = context.count = 0;
    context.callback = callback;
    context.data = data;
    context.width = w;
    context.line = memset(line, 0, sizeof(line));

    if (rate >= 0 && rate <= 1000) {
        context.period = 1000 / rate;
    } else {
        context.period = 1;
    }

    /* Enable the interrupts and start fusing. */

    set_software_isr_callback(fuse_sensor_data);
    prioritize_software_interrupt(3);

    sensors_set_callback(sensor_data_ready);
    sleep_while (context.period > 0);

    sensors_set_callback(NULL);
    set_software_isr_callback(NULL);
}

int fusion_in_progress()
{
    return context.period > 0;
}
