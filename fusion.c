#include <string.h>

#include "mk20dx128.h"
#include "util.h"
#include "fusion.h"
#include "sensors.h"

#define RING_SIZE 5
#define LINE_SIZE 10

static fusion_data_ready_callback callback;
static float lines[RING_SIZE][LINE_SIZE];
static uint8_t r = 0, w = 0;
static volatile uint8_t fetched;

static void sensor_data_ready(float *line)
{
    assert(callback);
    assert (fetched < RING_SIZE);

    memcpy (lines[w], line, sizeof(lines[0]));

    fetched += 1;
    w = (w + 1) % RING_SIZE;
}

void fusion_set_callback(fusion_data_ready_callback new)
{
    callback = new;

    if(new) {
        sensors_set_callback(sensor_data_ready);
    } else {
        sensors_set_callback(NULL);
    }
}

void fusion_start()
{
    float sums[LINE_SIZE];

    while (1) {
        float *line;
        int i, n;

        memset (sums, 0, sizeof(sums));

        for (n = 0 ; n < 100 ; n += 1) {
            sleep_while(fetched < 1);

            line = lines[r];

            delay_us(800);
            for (i = 0 ; i < LINE_SIZE ; i += 1) {
                sums[i] += line[i];
            }

            r = (r + 1) % RING_SIZE;
            fetched -= 1;
        }

        for (i = 0 ; i < LINE_SIZE ; i += 1) {
            sums[i] /= n;
        }

        callback((uint8_t *)sums);
    }
}
