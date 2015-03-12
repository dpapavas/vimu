#include <string.h>
#include <math.h>

#include "mk20dx128.h"
#include "util.h"
#include "usbcommon.h"
#include "sdio.h"
#include "fusion.h"
#include "calibration.h"

#define SAMPLING_RATE 1000
#define CONFIGURATION_DATA_BLOCK 0
#define GYROSCOPE_DATA_OFFSET 0

typedef struct {
    volatile int finished;

    float means[4], sums[8];
    uint32_t samples;
} OffsetsContext;

/* static int wait_for_it(float *samples, void *userdata) */
/* { */
/*     OffsetsContext *context = (OffsetsContext *)userdata; */
/*     float x, delta; */

/*     /\* x = sqrtf(samples[0] * samples[0] + *\/ */
/*     /\*           samples[1] * samples[1] + *\/ */
/*     /\*           samples[2] * samples[2]); *\/ */

/*     /\* delta = x - context->mean; *\/ */

/*     /\* context->mean += delta / (context->samples += 1); *\/ */
/*     /\* context->variance += delta * (x - context->mean); *\/ */

/*     /\* if (context->samples == 5000) { *\/ */
/*     /\*     usbserial_trace("%f, %f\n", *\/ */
/*     /\*                     context->mean, *\/ */
/*     /\*                     context->variance / context->samples); *\/ */

/*     /\*     context->mean = context->variance = context->samples = 0; *\/ */
/*     /\* } *\/ */

/*     x = samples[3]; */

/*     delta = x - context->mean; */

/*     context->mean += delta / (context->samples += 1); */
/*     context->variance += delta * (x - context->mean); */

/*     if (context->samples == 5000) { */
/*         usbserial_trace("%f, %f\n", */
/*                         context->mean, */
/*                         context->variance / context->samples); */

/*         context->mean = context->variance = context->samples = 0; */
/*     } */

/*     return 1; */
/* } */

static void calculate_least_squares_fit(OffsetsContext *context,
                                        float (*lines)[2],
                                        float *correlations,
                                        float *deviations)
{
    float ss_xx;
    int i;

    ss_xx = (context->sums[0] -
             context->samples *
             context->means[0] * context->means[0]);

    usbserial_trace("sum_xx = %f, ss_xx = %f\n", context->sums[0], ss_xx);

    for (i = 0 ; i < 3 ; i += 1) {
        float ss_yy = (context->sums[1 + i] -
                       context->samples *
                       context->means[1 + i] * context->means[1 + i]);
        float ss_xy = (context->sums[4 + i] -
                       context->samples *
                       context->means[0] * context->means[1 + i]);

        usbserial_trace("%d: n = %d\nsum_yy = %f, sum_xy = %f\nss_yy = %f, ss_xy = %f\nm_x = %f, m_y = %f\n", i, context->samples, context->sums[1 + i], context->sums[4 + i], ss_yy, ss_xy, context->means[0], context->means[1 + i]);

        if (lines) {
            lines[i][1] = ss_xy / ss_xx;
            lines[i][0] = (context->means[1 + i] -
                           lines[i][1] * context->means[0]);
        }

        if (correlations) {
            correlations[i] = (ss_xy * ss_xy) / (ss_xx * ss_yy);
        }

        if (deviations) {
            deviations[i] = sqrtf((ss_yy - ss_xy * ss_xy / ss_xx) /
                                  (context->samples - 2));
        }
    }
}

static int accumulate_samples_for_least_squares(float *samples, void *userdata)
{
    OffsetsContext *context = (OffsetsContext *)userdata;

    context->samples += 1;

    /* Means. */

    context->means[0] += (samples[3] - context->means[0]) / context->samples;
    context->means[1] += (samples[0] - context->means[1]) / context->samples;
    context->means[2] += (samples[1] - context->means[2]) / context->samples;
    context->means[3] += (samples[2] - context->means[3]) / context->samples;

    /* Squared sums for variances. */

    context->sums[0] += samples[3] * samples[3];
    context->sums[1] += samples[0] * samples[0];
    context->sums[2] += samples[1] * samples[1];
    context->sums[3] += samples[2] * samples[2];

    /* Squared sums for cross-covariances. */

    context->sums[4] += samples[3] * samples[0];
    context->sums[5] += samples[3] * samples[1];
    context->sums[6] += samples[3] * samples[2];

    context->sums[7] += context->samples * samples[3];

    if (context->samples % 5000 == 0) {
        float t, ss_tt, ss_xt, Tdot;

        t = (float)context->samples / SAMPLING_RATE;

        ss_tt = t * (context->samples * t - 1) / 12.0;
        ss_xt = (context->sums[7] / SAMPLING_RATE -
                 ((context->samples + 1) * t *
                  context->means[0] / 2));

        Tdot = ss_xt / ss_tt;

        if ((context->samples / 5000) % 2 == 0) {
            float rsquared[3];

            calculate_least_squares_fit (context, NULL, rsquared, NULL);

            usbserial_printf("T = %.2f, Tdot = %.1f, r^2 = %.3f, %.3f, %.3f\n",
                             samples[3], Tdot * 60,
                             rsquared[0], rsquared[1], rsquared[2]);
        } else {
            float s[3];

            calculate_least_squares_fit (context, NULL, NULL, s);

            usbserial_printf("T = %.2f, Tdot = %.1f, s = %.3f, %.3f, %.3f\n",
                             samples[3], Tdot * 60,
                             s[0], s[1], s[2]);
        }
    }

    return !context->finished;
}

static void break_sent (uint16_t duration, void *userdata)
{
    OffsetsContext *context = (OffsetsContext *)userdata;

    context->finished = 1;
}

static void load_gyroscope_calibration(OffsetsContext *context)
{
    uint8_t buffer[512];

    sdio_read_single_block(CONFIGURATION_DATA_BLOCK, buffer);
    sleep_while(sdio_is_busy());

    memcpy(&context->samples,
           buffer + GYROSCOPE_DATA_OFFSET,
           sizeof(context->samples));

    memcpy(&context->means,
           buffer + GYROSCOPE_DATA_OFFSET + sizeof(context->samples),
           sizeof(context->means));

    memcpy(&context->sums,
           buffer + GYROSCOPE_DATA_OFFSET +
           sizeof(context->samples) + sizeof(context->means),
           sizeof(context->sums));
}

static void save_gyroscope_calibration(OffsetsContext *context)
{
    uint8_t buffer[512];

    sdio_read_single_block(CONFIGURATION_DATA_BLOCK, buffer);
    sleep_while(sdio_is_busy());

    memcpy(buffer + GYROSCOPE_DATA_OFFSET,
           &context->samples,
           sizeof(context->samples));

    memcpy(buffer + GYROSCOPE_DATA_OFFSET + sizeof(context->samples),
           &context->means,
           sizeof(context->means));

    memcpy(buffer + GYROSCOPE_DATA_OFFSET +
           sizeof(context->samples) + sizeof(context->means),
           &context->sums,
           sizeof(context->sums));

    sdio_write_single_block(CONFIGURATION_DATA_BLOCK, buffer);
    sleep_while(sdio_is_busy());
}

void calibration_fit_gyroscope_offsets(calibration_Mode mode)
{
    OffsetsContext context;

    if (mode != CALIBRATION_RESET) {
        load_gyroscope_calibration (&context);
        context.finished = 0;
    } else {
        memset(&context, 0, sizeof(context));
    }

    if (mode != CALIBRATION_REPORT) {
        float l[3][2];

        usbserial_printf("Starting gyroscope offset calibration.\n");
        usb_set_send_break_callback(break_sent, &context);

        fusion_start((1 << FUSION_RAW_ANGULAR_SPEED) |
                     (1 << FUSION_SENSOR_TEMPERATURE),
                     SAMPLING_RATE,
                     accumulate_samples_for_least_squares, &context);

        usb_set_send_break_callback(NULL, NULL);
        usbserial_printf("Done.\n");

        save_gyroscope_calibration(&context);
        calculate_least_squares_fit(&context, l, NULL, NULL);
        fusion_set_gyroscope_offsets(l);
    } else {
        float l[3][2], s[3], rsquared[3];
        int i;

        calculate_least_squares_fit(&context, l, rsquared, s);

        for (i = 0 ; i < 3 ; i += 1) {
            usbserial_printf("%d: a = %.3f, b = %.3f, r^2 = %.13f, s = %.13f\n",
                             i + 1, l[i][0], l[i][1], rsquared[i], s[i]);
        }
    }
}

void calibration_initialize()
{
    OffsetsContext context;
    float l[3][2];

    /* Load and set gyroscope offset lines. */

    load_gyroscope_calibration (&context);
    calculate_least_squares_fit(&context, l, NULL, NULL);
    fusion_set_gyroscope_offsets(l);
}

#include <float.h>

static float noise(float mu, float sigma)
{
    static float z_0, z_1;
    static int generate;
    float u_1, u_2;

    generate = !generate;

    if (!generate) {
        return z_1 * sigma + mu;
    }

    do {
        u_1 = lcg_rand() * (1.0 / LCG_RAND_MAX);
        u_2 = lcg_rand() * (1.0 / LCG_RAND_MAX);
    } while (u_1 <= FLT_MIN);

    z_0 = sqrtf(-2.0 * logf(u_1)) * cosf(2 * M_PI * u_2);
    z_1 = sqrtf(-2.0 * logf(u_1)) * sinf(2 * M_PI * u_2);

    return z_0 * sigma + mu;
}

void calibration_foo()
{
    OffsetsContext context = {
        .finished = 0,
        .samples = 0,
        .means = {0, 0, 0, 0},
        .sums = {0, 0, 0, 0, 0, 0, 0, 0}
    };

    float l[3][2], s[3], rsquared[3];
    int i;

    for (i = 0 ; i < 50000 ; i += 1) {
        float t = i / 1000.0 / 60.0;
        float samples[4] = {1 + (2 * t) + noise(0, 5.2),
                            3 + (4 * t) + noise(0, 5.4),
                            5 + (6 * t) + noise(0, 5.6),
                            t};

        accumulate_samples_for_least_squares(samples, &context);
    }

    calculate_least_squares_fit(&context, l, rsquared, s);

    for (i = 0 ; i < 3 ; i += 1) {
        usbserial_printf("%d: a = %.3f, b = %.3f, r^2 = %.10f, s = %.10f\n",
                         i + 1, l[i][0], l[i][1], rsquared[i], s[i]);
    }
}
