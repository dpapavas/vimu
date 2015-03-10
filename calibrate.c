#include "mk20dx128.h"
#include "util.h"
#include "usbcommon.h"
#include "fusion.h"

#define GYROSCOPE_CALIBRATION_OFFSET 0

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

#define R 1000
#define T 5000

static int regression(float *samples, void *userdata)
{
    OffsetsContext *context = (OffsetsContext *)userdata;

    context->samples += 1;

    /* Means. */

    context->means[0] += (samples[3] - context->means[0]) / context->samples;
    context->means[1] += (samples[0] - context->means[1]) / context->samples;
    context->means[2] += (samples[1] - context->means[2]) / context->samples;
    context->means[3] += (samples[2] - context->means[3]) / context->samples;

    /* sigma_xx, sigma_yy */

    context->sums[0] += samples[3] * samples[3];
    context->sums[1] += samples[0] * samples[0];
    context->sums[2] += samples[1] * samples[1];
    context->sums[3] += samples[2] * samples[2];

    /* sigma_xy */

    context->sums[4] += samples[3] * samples[0];
    context->sums[5] += samples[3] * samples[1];
    context->sums[6] += samples[3] * samples[2];

    context->sums[7] += context->samples * samples[3];

    if (context->samples % 5000 == 0) {
        float t, ss_tt, ss_xx, ss_xt, Tdot, rsquared[3];
        int i;

        t = (float)context->samples / R;

        ss_tt = t * (context->samples * t - 1) / 12.0;
        ss_xt = context->sums[7] / R - ((context->samples + 1) * t *
                                        context->means[0] / 2);
        ss_xx = (context->sums[0] -
                 context->samples *
                 context->means[0] * context->means[0]);

        Tdot = ss_xt / ss_tt;

        for (i = 0 ; i < 3 ; i += 1) {
            float a, b;
            float ss_yy = (context->sums[1 + i] -
                           context->samples *
                           context->means[1 + i] * context->means[1 + i]);
            float ss_xy = (context->sums[4 + i] -
                           context->samples *
                           context->means[0] * context->means[1 + i]);

            b = ss_xy / ss_xx;
            a = context->means[1 + i] - b * context->means[0];
            rsquared[i] = (ss_xy * ss_xy) / (ss_xx * ss_yy);

            /* usbserial_trace("%d: %f, %f, %f\n", i + 1, a, b, rsquared); */
        }

        usbserial_printf("T = %.2f, Tdot = %.1f %.3f, %.3f, %.3f\n",
                         samples[3], Tdot * 60,
                         rsquared[0], rsquared[1], rsquared[2]);
    }

    return !context->finished;
}

static void break_sent (uint16_t duration, void *userdata)
{
    OffsetsContext *context = (OffsetsContext *)userdata;

    context->finished = 1;
}

void calibrate_gyroscope_offsets()
{
    OffsetsContext context = {
        .finished = 0,
        .samples = 0,
        .means = {0, 0, 0, 0},
        .sums = {0, 0, 0, 0, 0, 0, 0, 0}
    };

    usb_set_send_break_callback(break_sent, &context);

    /* int i; */
    /* for (i = 0 ; i < 10000 ; i += 1) { */
    /*     float t = .444 + (.555 * i / 1000.0); */
    /*     float samples[4] = {.111 + (.111 * t), */
    /*                         .222 + (.222 * t), */
    /*                         .333 + (.333 * t), */
    /*                         t}; */

    /*     regression(samples, &context); */
    /* } */

    fusion_start((1 << FUSION_RAW_ANGULAR_SPEED) |
                 (1 << FUSION_SENSOR_TEMPERATURE),
                 1000, regression, &context);

    usb_set_send_break_callback(NULL, NULL);
}
