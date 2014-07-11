#define _XOPEN_SOURCE

#include <string.h>

#include "mk20dx128.h"
#include "usbserial.h"
#include "sensors.h"
#include "sdio.h"
#include "util.h"
#include "i2c.h"

int main()
{
#if 0
    uint8_t buffers[2][512], *buffer;
    int16_t scratch[10];
    unsigned int i, n;

    sdio_initialize();
    usbserial_await_rts();
    power_sensors_up();

    for (i = 0, n = 0, buffer = buffers[0];
         usbserial_is_rts();
         i += 1, buffer = buffers[i % 2]) {
        int16_t (*line)[10];

        /* If there's left-over data from the last line copy it to the
         * start of the buffer. */

        if (n > 0) {
            memcpy(buffer, (uint8_t *)scratch + n, sizeof(scratch) - n);
            n = sizeof(scratch) - n;
        }

        /* Write entire lines to the buffer. */

        for (line = (int16_t (*)[10])(buffer + n);
             (uint8_t *)(line + 1) <= (buffer + 512);
             line += 1) {
            read_sensor_values(*line);
        }

        /* If there's still space left over in the buffer, write the
         * beginning of the next line. */

        n = buffer + 512 - (uint8_t *)line;

        if (n > 0) {
            read_sensor_values(scratch);
            memcpy(line, scratch, n);
        }

        /* Flush the buffer into the SD card.  */

        sleep_while(sdio_is_busy());
        sdio_write_single_block(i, buffer);
        toggle_led();
    }

    power_sensors_down();
#else
    int i, n;
    int16_t scratch[10];

    sdio_initialize();
    usbserial_await_rts();

    for(i = 0, n = 0 ; usbserial_is_rts() ; i += 1) {
        int16_t (*line)[10];
        uint8_t buffer[512];

        sdio_read_single_block(i, buffer);
        sleep_while(sdio_is_busy());

        if (n > 0) {
            memcpy((uint8_t *)scratch + n, buffer, sizeof(scratch) - n);
            n = sizeof(scratch) - n;

            line = &scratch;
            usbserial_printf("%+5hd, %+5hd, %+5hd, "
                             "%+5hd, %+5hd, %+5hd, "
                             "%+5hd, %+5hd, %+5hd\n",
                             (*line)[0], (*line)[1], (*line)[2],
                             (*line)[4], (*line)[5], (*line)[6],
                             (*line)[7], (*line)[8], (*line)[9]);
        }

        for (line = (int16_t (*)[10])(buffer + n);
             (uint8_t *)(line + 1) <= (buffer + 512);
             line += 1) {
            usbserial_printf("%+5hd, %+5hd, %+5hd, "
                             "%+5hd, %+5hd, %+5hd, "
                             "%+5hd, %+5hd, %+5hd\n",
                             (*line)[0], (*line)[1], (*line)[2],
                             (*line)[4], (*line)[5], (*line)[6],
                             (*line)[7], (*line)[8], (*line)[9]);
        }

        n = buffer + 512 - (uint8_t *)line;

        if (n > 0) {
            memcpy(scratch, line, n);
        }
    }
#endif

    return 0;
}
