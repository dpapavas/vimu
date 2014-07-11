#define _XOPEN_SOURCE

#include "mk20dx128.h"
#include "usbcommon.h"
#include "usbserial.h"
#include "sensors.h"
#include "sdio.h"
#include "log.h"
#include "util.h"
#include "i2c.h"

#define FOO 5

int main()
{
    usb_initialize();
    sdio_initialize();
    i2c_initialize();

    usbserial_await_rts();
    usbserial_set_state(SERIAL_STATE_DSR);

    console_enter();

    /* while(1) { */
    /*     usb_write("tick\n", 5, 0); */
    /*     log_begin(0); */
    /*     delay_ms(1000); */
    /*     usb_write("tack\n", 5, 0); */
    /*     log_end(); */
    /*     delay_ms(1000); */
    /* } */

#if FOO == 0
#elif FOO == 1
#elif FOO == 2
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
#elif FOO == 3
    sdio_initialize();
    usbserial_await_rts();
    log_list();
#endif

    return 0;
}
