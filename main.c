#define _XOPEN_SOURCE

#include "mk20dx128.h"
#include "usbserial.h"
#include "sensors.h"
#include "util.h"

int main()
{
    usb_await_enumeration();
    usbserial_await_dtr();

    while(1) {
        usbserial_await_rts();
        power_sensors_up();

        while(usbserial_is_rts()) {
            int16_t *line;

            /* if (i2c_busy()) { */
            /*     turn_on_led(); */
            /* } else { */
            /*     turn_off_led(); */
            /* } */

            {
                static int j;

                if ((j % 100) == 0) {
                    turn_on_led();
                } else {
                    turn_off_led();
                }

                j += 1;
            }

            while((line = read_sensor_values()) == NULL);

            usbserial_trace("%hd, %hd, %hd, %hd, %hd, %hd, %hd, %hd, %hd\n",
                            line[0], line[1], line[2],
                            line[3], line[4], line[5],
                            line[6], line[7], line[8]);

            /* usbserial_trace("p = %d, t = %d\n", pending, transferred); */
        }

        power_sensors_down();
    }


    return 0;
}
