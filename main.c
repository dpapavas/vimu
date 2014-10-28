#define _XOPEN_SOURCE

#include <string.h>

#include "mk20dx128.h"
#include "usbcommon.h"
#include "usbserial.h"
#include "sensors.h"
#include "fusion.h"
#include "console.h"
#include "button.h"
#include "sdio.h"
#include "log.h"
#include "util.h"
#include "i2c.h"

static void callback()
{
    log_toggle();
}

int main()
{
    usb_initialize();
    sdio_initialize();
    i2c_initialize();
    button_initialize();

    button_set_callback(callback);

    while(1) {
         if (usbserial_is_rts()) {
            console_cycle();
        }
    }

    return 0;
}
