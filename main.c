#define _XOPEN_SOURCE

#include <string.h>

#include "mk20dx128.h"
#include "usbcommon.h"
#include "usbserial.h"
#include "sensors.h"
#include "fusion.h"
#include "console.h"
#include "button.h"
#include "calibration.h"
#include "sdio.h"
#include "log.h"
#include "util.h"
#include "i2c.h"

int main()
{
    usb_initialize();
    usbserial_initialize();
    usbserial_await_rts();
    sdio_initialize();
    i2c_initialize();
    button_initialize();
    calibration_initialize();
    console_initialize();
    console_enter();

    return 0;
}
