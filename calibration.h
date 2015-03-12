#ifndef _CALIBRATION_H_
#define _CALIBRATION_H_

typedef enum {
    CALIBRATION_CONTINUE,
    CALIBRATION_RESET,
    CALIBRATION_REPORT
} calibration_Mode;

void calibration_initialize();
void calibration_fit_gyroscope_offsets(calibration_Mode mode);

#endif
