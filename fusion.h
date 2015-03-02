#ifndef _FUSION_H_
#define _FUSION_H_

typedef enum {
    FUSION_RAW_ACCELERATION,
    FUSION_RAW_ANGULAR_SPEED,
    FUSION_RAW_MAGNETIC_FIELD,

    FUSION_SENSOR_TEMPERATURE,

    FUSION_CALIBRATED_ACCELERATION,
    FUSION_CALIBRATED_ANGULAR_VELOCITY,
    FUSION_CALIBRATED_MAGNETIC_FIELD,

    FUSION_COURSE_OVER_GROUND,
    FUSION_SPEED_OVER_GROUND,
    FUSION_GPS_MODE,
    FUSION_LONGITUDE,
    FUSION_LATITUDE,
    FUSION_UTC_TIME,
    FUSION_DOP,
    FUSION_ALTITUDE,
} fusion_Datum;

typedef int (*fusion_DataReadyCallback)(float *samples);

void fusion_start(uint32_t data, int rate, fusion_DataReadyCallback callback);
int fusion_samples_per_line(uint32_t data);
int fusion_in_progress();

#endif
