#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H


struct SensorData {
    volatile float sum            = 0.0;
    volatile float value          = 0.0;
    volatile float filtered_value = 0.0;
    volatile float buff_raw [4]   = {0.0, 0.0, 0.0, 0.0};
    volatile float buff_filt[4]   = {0.0, 0.0, 0.0, 0.0};
};

#endif