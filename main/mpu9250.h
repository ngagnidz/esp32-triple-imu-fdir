#ifndef MPU_H
#define MPU_H
#include "esp_attr.h"
#include "driver/spi_master.h"

#define ACCEL_SENSITIVITY 4096.0f
#define GYRO_SENSITIVITY 65.5f
#define TEMP_SENSITIVITY 333.87f
#define TEMP_OFFSET 21.0f
#define CALIBRATION_SAMPLES 1000
#define GRAVITY 1.0f

#define PIN_SCLK 18
#define PIN_MOSI 23
#define PIN_MISO 19
#define PIN_CS1 5
#define PIN_CS2 4
#define PIN_CS3 15

typedef struct
{
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} correction_data_t;

typedef struct
{
    float accel_x;
    float accel_y;
    float accel_z;
    float temp;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} burst_read_data_t;

burst_read_data_t burst_read(correction_data_t corr, spi_device_handle_t dev, int idx);

#endif