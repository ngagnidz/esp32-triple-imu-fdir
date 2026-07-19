#ifndef MPU_H
#define MPU_H

#define ACCEL_SENSITIVITY 4096.0f
#define GYRO_SENSITIVITY 65.5f
#define TEMP_SENSITIVITY 333.87f
#define TEMP_OFFSET 21.0f
#define GRAVITY 1.0f

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

void mpu_init(void);
void mpu_calibrate(int samples);
void mpu_read_all(burst_read_data_t out[3]);

#endif
