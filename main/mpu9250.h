#include <stdint.h>

// Each register returns 8 bit numbers. Each returned value is 16 bits long. High bit and low bit.
struct mpu_output
{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
};

void mpu_init(int cs_pin);

struct mpu_output mpu_read(int cs_pin);