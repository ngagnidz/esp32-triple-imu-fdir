#include "fusion.h"
#include "stdio.h"

bool average_good(const burst_read_data_t readings[3], voting_result result, burst_read_data_t *out)
{

    burst_read_data_t avg = {0};
    float n = 0;

    if (result == ALL_DISAGREE)
    {
        return false;
    }

    for (int i = 0; i < 3; i++)
    {
        // MPU1_DISAGREE = 0
        if (result == MPU1_DISAGREE + i)
        {
            continue;
        }
        avg.accel_x += readings[i].accel_x;
        avg.accel_y += readings[i].accel_y;
        avg.accel_z += readings[i].accel_z;
        avg.gyro_x += readings[i].gyro_x;
        avg.gyro_y += readings[i].gyro_y;
        avg.gyro_z += readings[i].gyro_z;
        n++;
    }
    avg.accel_x /= n;
    avg.accel_y /= n;
    avg.accel_z /= n;
    avg.gyro_x /= n;
    avg.gyro_y /= n;
    avg.gyro_z /= n;

    *out = avg;

    return true;
}