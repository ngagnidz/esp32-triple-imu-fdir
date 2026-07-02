#include "math.h"
#include "mpu9250.h"
#include "voting.h"

const char *return_result(voting_result result)
{
    switch (result)
    {
    case ALL_AGREE:
        return "ALL AGREE";
    case MPU1_DISAGREE:
        return "MPU1 DISAGREE";
    case MPU2_DISAGREE:
        return "MPU2 DISAGREE";
    case MPU3_DISAGREE:
        return "MPU3 DISAGREE";
    case ALL_DISAGREE:
        return "ALL_DISAGREE";
    default:
        return "UNKNOWN_STATE";
    };
}

bool disagree(burst_read_data_t data1, burst_read_data_t data2)
{
    return fabs(data1.accel_x - data2.accel_x) >
               ACCEL_THRESHHOLD ||
           fabs(data1.accel_y - data2.accel_y) >
               ACCEL_THRESHHOLD ||
           fabs(data1.accel_z - data2.accel_z) >
               ACCEL_THRESHHOLD ||
           fabs(data1.gyro_x - data2.gyro_x) > GYRO_THRESHHOLD ||
           fabs(data1.gyro_y - data2.gyro_y) > GYRO_THRESHHOLD ||
           fabs(data1.gyro_z - data2.gyro_z) > GYRO_THRESHHOLD ||
           fabs(data1.temp - data2.temp) > TEMP_THRESHHOLD;
}

voting_result voter(burst_read_data_t data1, burst_read_data_t data2, burst_read_data_t data3)
{

    bool d12 = disagree(data1, data2);
    bool d13 = disagree(data1, data3);
    bool d23 = disagree(data2, data3);

    int disagreements[3] = {
        d12 + d13,
        d23 + d12,
        d23 + d13,
    };

    if (disagreements[0] == 2)
        return MPU1_DISAGREE;
    if (disagreements[1] == 2)
        return MPU2_DISAGREE;
    if (disagreements[2] == 2)
        return MPU3_DISAGREE;
    if (disagreements[0] && disagreements[1] && disagreements[2])
        return ALL_DISAGREE;
    return ALL_AGREE;
}