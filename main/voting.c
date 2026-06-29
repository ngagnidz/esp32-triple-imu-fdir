#include "math.h"
#include "mpu9250.h"
#include "voting.h"

const char *get_state_string(voting_result result)
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

voting_result voter(burst_read_data_t data1, burst_read_data_t data2, burst_read_data_t data3)
{

    int tracker[3] = {0, 0, 0};

    bool one_two_disagree =
        fabs(data1.accel_x - data2.accel_x) >
            ACCEL_THRESHHOLD ||
        fabs(data1.accel_y - data2.accel_y) >
            ACCEL_THRESHHOLD ||
        fabs(data1.accel_z - data2.accel_z) >
            ACCEL_THRESHHOLD ||
        fabs(data1.gyro_x - data2.gyro_x) > GYRO_THRESHHOLD ||
        fabs(data1.gyro_y - data2.gyro_y) > GYRO_THRESHHOLD ||
        fabs(data1.gyro_z - data2.gyro_z) > GYRO_THRESHHOLD ||
        fabs(data1.temp - data2.temp) > GYRO_THRESHHOLD;

    if (one_two_disagree)
    {
        tracker[0]++;
        tracker[1]++;
    }

    bool two_three_disagree =
        fabs(data2.accel_x - data3.accel_x) >
            ACCEL_THRESHHOLD ||
        fabs(data2.accel_y - data3.accel_y) >
            ACCEL_THRESHHOLD ||
        fabs(data2.accel_z - data3.accel_z) >
            ACCEL_THRESHHOLD ||
        fabs(data2.gyro_x - data3.gyro_x) > GYRO_THRESHHOLD ||
        fabs(data2.gyro_y - data3.gyro_y) > GYRO_THRESHHOLD ||
        fabs(data2.gyro_z - data3.gyro_z) > GYRO_THRESHHOLD ||
        fabs(data2.temp - data3.temp) > GYRO_THRESHHOLD;
    if (two_three_disagree)
    {
        tracker[1]++;
        tracker[2]++;
    }

    bool one_three_disagree =
        fabs(data1.accel_x - data3.accel_x) >
            ACCEL_THRESHHOLD ||
        fabs(data1.accel_y - data3.accel_y) >
            ACCEL_THRESHHOLD ||
        fabs(data1.accel_z - data3.accel_z) >
            ACCEL_THRESHHOLD ||
        fabs(data1.gyro_x - data3.gyro_x) > GYRO_THRESHHOLD ||
        fabs(data1.gyro_y - data3.gyro_y) > GYRO_THRESHHOLD ||
        fabs(data1.gyro_z - data3.gyro_z) > GYRO_THRESHHOLD ||
        fabs(data1.temp - data3.temp) > GYRO_THRESHHOLD;

    if (one_three_disagree)
    {
        tracker[0]++;
        tracker[2]++;
    }

    if (tracker[0] == 2 && tracker[1] == 2 && tracker[2] == 2)
    {
        return ALL_DISAGREE;
    }

    if (tracker[2] == 2)
    {
        return MPU3_DISAGREE;
    }

    if (tracker[0] == 2)
    {
        return MPU1_DISAGREE;
    }

    if (tracker[1] == 2)
    {
        return MPU2_DISAGREE;
    }

    return ALL_AGREE;
}