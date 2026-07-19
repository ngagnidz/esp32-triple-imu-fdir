#include "math.h"
#include "voting.h"

int mpu_fault[3] = {0, 0, 0};

const char *
return_result(voting_result result)
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
           fabs(data1.gyro_z - data2.gyro_z) > GYRO_THRESHHOLD;
}

voting_result voter(burst_read_data_t data1, burst_read_data_t data2, burst_read_data_t data3)
{

    bool d12 = disagree(data1, data2);
    bool d13 = disagree(data1, data3);
    bool d23 = disagree(data2, data3);

    // 1 2 disagrees
    // 1 3 disagrees
    // 2 3 agrees
    // 1 is outlier
    if (d12 && d13 && !d23)
    {
        mpu_fault[0] = 1;
        mpu_fault[1] = 0;
        mpu_fault[2] = 0;

        return MPU1_DISAGREE;
    }

    // 2 3 disagrees
    // 1 2 disagrees
    // 1 3 agrees
    // 2 is outlier
    if (d23 && d12 && !d13)
    {
        mpu_fault[0] = 0;
        mpu_fault[1] = 1;
        mpu_fault[2] = 0;
        return MPU2_DISAGREE;
    }

    // 2 3 disagrees
    // 1 3 disagrees
    // 1 2 agrees
    // 3 is outlier
    if (d23 && d13 && !d12)
    {
        mpu_fault[0] = 0;
        mpu_fault[1] = 0;
        mpu_fault[2] = 1;
        return MPU3_DISAGREE;
    }

    // 1 3 agrees
    // 1 2 disagrees
    // 2 3 disagrees

    // No one agrees
    if (!d13 && !d23 && !d12)
    {
        mpu_fault[0] = 0;
        mpu_fault[1] = 0;
        mpu_fault[2] = 0;
        return ALL_AGREE;
    }
    else
    {
        mpu_fault[0] = 1;
        mpu_fault[1] = 1;
        mpu_fault[2] = 1;

        return ALL_DISAGREE;
    }
}