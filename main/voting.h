#ifndef VOTING_H
#define VOTING_H
#include "mpu9250.h"

#define ACCEL_THRESHHOLD 1
#define GYRO_THRESHHOLD 5

typedef enum
{
    ALL_AGREE,
    MPU1_DISAGREE,
    MPU2_DISAGREE,
    MPU3_DISAGREE,
    ALL_DISAGREE
} voting_result;
voting_result voter(burst_read_data_t data1, burst_read_data_t data2, burst_read_data_t data3);
const char *get_state_string(voting_result result);

#endif