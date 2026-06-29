#include "voting.h"
typedef enum
{
    NOMINAL,
    DEGRADED,
    FAILED,
} system_condition;

typedef enum
{
    MPU1,
    MPU2,
    MPU3,
} faulty;

system_condition status = NOMINAL;
faulty faulty_mpu;

const char *get_status(system_condition status)
{
    switch (status)
    {
    case NOMINAL:
        return "NOMINAL";
        break;
    case DEGRADED:
        return "DEGRADED";
        break;
    case FAILED:
        return "FAILED";
        break;
    default:
        return "NOMINAL";
        break;
    };
}

system_condition update_state(voting_result result)
{
    switch (result)
    {
    case MPU1_DISAGREE:
        if (status == DEGRADED && faulty_mpu != MPU1)
            status = FAILED;
        else
        {
            status = DEGRADED;
        }
        faulty_mpu = MPU1;
        break;

    case MPU2_DISAGREE:
        if (status == DEGRADED && faulty_mpu != MPU2)
            status = FAILED;
        else
        {
            status = DEGRADED;
        }
        faulty_mpu = MPU1;
        break;

    case MPU3_DISAGREE:
        if (status == DEGRADED && faulty_mpu != MPU3)
            status = FAILED;
        else
        {
            status = DEGRADED;
        }
        faulty_mpu = MPU1;

        break;

    case ALL_AGREE:
        status = NOMINAL;
        break;

    default:
        status = FAILED;
        break;
    }

    return status;
}