#include "voting.h"
#include "mpu9250.h"

// States
typedef enum
{
    NOMINAL,
    DEGRADED,
    FAILED,
} system_condition;

// MPUs
typedef enum
{
    MPU1,
    MPU2,
    MPU3,
    MPU_NONE,
} faulty;

static system_condition status;
static faulty faulty_mpu = MPU_NONE;
static int clean_streak = 0;

system_condition
update_state(voting_result result)
{
    if (status == FAILED && result == ALL_AGREE)
    {
        printf("%d", clean_streak);
        clean_streak++;
        if (clean_streak >= ALL_AGREE_N)
        {
            status = NOMINAL;
            faulty_mpu = MPU_NONE;
            clean_streak = 0;
            return status;
        }
    }

    if (status == FAILED)
    {
        return status;
    }

    switch (result)
    {

    case MPU1_DISAGREE:
        clean_streak = 0;

        if (status == DEGRADED && faulty_mpu != MPU1)
            status = FAILED;
        else
        {
            status = DEGRADED;
        }
        faulty_mpu = MPU1;
        break;

    case MPU2_DISAGREE:
        clean_streak = 0;

        if (status == DEGRADED && faulty_mpu != MPU2)
            status = FAILED;
        else
        {
            status = DEGRADED;
        }
        faulty_mpu = MPU2;
        break;

    case MPU3_DISAGREE:
        clean_streak = 0;

        if (status == DEGRADED && faulty_mpu != MPU3)
            status = FAILED;
        else
        {
            status = DEGRADED;
        }
        faulty_mpu = MPU3;
        break;

    case ALL_AGREE:
        if (status == DEGRADED || status == FAILED)
        {
            clean_streak++;
        }
        if (status == DEGRADED && clean_streak >= ALL_AGREE_N)
        {
            printf("%d", clean_streak);
            status = NOMINAL;
            clean_streak = 0;
            faulty_mpu = MPU_NONE;
        }
        break;

    case ALL_DISAGREE:
        clean_streak = 0;

        status = FAILED;
        break;

    default:
        status = FAILED;
        break;
    }

    return status;
}

const char *state_name(system_condition status)
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