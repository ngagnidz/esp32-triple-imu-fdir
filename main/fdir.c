#include <stdio.h>
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
static int clean_streak = 0;
extern int mpu_fault[3];

char *state_name(system_condition status)
{
    switch (status)
    {
    case NOMINAL:
        return "NOMINAL";
    case DEGRADED:
        return "DEGRADED";
    case FAILED:
        return "FAILED";
    default:
        return "UNKNOWN";
    };
}

system_condition
update_state(voting_result result)
{
    if (mpu_fault[0] == 1 && mpu_fault[1] == 1 && mpu_fault[2] == 1)
    {
        status = FAILED;
    }

    if (status == FAILED)
    {
        if (result == ALL_AGREE || result == MPU1_DISAGREE ||
            result == MPU2_DISAGREE || result == MPU3_DISAGREE)
        {
            clean_streak++;
            printf("[FDIR] FAILED, clean result (%s): streak %d/%d\n",
                   return_result(result), clean_streak, ALL_AGREE_N);

            if (clean_streak >= ALL_AGREE_N)
            {
                int counter = 0;
                for (int i = 0; i < 3; i++)
                {
                    counter += mpu_fault[i];
                }
                printf("[FDIR] recovery threshold reached, fault flags = [%d %d %d], sum = %d\n",
                       mpu_fault[0], mpu_fault[1], mpu_fault[2], counter);

                if (counter == 0)
                {
                    status = NOMINAL;
                }
                else if (counter == 1)
                {
                    status = DEGRADED;
                }
                else
                {
                    printf("[FDIR] %d faults still flagged, staying FAILED\n", counter);
                }
                clean_streak = 0;
            }
        }
        else
        {
            printf("[FDIR] FAILED, bad result (%s): streak reset (was %d)\n",
                   return_result(result), clean_streak);
            clean_streak = 0;
        }
        return status;
    }

    switch (result)
    {
    case MPU1_DISAGREE:
    case MPU2_DISAGREE:
    case MPU3_DISAGREE:
        printf("[FDIR] %s in %s: streak reset (was %d), -> DEGRADED\n",
               return_result(result), state_name(status), clean_streak);
        clean_streak = 0;
        status = DEGRADED;
        break;

    case ALL_AGREE:
        if (status == DEGRADED)
        {
            clean_streak++;
            printf("[FDIR] DEGRADED, ALL_AGREE: streak %d/%d\n",
                   clean_streak, ALL_AGREE_N);
        }
        if (status == DEGRADED && clean_streak >= ALL_AGREE_N)
        {
            status = NOMINAL;
            clean_streak = 0;
        }
        break;

    case ALL_DISAGREE:
        printf("[FDIR] ALL_DISAGREE in %s: streak reset (was %d), -> FAILED\n",
               state_name(status), clean_streak);
        clean_streak = 0;
        status = FAILED;
        break;

    default:
        printf("[FDIR] UNKNOWN result %d in %s -> FAILED\n",
               result, state_name(status));
        clean_streak = 0;
        status = FAILED;
        break;
    }

    return status;
}