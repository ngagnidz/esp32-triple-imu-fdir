#include <stdio.h>
#include "voting.h"
#include "fdir.h"
static system_condition status;
static int clean_streak = 0;
static int error_streak = 0;
static int system_wide_error_streak = 0;
extern int mpu_fault[3];

const char *get_state_name(system_condition status)
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

int get_state_number(system_condition status)
{
    switch (status)
    {
    case NOMINAL:
        return 1;
    case DEGRADED:
        return 2;
    case FAILED:
        return 3;
    default:
        return 4;
    };
}

system_condition
update_state(voting_result result)
{
    if (status == FAILED)
    {
        if (result == ALL_AGREE || result == MPU1_DISAGREE ||
            result == MPU2_DISAGREE || result == MPU3_DISAGREE)
        {
            clean_streak++;

            if (clean_streak >= ALL_AGREE_N)
            {
                int counter = 0;
                for (int i = 0; i < 3; i++)
                {
                    counter += mpu_fault[i];
                }

                if (counter == 0)
                {
                    status = NOMINAL;
                }
                else if (counter == 1)
                {
                    status = DEGRADED;
                }

                clean_streak = 0;
            }
        }
        else
        {
            clean_streak = 0;
        }
        return status;
    }

    switch (result)
    {
    case MPU1_DISAGREE:
    case MPU2_DISAGREE:
    case MPU3_DISAGREE:
        clean_streak = 0;
        system_wide_error_streak = 0;
        error_streak++;
        if (error_streak >= 10)
        {
            status = DEGRADED;
            error_streak = 0;
            break;
        }
        break;

    case ALL_AGREE:
        error_streak = 0;
        system_wide_error_streak = 0;
        if (status == DEGRADED)
        {
            clean_streak++;
        }
        if (status == DEGRADED && clean_streak >= ALL_AGREE_N)
        {
            status = NOMINAL;
            clean_streak = 0;
        }
        break;

    case ALL_DISAGREE:

        clean_streak = 0;
        system_wide_error_streak++;
        error_streak = 0;
        if (system_wide_error_streak >= 10)
        {
            status = FAILED;
            system_wide_error_streak = 0;
            break;
        }
        break;

    default:

        clean_streak = 0;
        status = FAILED;
        break;
    }

    return status;
}