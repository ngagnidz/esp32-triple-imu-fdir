#include <stdio.h>
#include "mpu9250.h"
#include "voting.h"
#include "fdir.h"

static float gyro_error[3] = {0, 0, 0};

static int failures = 0;

static burst_read_data_t fake_reading(int unit)
{
    burst_read_data_t r = {
        .accel_x = 0.0f,
        .accel_y = 0.0f,
        .accel_z = -1.0f,
        .temp = 25.0f,
        .gyro_x = 0.2f + gyro_error[unit],
        .gyro_y = 5.0f,
        .gyro_z = 0.0f,
    };
    return r;
}

static void phase(const char *name, float e1, float e2, float e3,
                  int samples, system_condition expect)
{
    gyro_error[0] = e1;
    gyro_error[1] = e2;
    gyro_error[2] = e3;

    voting_result vote = ALL_AGREE;
    system_condition state = NOMINAL;

    for (int i = 0; i < samples; i++)
    {
        burst_read_data_t readings[3];
        for (int u = 0; u < 3; u++)
            readings[u] = fake_reading(u);

        vote = voter(readings[0], readings[1], readings[2]);
        state = update_state(vote);
    }

    if (state == expect)
    {
        printf("%s -> %s (voter: %s) PASS\n", name,
               get_state_name(state), return_result(vote));
    }
    else
    {
        printf("%s -> %s (voter: %s) FAIL\n", name,
               get_state_name(state), return_result(vote));
        failures++;
    }
}

int main(void)
{

    phase("all healthy", 0, 0, 0, 5, NOMINAL);
    phase("MPU1 breaks", 40, 0, 0, 12, DEGRADED);
    phase("MPU1 healed", 0, 0, 0, 5, NOMINAL);

    phase("MPU1+MPU2 break", 40, -40, 0, 12, FAILED);

    phase("MPU2 healed", 40, 0, 0, 5, DEGRADED);
    phase("MPU1 healed", 0, 0, 0, 5, NOMINAL);

    phase("both break again", 40, -40, 0, 12, FAILED);
    phase("both healed", 0, 0, 0, 5, NOMINAL);

    phase("both break again", 40, -40, 0, 12, FAILED);
    phase("not enough readings to heal", 0, 0, 0, 2, FAILED);
    phase("both break again", 40, -40, 0, 12, FAILED);
    phase("both healed", 0, 0, 0, 5, NOMINAL);

    phase("less than 10 broken readings", 40, -40, 0, 6, NOMINAL);

    if (failures == 0)
        printf("\nALL TESTS PASSED\n\n");
    else
        printf("\n%d TESTS FAILED\n\n", failures);

    return failures;
}
