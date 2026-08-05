#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "mpu9250.h"
#include "voting.h"
#include "tx.h"
#include "fdir.h"
#include "fusion.h"
#include <string.h>

static QueueHandle_t queue;

static void reader_task(void *pv)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(10);
    burst_read_data_t readings[3];

    while (1)
    {
        mpu_read_all(readings);
        xQueueSend(queue, &readings, portMAX_DELAY);
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

static void voter_task(void *pv)
{
    burst_read_data_t readings[3];
    while (1)
    {

        xQueueReceive(queue, &readings, portMAX_DELAY);
        voting_result result = voter(readings[0], readings[1], readings[2]);
        system_condition condition = update_state(result);
        int state_number = get_state_number(condition);

        data_to_send avg;

        if (average_good(readings, result, &avg))
        {
            can_send_state((uint8_t)state_number);
            can_send_data('X', avg.gyro_x);
            can_send_data('Y', avg.gyro_y);
            can_send_data('Z', avg.gyro_z);
        }
    }
}

void app_main(void)
{
    mpu_init();
    mpu_calibrate(200);

    can_link_init();

    queue = xQueueCreate(1, 3 * sizeof(burst_read_data_t));

    xTaskCreate(reader_task, "reader", 4096, NULL, 5, NULL);
    xTaskCreate(voter_task, "voter", 4096, NULL, 4, NULL);
}
