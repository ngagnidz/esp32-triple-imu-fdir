#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "mpu9250.h"
#include "voting.h"
#include "fdir.h"

static QueueHandle_t queues[3];

static void queue_creation(void)
{
    for (int i = 0; i < 3; i++)
    {
        queues[i] = xQueueCreate(1, sizeof(burst_read_data_t));
    }
}

static void reader_task(void *pv)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(10);
    burst_read_data_t readings[3];

    while (1)
    {
        mpu_read_all(readings);
        for (int i = 0; i < 3; i++)
        {
            xQueueSend(queues[i], &readings[i], portMAX_DELAY);
        }
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

static void voter_task(void *pv)
{
    burst_read_data_t readings[3];
    while (1)
    {
        for (int i = 0; i < 3; i++)
        {
            xQueueReceive(queues[i], &readings[i], portMAX_DELAY);
        }

        voting_result result = voter(readings[0], readings[1], readings[2]);
        system_condition condition = update_state(result);
        char *state = state_name(condition);
        printf("state: %s\n\n", state);
    }
}

void app_main(void)
{
    mpu_init();
    queue_creation();
    // mpu_calibrate(200);

    xTaskCreate(reader_task, "reader", 4096, NULL, 5, NULL);
    xTaskCreate(voter_task, "voter", 4096, NULL, 4, NULL);
}
