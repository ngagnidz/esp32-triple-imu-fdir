#include <string.h>
#include "driver/twai.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define RX_TX_GPIO 21
#define RX_RX_GPIO 22
#define CAN_ID 0x100

static const char *TAG = "CAN_RX";

void receive_config(void)
{
    twai_general_config_t g =
        TWAI_GENERAL_CONFIG_DEFAULT(RX_TX_GPIO, RX_RX_GPIO, TWAI_MODE_NORMAL);
    twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g, &t, &f));
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "CAN RX up: tx=%d rx=%d", RX_TX_GPIO, RX_RX_GPIO);
}

void app_main(void)
{
    receive_config();

    twai_message_t msg;

    for (;;)
    {
        if (twai_receive(&msg, portMAX_DELAY) != ESP_OK)
        {
            continue;
        }

        if (msg.identifier != CAN_ID)
        {
            continue;
        }

        if (msg.data_length_code == 1)
        {
            uint8_t state = msg.data[0];
            ESP_LOGI(TAG, "id=0x%lx state = %d",
                     (unsigned long)msg.identifier, state);
        }
        else if (msg.data_length_code == 5)
        {
            float gyro;
            char str;
            memcpy(&str, &msg.data[0], sizeof(char));
            memcpy(&gyro, &msg.data[1], sizeof(float));

            ESP_LOGI(TAG, "id=0x%lx direction = %c gyro = %f",
                     (unsigned long)msg.identifier, str, gyro);
        }
        else
        {
            ESP_LOGW(TAG, "id=0x%lx unexpected dlc=%u",
                     (unsigned long)msg.identifier, msg.data_length_code);
        }
    }
}