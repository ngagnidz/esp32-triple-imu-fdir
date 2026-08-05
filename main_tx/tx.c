#include <string.h>
#include <stdbool.h>
#include "tx.h"
#include "driver/twai.h"
#include "esp_log.h"

static const char *TAG = "CAN_TX";

void can_link_init(void)
{
    twai_general_config_t g =
        TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO, RX_GPIO, TWAI_MODE_NORMAL);

    twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();

    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g, &t, &f));
    ESP_ERROR_CHECK(twai_start());

    ESP_LOGI(TAG, "CAN link up: tx=%d rx=%d",
             TX_GPIO, RX_GPIO);
}

static bool can_bus_ready(void)
{
    twai_status_info_t st;
    twai_get_status_info(&st);

    switch (st.state)
    {
    case TWAI_STATE_RUNNING:
        return true;

    case TWAI_STATE_BUS_OFF:
        ESP_LOGW(TAG, "initiating recovery");
        twai_initiate_recovery();
        return false;

    case TWAI_STATE_STOPPED:
        twai_start();
        ESP_LOGI(TAG, "Recovery complete. Restarted");
        return false;

    case TWAI_STATE_RECOVERING:
    default:
        return false;
    }
}

static void log_tx_failure(esp_err_t r)
{
    static uint32_t fails;

    // rate limit - 1, 101, 201..
    if (fails++ % 100 != 1)
        return;

    twai_status_info_t st;
    twai_get_status_info(&st);

    ESP_LOGW(TAG, "TX fail (%s) x%lu | tx_err=%lu rx_err=%lu tx_fail=%lu",
             esp_err_to_name(r), (unsigned long)fails,
             (unsigned long)st.tx_error_counter,
             (unsigned long)st.rx_error_counter,
             (unsigned long)st.tx_failed_count);
}

void can_send_state(uint8_t state)
{
    if (!can_bus_ready())
        return;

    twai_message_t msg = {
        .identifier = CAN_ID,
        .data_length_code = 1};

    msg.data[0] = state;

    esp_err_t r = twai_transmit(&msg, pdMS_TO_TICKS(10));
    if (r != ESP_OK)
        log_tx_failure(r);
}

void can_send_data(char axis, float data)
{
    if (!can_bus_ready())
        return;

    twai_message_t msg = {
        .identifier = CAN_ID,
        .data_length_code = 5};

    msg.data[0] = axis;
    memcpy(&msg.data[1], &data, sizeof(float));

    esp_err_t r = twai_transmit(&msg, pdMS_TO_TICKS(10));
    if (r != ESP_OK)
        log_tx_failure(r);
}