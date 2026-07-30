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

    ESP_LOGI(TAG, "CAN link up: tx=%d rx=%d 500 kbit/s (normal mode)",
             TX_GPIO, RX_GPIO);
}

void can_send_state(uint8_t state)
{
    twai_status_info_t st;
    twai_get_status_info(&st);

    switch (st.state)
    {
    case TWAI_STATE_RUNNING:
    {
        twai_message_t msg = {
            .identifier = CAN_ID,
            .data_length_code = 1};
        msg.data[0] = state;
        esp_err_t r = twai_transmit(&msg, pdMS_TO_TICKS(10));
        if (r == ESP_OK)
        {
            ESP_LOGI(TAG, "TX ok  state=%u | tx_err=%lu rx_err=%lu tx_fail=%lu",
                     state,
                     (unsigned long)st.tx_error_counter,
                     (unsigned long)st.rx_error_counter,
                     (unsigned long)st.tx_failed_count);
        }
        else
        {
            ESP_LOGW(TAG, "TX fail (%s) | tx_err=%lu rx_err=%lu tx_fail=%lu",
                     esp_err_to_name(r),
                     (unsigned long)st.tx_error_counter,
                     (unsigned long)st.rx_error_counter,
                     (unsigned long)st.tx_failed_count);
        }
        break;
    }

    case TWAI_STATE_BUS_OFF:
        ESP_LOGW(TAG, "BUS_OFF initiating recovery");
        twai_initiate_recovery();
        break;

    case TWAI_STATE_RECOVERING:
        break;

    case TWAI_STATE_STOPPED:
        twai_start();
        ESP_LOGI(TAG, "Recovery complete. Restarted");
        break;

    default:
        break;
    }
}

void can_send_data(float data)
{
    twai_status_info_t st;
    twai_get_status_info(&st);

    switch (st.state)
    {
    case TWAI_STATE_RUNNING:
    {
        twai_message_t msg = {
            .identifier = CAN_ID,
            .data_length_code = sizeof(float)};
        memcpy(&msg.data[0], &data, sizeof(float));
        esp_err_t r = twai_transmit(&msg, pdMS_TO_TICKS(10));
        if (r == ESP_OK)
        {
            ESP_LOGI(TAG, "TX ok  data=%.3f | tx_err=%lu rx_err=%lu tx_fail=%lu",
                     data,
                     (unsigned long)st.tx_error_counter,
                     (unsigned long)st.rx_error_counter,
                     (unsigned long)st.tx_failed_count);
        }
        else
        {
            ESP_LOGW(TAG, "TX fail (%s) | tx_err=%lu rx_err=%lu tx_fail=%lu",
                     esp_err_to_name(r),
                     (unsigned long)st.tx_error_counter,
                     (unsigned long)st.rx_error_counter,
                     (unsigned long)st.tx_failed_count);
        }
        break;
    }

    case TWAI_STATE_BUS_OFF:
        ESP_LOGW(TAG, "BUS_OFF initiating recovery");
        twai_initiate_recovery();
        break;

    case TWAI_STATE_RECOVERING:
        break;

    case TWAI_STATE_STOPPED:
        twai_start();
        ESP_LOGI(TAG, "Recovery complete. Restarted");
        break;

    default:
        break;
    }
}