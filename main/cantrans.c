#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_log.h"
#include "fdir.h"

#define TAG "can_bringup"
#define TX_GPIO 19
#define RX_GPIO 18

static twai_node_handle_t node_hdl = NULL;

void config(void)
{
    twai_onchip_node_config_t node_config = {
        .io_cfg.tx = TX_GPIO,
        .io_cfg.rx = RX_GPIO,
        .bit_timing.bitrate = 500000,
        .tx_queue_depth = 1,
    };

    twai_new_node_onchip(&node_config, &node_hdl);
    twai_node_enable(node_hdl);
}

void transmit(int status_code)
{

    uint8_t send_buff[1] = {(uint8_t)status_code};

    twai_frame_t tx_frame = {
        .header.id = 0x100,
        .buffer = send_buff,
        .buffer_len = sizeof(send_buff),
    };

    esp_err_t result = twai_node_transmit(node_hdl, &tx_frame, 1000);
    if (result == ESP_OK)
    {
        twai_node_transmit_wait_all_done(node_hdl, 1000);
        ESP_LOGI(TAG, "CAN TX ok: status=%d", status_code);
    }
    else
    {
        ESP_LOGW(TAG, "CAN TX failed: %s", esp_err_to_name(result));
    }
}