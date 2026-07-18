#include <stdio.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "mpu9250.h"
#include "voting.h"
#include "fdir.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"

spi_transaction_t whoami_trans;
spi_device_handle_t device_list[3];

// setup the SPI bus on SPI3 with DMA so CPU is free during transfers
void bus_config()
{

    spi_bus_config_t busConfig = {
        .sclk_io_num = PIN_SCLK,
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    spi_bus_initialize(SPI3_HOST, &busConfig, SPI_DMA_CH_AUTO);
}

// add the IMU as a device on the bus
// 1MHz because datasheet says config registers cant go faster, sensor reads can do 20MHz later
// command_bits=1 is the read/write bit, address_bits=7 is the register address
void device_config()
{

    spi_device_interface_config_t devConfig1 = {
        .mode = 3,
        .clock_speed_hz = 1000000,
        .command_bits = 1,
        .address_bits = 7,
        .dummy_bits = 0,
        .queue_size = 1,
        .spics_io_num = PIN_CS1,

    };
    spi_bus_add_device(SPI3_HOST, &devConfig1, &device_list[0]);

    spi_device_interface_config_t devConfig2 = {
        .mode = 3,
        .clock_speed_hz = 1000000,
        .command_bits = 1,
        .address_bits = 7,
        .dummy_bits = 0,
        .queue_size = 1,
        .spics_io_num = PIN_CS2,

    };
    spi_bus_add_device(SPI3_HOST, &devConfig2, &device_list[1]);

    spi_device_interface_config_t devConfig3 = {
        .mode = 3,
        .clock_speed_hz = 1000000,
        .command_bits = 1,
        .address_bits = 7,
        .dummy_bits = 0,
        .queue_size = 1,
        .spics_io_num = PIN_CS3,
    };
    spi_bus_add_device(SPI3_HOST, &devConfig3, &device_list[2]);
}

int setup(spi_device_handle_t dev)
{

    spi_transaction_t wakeup_trans = {
        .length = 8,
        .cmd = 0,
        .addr = 0x6B,
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA,
        .tx_data = {0x01},
    };
    spi_device_polling_transmit(dev, &wakeup_trans);

    spi_transaction_t wakeup_read = {
        .length = 8,
        .cmd = 1,
        .addr = 0x6B,
        .flags = SPI_TRANS_USE_RXDATA,
    };
    spi_device_polling_transmit(dev, &wakeup_read);

    printf("SLEEPING_MODE_STATUS: 0x%02X\n", wakeup_read.rx_data[0]);

    if (wakeup_read.rx_data[0] == 0x01)
    {
        spi_transaction_t gyro_range_write = {
            .length = 8,
            .cmd = 0,
            .addr = 0x1B,
            .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA,
            .tx_data = {0x08},
        };

        spi_device_polling_transmit(dev, &gyro_range_write);

        spi_transaction_t gyro_range_read = {
            .length = 8,
            .cmd = 1,
            .addr = 0x1B,
            .flags = SPI_TRANS_USE_RXDATA,
        };
        spi_device_polling_transmit(dev, &gyro_range_read);
        printf("Gyro Range: 0x%X\n", gyro_range_read.rx_data[0]);

        spi_transaction_t accel_range_write = {
            .length = 8,
            .cmd = 0,
            .addr = 0x1C,
            .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA,
            .tx_data = {0x10},
        };

        spi_device_polling_transmit(dev, &accel_range_write);

        spi_transaction_t accel_range_read = {
            .length = 8,
            .cmd = 1,
            .addr = 0x1C,
            .flags = SPI_TRANS_USE_RXDATA,
        };
        spi_device_polling_transmit(dev, &accel_range_read);
        printf("Accel Range: 0x%X\n", accel_range_read.rx_data[0]);

        return 1;
    }
    else
        return 0;
}

void whoami(spi_device_handle_t dev)
{

    whoami_trans = (spi_transaction_t){
        .cmd = 1,
        .addr = 0x75,
        .length = 8,
        .flags = SPI_TRANS_USE_RXDATA,
    };
    spi_device_polling_transmit(dev, &whoami_trans);
    printf("WHO_AM_I: 0x%02X\n", whoami_trans.rx_data[0]);
}

uint8_t buf[3][14];

void burst_read()
{

    spi_device_handle_t dev;
    for (int idx = 0; idx < 3; idx++)
    {
        spi_transaction_t burst_read = {
            .cmd = 1,
            .length = 8 * 14,
            .rx_buffer = buf[idx],
            .addr = 0x3B,
        };

        dev = device_list[idx];

        // Gives the transaction to the driver's pending queue and returns
        // Blocks only if the pending queue is full
        spi_device_queue_trans(dev, &burst_read, portMAX_DELAY);

        spi_transaction_t *dummy;
        // Blocks until a transaction completes and goes into the ready queueurst_read.
        // dummy will be pointing to &burst_read
        spi_device_get_trans_result(dev, &dummy, portMAX_DELAY);
    }
}

typedef struct
{
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
} mpu_offset;

mpu_offset offsets[3];

burst_read_data_t reading_extraction(int idx)
{
    burst_read_data_t read = {0};

    read.accel_x = ((int16_t)((buf[idx][0] << 8) | buf[idx][1]) / ACCEL_SENSITIVITY) - offsets[idx].ax;
    read.accel_y = ((int16_t)((buf[idx][2] << 8) | buf[idx][3]) / ACCEL_SENSITIVITY) - offsets[idx].ay;
    read.accel_z = ((int16_t)((buf[idx][4] << 8) | buf[idx][5]) / ACCEL_SENSITIVITY) - offsets[idx].az;

    read.temp = ((int16_t)((buf[idx][6] << 8) | buf[idx][7]) / TEMP_SENSITIVITY) + TEMP_OFFSET;

    read.gyro_x = ((int16_t)((buf[idx][8] << 8) | buf[idx][9]) / GYRO_SENSITIVITY) - offsets[idx].gx;
    read.gyro_y = ((int16_t)((buf[idx][10] << 8) | buf[idx][11]) / GYRO_SENSITIVITY) - offsets[idx].gy;
    read.gyro_z = ((int16_t)((buf[idx][12] << 8) | buf[idx][13]) / GYRO_SENSITIVITY) - offsets[idx].gz;

    return read;
}

void calibrate(int samples)
{
    for (int i = 0; i < 3; i++)
    {
        float ay = 0;
        float ax = 0;
        float az = 0;
        float gx = 0;
        float gy = 0;
        float gz = 0;

        for (int n = 0; n < samples; n++)
        {
            burst_read();
            burst_read_data_t reading = reading_extraction(i);
            ax += reading.accel_x;
            ay += reading.accel_y;
            az += reading.accel_z;
            gx += reading.gyro_x;
            gy += reading.gyro_y;
            gz += reading.gyro_z;
        }

        offsets[i].ax = ax / samples;
        offsets[i].ay = ay / samples;
        offsets[i].az = az / samples - 1.0;
        offsets[i].gx = gx / samples;
        offsets[i].gy = gy / samples;
        offsets[i].gz = gz / samples;
    }
}

QueueHandle_t queues[3];
void queue_creation()
{
    for (int i = 0; i < 3; i++)
    {
        queues[i] = xQueueCreate(1, sizeof(burst_read_data_t));
    }
}

void reader_task(void *pv)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(10);

    while (1)
    {
        burst_read();
        for (int i = 0; i < 3; i++)
        {
            burst_read_data_t reading = reading_extraction(i);
            xQueueSend(queues[i], &reading, portMAX_DELAY);
        }
    }

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
}

void voter_task(void *pv)
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
    bus_config();
    device_config();
    setup(device_list[0]);
    setup(device_list[1]);
    setup(device_list[2]);

    queue_creation();

    offsets[0] = (mpu_offset){0};
    offsets[1] = (mpu_offset){0};
    offsets[2] = (mpu_offset){0};
    calibrate(500);

    xTaskCreate(reader_task, "reader", 4096, NULL, 5, NULL);
    xTaskCreate(voter_task, "voter", 4096, NULL, 4, NULL);
}