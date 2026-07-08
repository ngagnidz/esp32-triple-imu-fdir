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

spi_device_handle_t mpu9250_1;
spi_device_handle_t mpu9250_2;
spi_device_handle_t mpu9250_3;

QueueHandle_t queues[3];

static QueueHandle_t queue_1;
static QueueHandle_t queue_2;
static QueueHandle_t queue_3;

static DRAM_ATTR uint8_t buf[3][14];

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
    spi_bus_add_device(SPI3_HOST, &devConfig1, &mpu9250_1);

    spi_device_interface_config_t devConfig2 = {
        .mode = 3,
        .clock_speed_hz = 1000000,
        .command_bits = 1,
        .address_bits = 7,
        .dummy_bits = 0,
        .queue_size = 1,
        .spics_io_num = PIN_CS2,

    };
    spi_bus_add_device(SPI3_HOST, &devConfig2, &mpu9250_2);

    spi_device_interface_config_t devConfig3 = {
        .mode = 3,
        .clock_speed_hz = 1000000,
        .command_bits = 1,
        .address_bits = 7,
        .dummy_bits = 0,
        .queue_size = 1,
        .spics_io_num = PIN_CS3,
    };
    spi_bus_add_device(SPI3_HOST, &devConfig3, &mpu9250_3);
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
        .cmd = 1,
        .addr = 0x6B,
        .length = 8,
        .flags = SPI_TRANS_USE_RXDATA,
    };
    // blocking transmit, result lands in rx_data[0]
    spi_device_polling_transmit(dev, &wakeup_read);

    printf("SLEEPING_MODE_STATUS: 0x%02X\n", wakeup_read.rx_data[0]);
    if (wakeup_read.rx_data[0] == 0x01)
    {
        // +- 500dps
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

        // +-8g
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
    // read WHO_AM_I register 0x75
    // should come back as 0x71 for MPU-9250 or 0x70 for MPU-6500
    // only 1 byte back so using inline rx_data instead of a separate buffer
    whoami_trans = (spi_transaction_t){
        .cmd = 1,
        .addr = 0x75,
        .length = 8,
        .flags = SPI_TRANS_USE_RXDATA,
    };
    // blocking transmit, result lands in rx_data[0]
    spi_device_polling_transmit(dev, &whoami_trans);
    printf("WHO_AM_I: 0x%02X\n", whoami_trans.rx_data[0]);
}

burst_read_data_t burst_read(correction_data_t corr, spi_device_handle_t dev, int idx)
{
    burst_read_data_t read = {0};

    spi_transaction_t burst_accel_read = {
        .cmd = 1,
        .length = 8 * 14,
        .rx_buffer = buf[idx],
        .addr = 0x3B,
    };

    spi_device_queue_trans(dev, &burst_accel_read, portMAX_DELAY);

    spi_transaction_t *rtrans;
    spi_device_get_trans_result(dev, &rtrans, portMAX_DELAY);

    read.accel_x = ((int16_t)((buf[idx][0] << 8) | buf[idx][1]) / ACCEL_SENSITIVITY) - corr.accel_x;
    read.accel_y = ((int16_t)((buf[idx][2] << 8) | buf[idx][3]) / ACCEL_SENSITIVITY) - corr.accel_y;
    read.accel_z = ((int16_t)((buf[idx][4] << 8) | buf[idx][5]) / ACCEL_SENSITIVITY) - corr.accel_z;

    read.temp = ((int16_t)((buf[idx][6] << 8) | buf[idx][7]) / TEMP_SENSITIVITY) + TEMP_OFFSET;

    read.gyro_x = ((int16_t)((buf[idx][8] << 8) | buf[idx][9]) / GYRO_SENSITIVITY) - corr.gyro_x;
    read.gyro_y = ((int16_t)((buf[idx][10] << 8) | buf[idx][11]) / GYRO_SENSITIVITY) - corr.gyro_y;
    read.gyro_z = ((int16_t)((buf[idx][12] << 8) | buf[idx][13]) / GYRO_SENSITIVITY) - corr.gyro_z;

    return read;
}

correction_data_t calibrate(spi_device_handle_t dev, int idx)
{
    correction_data_t corr = {0};
    correction_data_t zero = {0};

    for (int i = 0; i < CALIBRATION_SAMPLES; i++)
    {

        burst_read_data_t burst_read_result = burst_read(zero, dev, idx);
        corr.accel_x += burst_read_result.accel_x;
        corr.accel_y += burst_read_result.accel_y;
        corr.accel_z += burst_read_result.accel_z;

        corr.gyro_x += burst_read_result.gyro_x;
        corr.gyro_y += burst_read_result.gyro_y;
        corr.gyro_z += burst_read_result.gyro_z;
    }

    corr.accel_x /= CALIBRATION_SAMPLES;
    corr.accel_y /= CALIBRATION_SAMPLES;
    corr.accel_z /= CALIBRATION_SAMPLES;
    corr.accel_z -= GRAVITY;

    corr.gyro_x /= CALIBRATION_SAMPLES;
    corr.gyro_y /= CALIBRATION_SAMPLES;
    corr.gyro_z /= CALIBRATION_SAMPLES;
    return corr;
}

typedef struct
{
    correction_data_t corr[3];
    spi_device_handle_t dev[3];
} reader_arguments;

void reader_task(void *pv)
{
    reader_arguments *args = (reader_arguments *)pv;

    while (1)
    {
        for (int i = 0; i < 3; i++)
        {
            burst_read_data_t reading = burst_read(args->corr[i], args->dev[i], i);
            xQueueSend(queues[i], &reading, portMAX_DELAY);
            // printf("Voter Task\n");
        }
    }
}

static reader_arguments args;

// voting_result voter(burst_read_data_t data1, burst_read_data_t data2, burst_read_data_t data3);
void voter_task(void *pv)
{
    burst_read_data_t readings[3];
    while (1)
    {

        vTaskDelay(pdMS_TO_TICKS(2000));

        int64_t t_start = esp_timer_get_time();
        for (int i = 0; i < 3; i++)
        {
            xQueueReceive(queues[i], &readings[i], portMAX_DELAY);
        }
        voting_result result = voter(readings[0], readings[1], readings[2]);
        system_condition condition = update_state(result);
        char *state = state_name(condition);
        // printf("Test");
        printf("%s\n", state);

        int64_t t_end = esp_timer_get_time();

        printf("[t=%lld us, dt=%lld us] result=%d state=%s\n",
               t_start, t_end - t_start, result, state);
    }
}

void app_main(void)
{
    bus_config();
    device_config();
    setup(mpu9250_1);
    setup(mpu9250_2);
    setup(mpu9250_3);

    spi_device_handle_t handlers[3];
    handlers[0] = mpu9250_1;
    handlers[1] = mpu9250_2;
    handlers[2] = mpu9250_3;

    correction_data_t corrections[3];
    corrections[0] = calibrate(mpu9250_1, 0);
    corrections[1] = calibrate(mpu9250_2, 1);
    corrections[2] = calibrate(mpu9250_3, 2);

    for (int i = 0; i < 3; i++)
    {
        args.corr[i] = corrections[i];
        args.dev[i] = handlers[i];
    }

    queue_1 = xQueueCreate(1, sizeof(burst_read_data_t));
    queue_2 = xQueueCreate(1, sizeof(burst_read_data_t));
    queue_3 = xQueueCreate(1, sizeof(burst_read_data_t));

    queues[0] = queue_1;
    queues[1] = queue_2;
    queues[2] = queue_3;

    xTaskCreate(reader_task, "reader", 4096, &args, 5, NULL);
    xTaskCreate(voter_task, "voter", 4096, &args, 6, NULL);
}