#include <stdio.h>
#include "driver/spi_master.h"
#include "esp_attr.h"
#include "mpu9250.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static spi_device_handle_t device_list[3];
static WORD_ALIGNED_ATTR uint8_t buf[2][3][16];
static int fill = 0;
static bool primed = false;
typedef struct
{
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
} mpu_offset;

static mpu_offset offsets[3];

static void bus_config(void)
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

static void device_config(void)
{
    const int cs_pins[3] = {PIN_CS1, PIN_CS2, PIN_CS3};

    for (int i = 0; i < 3; i++)
    {
        spi_device_interface_config_t devConfig = {
            .mode = 3,
            .clock_speed_hz = 1000000,
            .command_bits = 1,
            .address_bits = 7,
            .dummy_bits = 0,
            .queue_size = 1,
            .spics_io_num = cs_pins[i],
        };
        spi_bus_add_device(SPI3_HOST, &devConfig, &device_list[i]);
    }
}

static int setup(spi_device_handle_t dev)
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
    return 0;
}

static void burst_queue(int slot)
{

    static spi_transaction_t trans[3];
    for (int idx = 0; idx < 3; idx++)
    {
        trans[idx].cmd = 1;
        trans[idx].length = 8 * 14;
        trans[idx].addr = 0x3B;
        trans[idx].rx_buffer = buf[slot][idx];
        spi_device_queue_trans(device_list[idx], &trans[idx], portMAX_DELAY);
    }
}

static int burst_cycle()
{
    if (primed == false)
    {
        burst_queue(fill);
        primed = true;
    }

    spi_transaction_t *r;

    for (int idx = 0; idx < 3; idx++)
    {
        spi_device_get_trans_result(device_list[idx], &r, portMAX_DELAY);
    }

    int done = fill;
    fill = fill ^ 1;
    burst_queue(fill);
    return done;
}

static burst_read_data_t reading_extraction(int slot, int idx)
{
    burst_read_data_t read = {0};

    read.accel_x = ((int16_t)((buf[slot][idx][0] << 8) | buf[slot][idx][1]) / ACCEL_SENSITIVITY) - offsets[idx].ax;
    read.accel_y = ((int16_t)((buf[slot][idx][2] << 8) | buf[slot][idx][3]) / ACCEL_SENSITIVITY) - offsets[idx].ay;
    read.accel_z = ((int16_t)((buf[slot][idx][4] << 8) | buf[slot][idx][5]) / ACCEL_SENSITIVITY) - offsets[idx].az;

    read.temp = ((int16_t)((buf[slot][idx][6] << 8) | buf[slot][idx][7]) / TEMP_SENSITIVITY) + TEMP_OFFSET;

    read.gyro_x = ((int16_t)((buf[slot][idx][8] << 8) | buf[slot][idx][9]) / GYRO_SENSITIVITY) - offsets[idx].gx;
    read.gyro_y = ((int16_t)((buf[slot][idx][10] << 8) | buf[slot][idx][11]) / GYRO_SENSITIVITY) - offsets[idx].gy;
    read.gyro_z = ((int16_t)((buf[slot][idx][12] << 8) | buf[slot][idx][13]) / GYRO_SENSITIVITY) - offsets[idx].gz;

    return read;
}

void mpu_init(void)
{
    bus_config();
    device_config();
    for (int i = 0; i < 3; i++)
    {
        setup(device_list[i]);
    }
}

void mpu_read_all(burst_read_data_t out[3])
{

    int slot = burst_cycle();
    for (int i = 0; i < 3; i++)
    {
        out[i] = reading_extraction(slot, i);
    }
}

void mpu_calibrate(int samples)
{

    float ax[3] = {0}, ay[3] = {0}, az[3] = {0};
    float gx[3] = {0}, gy[3] = {0}, gz[3] = {0};

    for (int n = 0; n < samples; n++)
    {
        int slot = burst_cycle();

        for (int i = 0; i < 3; i++)
        {
            burst_read_data_t reading = reading_extraction(slot, i);
            ax[i] += reading.accel_x;
            ay[i] += reading.accel_y;
            az[i] += reading.accel_z;
            gx[i] += reading.gyro_x;
            gy[i] += reading.gyro_y;
            gz[i] += reading.gyro_z;
        }
    }

    for (int i = 0; i < 3; i++)
    {
        offsets[i].ax = ax[i] / samples;
        offsets[i].ay = ay[i] / samples;
        offsets[i].az = az[i] / samples - (-1.0f);
        offsets[i].gx = gx[i] / samples;
        offsets[i].gy = gy[i] / samples;
        offsets[i].gz = gz[i] / samples;
    }
}
