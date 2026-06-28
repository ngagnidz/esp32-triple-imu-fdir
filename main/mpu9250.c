#include <stdio.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_attr.h"

#define PIN_SCLK 18
#define PIN_MOSI 23
#define PIN_MISO 19
#define PIN_CS1 5
#define PIN_CS2 4
#define PIN_CS3 15

spi_transaction_t whoami_trans;
spi_device_handle_t mpu9250_1;
spi_device_handle_t mpu9250_2;
spi_device_handle_t mpu9250_3;

static DRAM_ATTR uint8_t buf[14];
#define ACCEL_SENSITIVITY 4096.0f
#define GYRO_SENSITIVITY 65.5f
#define TEMP_SENSITIVITY 333.87f
#define TEMP_OFFSET 21.0f
#define CALIBRATION_SAMPLES 1000
#define GRAVITY 1.0f

typedef struct
{
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} correction_data_t;

typedef struct
{
    float accel_x;
    float accel_y;
    float accel_z;
    float temp;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} burst_read_data_t;

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

burst_read_data_t burst_read(correction_data_t corr, spi_device_handle_t dev)
{

    burst_read_data_t read;

    spi_transaction_t burst_accel_read = {

        .cmd = 1,
        .length = 8 * 14,
        .rx_buffer = buf,
        .addr = 0x3B,

    };

    spi_device_polling_transmit(dev, &burst_accel_read);
    read.accel_x = ((int16_t)((buf[0] << 8) | buf[1]) / ACCEL_SENSITIVITY) - corr.accel_x;
    read.accel_y = ((int16_t)((buf[2] << 8) | buf[3]) / ACCEL_SENSITIVITY) - corr.accel_y;
    read.accel_z = ((int16_t)((buf[4] << 8) | buf[5]) / ACCEL_SENSITIVITY) - corr.accel_z;

    read.temp = ((int16_t)((buf[6] << 8) | buf[7]) / TEMP_SENSITIVITY) + TEMP_OFFSET;

    read.gyro_x = ((int16_t)((buf[8] << 8) | buf[9]) / GYRO_SENSITIVITY) - corr.gyro_x;
    read.gyro_y = ((int16_t)((buf[10] << 8) | buf[11]) / GYRO_SENSITIVITY) - corr.gyro_y;
    read.gyro_z = ((int16_t)((buf[12] << 8) | buf[13]) / GYRO_SENSITIVITY) - corr.gyro_z;
    return read;
}

correction_data_t calibrate(spi_device_handle_t dev)
{
    correction_data_t corr = {0};
    correction_data_t zero = {0};

    for (int i = 0; i < CALIBRATION_SAMPLES; i++)
    {

        burst_read_data_t burst_read_result = burst_read(zero, dev);
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

void app_main(void)
{
    bus_config();
    device_config();
    whoami(mpu9250_2);

    if (!setup(mpu9250_2))
    {
        printf("IMU setup failed\n");
        return;
    }

    correction_data_t corr = calibrate(mpu9250_1);

    printf("Device 1:\n");
    burst_read_data_t data = burst_read(corr, mpu9250_1);
    printf("Accel X: %.3f g\n", data.accel_x);
    printf("Accel Y: %.3f g\n", data.accel_y);
    printf("Accel Z: %.3f g\n", data.accel_z);
    printf("Temp:    %.2f C\n", data.temp);
    printf("Gyro X:  %.3f dps\n", data.gyro_x);
    printf("Gyro Y:  %.3f dps\n", data.gyro_y);
    printf("Gyro Z:  %.3f dps\n", data.gyro_z);

    corr = calibrate(mpu9250_2);

    printf("Device 2:\n");
    data = burst_read(corr, mpu9250_2);
    printf("Accel X: %.3f g\n", data.accel_x);
    printf("Accel Y: %.3f g\n", data.accel_y);
    printf("Accel Z: %.3f g\n", data.accel_z);
    printf("Temp:    %.2f C\n", data.temp);
    printf("Gyro X:  %.3f dps\n", data.gyro_x);
    printf("Gyro Y:  %.3f dps\n", data.gyro_y);
    printf("Gyro Z:  %.3f dps\n", data.gyro_z);

    printf("Device 3:\n");
    data = burst_read(corr, mpu9250_3);
    printf("Accel X: %.3f g\n", data.accel_x);
    printf("Accel Y: %.3f g\n", data.accel_y);
    printf("Accel Z: %.3f g\n", data.accel_z);
    printf("Temp:    %.2f C\n", data.temp);
    printf("Gyro X:  %.3f dps\n", data.gyro_x);
    printf("Gyro Y:  %.3f dps\n", data.gyro_y);
    printf("Gyro Z:  %.3f dps\n", data.gyro_z);
}