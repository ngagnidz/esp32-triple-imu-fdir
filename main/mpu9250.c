#include <stdio.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"

#define PIN_SCLK 18
#define PIN_MOSI 23
#define PIN_MISO 19
#define PIN_CS 5

spi_transaction_t whoami_trans;
spi_device_handle_t mpu9250;

typedef struct
{
    float x;
    float y;
    float z;
} gyro_corrections;

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
    spi_device_interface_config_t devConfig = {
        .mode = 3,
        .clock_speed_hz = 1000000,
        .command_bits = 1,
        .address_bits = 7,
        .dummy_bits = 0,
        .queue_size = 1,
        .spics_io_num = PIN_CS,
    };
    spi_bus_add_device(SPI3_HOST, &devConfig, &mpu9250);
}

int setup()
{

    spi_transaction_t wakeup_trans = {
        .length = 8,
        .cmd = 0,
        .addr = 0x6B,
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA,
        .tx_data = {0x01},
    };
    spi_device_polling_transmit(mpu9250, &wakeup_trans);

    spi_transaction_t wakeup_read = {
        .cmd = 1,
        .addr = 0x6B,
        .length = 8,
        .flags = SPI_TRANS_USE_RXDATA,
    };
    // blocking transmit, result lands in rx_data[0]
    spi_device_polling_transmit(mpu9250, &wakeup_read);

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

        spi_device_polling_transmit(mpu9250, &gyro_range_write);

        spi_transaction_t gyro_range_read = {
            .length = 8,
            .cmd = 1,
            .addr = 0x1B,
            .flags = SPI_TRANS_USE_RXDATA,
        };
        spi_device_polling_transmit(mpu9250, &gyro_range_read);
        printf("Gyro Range: 0x%X\n", gyro_range_read.rx_data[0]);

        // +-8g
        spi_transaction_t accel_range_write = {
            .length = 8,
            .cmd = 0,
            .addr = 0x1C,
            .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA,
            .tx_data = {0x10},
        };

        spi_device_polling_transmit(mpu9250, &accel_range_write);

        spi_transaction_t accel_range_read = {
            .length = 8,
            .cmd = 1,
            .addr = 0x1C,
            .flags = SPI_TRANS_USE_RXDATA,
        };
        spi_device_polling_transmit(mpu9250, &accel_range_read);
        printf("Accel Range: 0x%X\n", accel_range_read.rx_data[0]);

        return 1;
    }
    else
        return 0;
}

void whoami()
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
    spi_device_polling_transmit(mpu9250, &whoami_trans);
    printf("WHO_AM_I: 0x%02X\n", whoami_trans.rx_data[0]);
}

typedef struct
{
    float x;
    float y;
    float z;
} accelometer_data_t;

typedef struct
{
    float x;
    float y;
    float z;
} accelometer_correction;

accelometer_data_t read_accelometer(accelometer_correction corr)
{
    accelometer_data_t accel;

    spi_transaction_t accel_x_high = {
        .cmd = 1,
        .addr = 0x3B, // accel high
        .length = 8,
        .flags = SPI_TRANS_USE_RXDATA,
    };
    spi_transaction_t accel_x_low = {
        .cmd = 1,
        .addr = 0x3C, // accel low
        .length = 8,
        .flags = SPI_TRANS_USE_RXDATA,
    };

    spi_transaction_t accel_y_high = {
        .cmd = 1,
        .addr = 0x3D, // accel high
        .length = 8,
        .flags = SPI_TRANS_USE_RXDATA,
    };
    spi_transaction_t accel_y_low = {
        .cmd = 1,
        .addr = 0x3E, // accel low
        .length = 8,
        .flags = SPI_TRANS_USE_RXDATA,
    };

    spi_transaction_t accel_z_high = {
        .cmd = 1,
        .addr = 0x3F, // accel high
        .length = 8,
        .flags = SPI_TRANS_USE_RXDATA,
    };
    spi_transaction_t accel_z_low = {
        .cmd = 1,
        .addr = 0x40, // accel low
        .length = 8,
        .flags = SPI_TRANS_USE_RXDATA,
    };

    spi_device_polling_transmit(mpu9250, &accel_x_high);
    spi_device_polling_transmit(mpu9250, &accel_x_low);
    float accel_x = ((int16_t)((accel_x_high.rx_data[0] << 8) | accel_x_low.rx_data[0]) / 4096.0f) - corr.x;

    spi_device_polling_transmit(mpu9250, &accel_y_high);
    spi_device_polling_transmit(mpu9250, &accel_y_low);
    float accel_y = ((int16_t)((accel_y_high.rx_data[0] << 8) | accel_y_low.rx_data[0]) / 4096.0f) - corr.y;

    spi_device_polling_transmit(mpu9250, &accel_z_high);
    spi_device_polling_transmit(mpu9250, &accel_z_low);
    float accel_z = ((int16_t)((accel_z_high.rx_data[0] << 8) | accel_z_low.rx_data[0]) / 4096.0f) - corr.z;

    accel.x = accel_x;
    accel.y = accel_y;
    accel.z = accel_z;
    return accel;
}
typedef struct
{
    float x;
    float y;
    float z;
} gyro_data_t;

gyro_data_t read_gyro(gyro_corrections corr)
{

    gyro_data_t gyro;

    spi_transaction_t gyro_x_high = {
        .cmd = 1,
        .addr = 0x43, // accel high
        .length = 8,
        .flags = SPI_TRANS_USE_RXDATA,
    };
    spi_transaction_t gyro_x_low = {
        .cmd = 1,
        .addr = 0x44, // accel low
        .length = 8,
        .flags = SPI_TRANS_USE_RXDATA,
    };

    spi_transaction_t gyro_y_high = {
        .cmd = 1,
        .addr = 0x45, // accel high
        .length = 8,
        .flags = SPI_TRANS_USE_RXDATA,
    };
    spi_transaction_t gyro_y_low = {
        .cmd = 1,
        .addr = 0x46, // accel low
        .length = 8,
        .flags = SPI_TRANS_USE_RXDATA,
    };

    spi_transaction_t gyro_z_high = {
        .cmd = 1,
        .addr = 0x47, // accel high
        .length = 8,
        .flags = SPI_TRANS_USE_RXDATA,
    };
    spi_transaction_t gyro_z_low = {
        .cmd = 1,
        .addr = 0x48, // accel low
        .length = 8,
        .flags = SPI_TRANS_USE_RXDATA,
    };

    spi_device_polling_transmit(mpu9250, &gyro_x_high);
    spi_device_polling_transmit(mpu9250, &gyro_x_low);
    float gyro_x = (int16_t)((gyro_x_high.rx_data[0] << 8) | gyro_x_low.rx_data[0]) / 65.5 - corr.x;

    spi_device_polling_transmit(mpu9250, &gyro_y_high);
    spi_device_polling_transmit(mpu9250, &gyro_y_low);
    float gyro_y = (int16_t)((gyro_y_high.rx_data[0] << 8) | gyro_y_low.rx_data[0]) / 65.5 - corr.y;

    spi_device_polling_transmit(mpu9250, &gyro_z_high);
    spi_device_polling_transmit(mpu9250, &gyro_z_low);
    float gyro_z = (int16_t)((gyro_z_high.rx_data[0] << 8) | gyro_z_low.rx_data[0]) / 65.5 - corr.z;

    gyro.x = gyro_x;
    gyro.y = gyro_y;
    gyro.z = gyro_z;

    return gyro;
}

gyro_corrections calibrate_gyro()
{
    gyro_corrections corr = {0};
    gyro_corrections zero = {0};

    for (int i = 0; i < 1000; i++)
    {
        gyro_data_t gyro = read_gyro(zero);
        corr.x += gyro.x;
        corr.y += gyro.y;
        corr.z += gyro.z;
    }
    corr.x /= 1000;
    corr.y /= 1000;
    corr.z /= 1000;
    return corr;
}

accelometer_correction calibrate_accel()
{
    accelometer_correction corr = {0};
    accelometer_correction zero = {0};

    for (int i = 0; i < 1000; i++)
    {
        accelometer_data_t accel = read_accelometer(zero);
        corr.x += accel.x;
        corr.y += accel.y;
        corr.z += accel.z;
    }
    corr.x /= 1000;
    corr.y /= 1000;
    corr.z /= 1000;
    corr.z += 1;
    return corr;
}

void app_main(void)
{
    bus_config();
    device_config();
    setup();

    gyro_corrections gyro_corrections = calibrate_gyro();
    gyro_data_t gyro = read_gyro(gyro_corrections);

    accelometer_correction accelometer_corrections = calibrate_accel();
    accelometer_data_t accel = read_accelometer(accelometer_corrections);

    printf("%f\n", accel.x);
    printf("%f\n", accel.y);
    printf("%f\n", accel.z);
    printf("%f\n", gyro.x);
    printf("%f\n", gyro.y);
    printf("%f\n", gyro.z);
}