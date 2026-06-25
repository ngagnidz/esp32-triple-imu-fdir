#include <stdio.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"

#define PIN_SCLK 18
#define PIN_MOSI 23
#define PIN_MISO 19
#define PIN_CS 5

spi_device_handle_t mpu9250;
spi_transaction_t whoami_trans;

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

int wakeup()
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
        return 1;
    }
    else
        return 0;
}

void app_main(void)
{
    bus_config();
    device_config();
    whoami();
    wakeup();

    spi_transaction_t accel_x_high = {
        .cmd = 1,
        .addr = 0x3B, // accel high
        .length = 8,
        .flags = SPI_TRANS_USE_RXDATA,
    };
    // blocking transmit, result lands in rx_data[0]
    spi_device_polling_transmit(mpu9250, &accel_x_high);
    printf("Accel X High: 0x%02X\n", accel_x_high.rx_data[0]);

    while (1)
    { // Z high byte should be around 0x3F-0x40 (approx 16384 = 0x4000)
        spi_transaction_t accel_z_high = {
            .cmd = 1,
            .addr = 0x3F,
            .length = 8,
            .flags = SPI_TRANS_USE_RXDATA,
        };
        spi_device_polling_transmit(mpu9250, &accel_z_high);
        printf("Accel Z High: 0x%02X\n", accel_z_high.rx_data[0]);
    }
}