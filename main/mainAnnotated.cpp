#include <stdio.h>
#include "esp_log.h"

extern "C" {
#include "driver/spi_master.h"
#include "driver/spi_common.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

/*
Pinout
VCC -> 3.3v
CSN -> GPIO 5 (chip select, this can be any suitable GPIO pin)
MOSI -> GPIO 23 (VSPI MOSI)
GD02 -> No mapping 

GND -> GND
GD00 -> GPIO 4 (Any suitable GPIO)
SCK -> GPIO 18 (CLK Clock Signal)
MISO -> GPIO 19 (VSPI MISO)
MISO, MOSI, and SCK (The SPI interface pins)
can go to any pins technically but 
these are the default pin values...

ESP-32 API reference for the SPI protocol 
https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/spi_master.html
Note - I am getting a status byte indicating chip is not ready (0xF1). To investigate
*/

void strobe_reset(spi_device_handle_t cc1101) {
    /*
    We MUST reset the cc1101 chip as specified in section 19.1 of the data sheet
    19.1 Power-On Start-Up Sequence: When the power supply is turned on, the system
    must be reset.
    */
    spi_transaction_t t = {};
     // Register for the reset command is 0x30
     // We therefore send 1 (read) 0 (NO burst bit, this is a config register) then the address (11 0000)
    uint8_t tx[1]= {0x30}; 
    uint8_t rx[1]= {0x00}; 
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    t.length = 8; 
    ESP_ERROR_CHECK(spi_device_polling_transmit(cc1101, &t));
};

extern "C" void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI("MAIN", "Hello World!");

    // ======================CONFIGURE BUS SECTION=========================== //
    // Configure bus with appropriate GPIO pins?
    spi_bus_config_t busConfig = {};
    busConfig.mosi_io_num = GPIO_NUM_23;
    busConfig.miso_io_num = GPIO_NUM_19;
    busConfig.sclk_io_num = GPIO_NUM_18;
    busConfig.quadwp_io_num = -1; // -1 value, we are not using these
    busConfig.quadhd_io_num = -1; // -1 value, we are not using these 
    // We can then initialize the bus. If we need a larger buffer size in the future, just use 
    // SPI_DMA_CH_AUTO. This function expects a pointer to our config, not the actual object
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &busConfig, SPI_DMA_DISABLED));
    // ===================== END CONFIGURE BUS SECTION ====================== //

    // ===================== CONFIGURE DEVICE SECTION ======================= //
        /* 
    SPI2_HOST:
    ESP-32 has four SPI peripherals. SPI0 and SPI1 are for internal memory.
    SPI2 and SPI 3 are general purpose controllers for user applications
    VSPI is for SPI 3 and HSPI is for SPI 2.

    spi_device_interface_config_t:

    Configuration for an SPI slave device that is connected to one of the SPI buses. Important params - 
    - command_bits
    The default bits in a command
    - address_bits
    Default amount of bits in address phase
    - dummy_bits
    Amount of dummy bits to insert between address and data phase
    These three params will actually be defaulted to zero due to
    the contract the cc1101 expects. We will just construct the byte manually
    in the tx_buffer for transactions. According to the cc1101 datasheet we
    can see this header byte specified -

    Bit7   Bit6   Bit5-0
    R/W    Burst  Address(6 bits)
    1 = Read, 0 = write. 0 = single, 1 = burst. Burst bit will keep reading memory in a block every clock cycle. 
    See section 10.2 (Register Access) for more information
    ONLY send these bits if a data sheet specifies explicit phases such as -
    [Command phase]
    [Address phase]
    [Dummy clocks]
    [Data phase]
    Our data sheet just says to write normal data (i.e send this byte, then this byte...)

    - mode
    This is the SPI mode. From table 24 in the data sheet, I can see the mode is simply 0.
    - clock_speed_hz
    This is the clock speed (SCLK frequency). From figure 15 in the data sheet, I can see
    the max speed is 10 MHz for the cc1101. Anything below this should work. We should start smaller.
    - spics_io_num
    The GPIO pin we set CSN to (which is physically connected to 5 in our case)
    - queue_size
    Irrelevant if we use polling. spi_device_transmit and the method for polling transmit
    are synchronous so there is no queue. This is only when we queue transactions with
    spi_device_queue_trans() so ignore for now. 
    */ 
    spi_device_interface_config_t deviceConfig = {};
    spi_device_handle_t cc1101; // An arbitrary name we refer to initialzed as garbage. We pass in a pointer to allow the driver to modify this
    deviceConfig.command_bits = 0; // Only 1 phase to send bytes, no special phases
    deviceConfig.address_bits = 0;
    deviceConfig.dummy_bits = 0;
    deviceConfig.mode = 0; // As specified by the data sheet
    deviceConfig.clock_speed_hz = 1000000; // 1 MHz
    deviceConfig.spics_io_num = GPIO_NUM_5; // Physical GPIO for CSN
    deviceConfig.queue_size = 1; // We don't use queue, see above comments
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &deviceConfig, &cc1101));
    // =================== END CONFIGURE DEVICE SECTION ===================== //
    
    // =================== CONFIGURE TRANSACTION SECTION ==================== //
    strobe_reset(cc1101); // Send 1011 0000 to the slave to reset it
    /*
    Now, we are setting up a 'hello world' version of interacting with the cc1101.
    Let's confirm we can read values from the device. In the data sheet, I can see that 
    a static status register I can read from is the PARTNUM located at 0x30. The register
    for resetting the chip is also at this address. According to the data sheet, to access
    the status register at an overloaded register, we specify the burst bit as being 1. 

    Now we can construct our entire byte to send. 1 (Read) 1 (burtst) 110000 (0x30 address in 6 bits)
    Converted to hex, this is 0xF0. To receive an answer we must keep the master clock pulsing. We
    can send a dummy byte after the fact.
    */
    spi_transaction_t t = {};
    // The address of register is 0xF0. We will send this to the slave, as well as a dummy byte after.
    uint8_t tx[2] = {0xF0, 0x00}; 
    // Two bytes of buffer that we will receive back. A bit shifts in and out every clock cycle. The bytes inside are arbitrary.
    // According to the data sheet, we will always get back a status byte from cc1101 and then the response. 
    uint8_t rx[2] = {0x00, 0x00};
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    t.length = 16; // Two Bytes
    // Interrupt section
    // spi_device_queue_trans();
    // spi_device_get_trans_result();
    // spi_device_transmit();
    // Polling section
    // spi_device_polling_transmit();
    // spi_device_polling_start();
    // spi_device_polling_end();
    ESP_ERROR_CHECK(spi_device_polling_transmit(cc1101, &t));
    // We can then log the values of the two bytes that are now filled from the slave.
    ESP_LOGI("CC1101", "Status Byte: 0x%02X, PARTNUM Register value: 0x%02X", rx[0], rx[1]);

    spi_transaction_t version_register = {};
    uint8_t tx_v[2] = {0xF1, 0x00};
    uint8_t rx_v[2] = {0x00, 0x00};
    version_register.tx_buffer = tx_v;
    version_register.rx_buffer = rx_v;
    version_register.length = 16;

    ESP_ERROR_CHECK(spi_device_polling_transmit(cc1101, &version_register));
    ESP_LOGI("CC1101", "Status Byte: 0x%02X, VERSION Register value: 0x%02X", tx_v[0], rx_v[1]);

    /*
    In the Master Slave paradigm, the slave cannot send bits unless the master is clocking
    So in order to properly retrieve a register value, we first send the header byte telling 
    cc1101 what to do. We will receive the answer after a few clock pulses. But in order for
    use to receive it, the master still must be clocking. So we must send a dummy byte the 
    next clock pulse as well. 
    */
}