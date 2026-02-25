/******************************************************************************
 * Author: Ryan Freas
 * Project: CC1101 SPI Interface (ESP-IDF)
 *
 * Purpose:
 * Basic Program to confirm your ESP-32 and cc1101 device are communicating properly
 * 
 * Pinout:
 * VCC -> 3.3v
 * CSN -> GPIO 5 (chip select, this can be any suitable GPIO pin)
 * MOSI -> GPIO 23 (VSPI MOSI)
 * GD02 -> No mapping 
 * GND -> GND
 * GD00 -> GPIO 4 (Any suitable GPIO)
 * SCK -> GPIO 18 (CLK Clock Signal)
 * MISO -> GPIO 19 (VSPI MISO)
 * 
 * cc1101 Byte Contract:
 * Bit 7   Bit 6   Bit 5-0
 * R/W     Burst    Address 
 * 
 * Further reading:
 * https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/spi_master.html
 * https://www.ti.com/lit/ds/symlink/cc1101.pdf
 * 
 ******************************************************************************/

#include <stdio.h>
#include "esp_log.h"

extern "C" {
#include "driver/spi_master.h"
#include "driver/spi_common.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

void strobe_reset(spi_device_handle_t cc1101) {
    spi_transaction_t reset_strobe = {};
    uint8_t tx[1]= {0x30}; // Target the command strobe (NON-status register) SRES (Reset Chip)
    uint8_t rx[1]= {0x00}; 
    reset_strobe.tx_buffer = tx;
    reset_strobe.rx_buffer = rx;
    reset_strobe.length = 8; 
    ESP_ERROR_CHECK(spi_device_polling_transmit(cc1101, &reset_strobe));

    spi_transaction_t idle = {};
    uint8_t tx_i[1]= {0x36}; // Target the idle strobe (NON-status register) SIDLE (Exit RX / TX)
    uint8_t rx_i[1]= {0x00}; 
    idle.tx_buffer = tx_i;
    idle.rx_buffer = rx_i;
    idle.length = 8; 
    ESP_ERROR_CHECK(spi_device_polling_transmit(cc1101, &idle)); // You can only flush in idle mode

    spi_transaction_t flush_tx = {};
    uint8_t tx_f[1]= {0x3B}; // Target the flush tx strobe (NON-status register) SFTX (Flush the TX FIFO buffer)
    uint8_t rx_f[1]= {0x00}; 
    flush_tx.tx_buffer = tx_f;
    flush_tx.rx_buffer = rx_f;
    flush_tx.length = 8; 
    ESP_ERROR_CHECK(spi_device_polling_transmit(cc1101, &flush_tx));
    ESP_LOGI("CC1101", "Status Byte: 0x%02X", rx_f[0]);
    // A sequence of resetting, idling, then flushing lets cc1101 have safe register values to continue.
    // Alternatively, we can just add a delay after the reset (vTaskDelay(pdMS_TO_TICKS(2))) and this 
    // accomplishes the same goal. Idling and flushing may not be necessary.
};

extern "C" void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI("MAIN", "Hello World...?");


    // ======================CONFIGURE BUS SECTION=========================== //
    spi_bus_config_t busConfig = {};
    busConfig.mosi_io_num = GPIO_NUM_23;
    busConfig.miso_io_num = GPIO_NUM_19;
    busConfig.sclk_io_num = GPIO_NUM_18;
    busConfig.quadwp_io_num = -1; 
    busConfig.quadhd_io_num = -1;
    // SPI3_HOST: Open SPI peripheral on the ESP-32 for VSPI pins
    // busConfig: Three standard Master/Slave signals that correspond to physical wiring
    // SPI_DMA_DISABLED: Simple config for small transfer size, no need for DMA
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &busConfig, SPI_DMA_DISABLED));
    // ===================== END CONFIGURE BUS SECTION ====================== //


    // ===================== CONFIGURE DEVICE SECTION ======================= //
    spi_device_interface_config_t deviceConfig = {};
    spi_device_handle_t cc1101; 
    deviceConfig.command_bits = 0; 
    deviceConfig.address_bits = 0;
    deviceConfig.dummy_bits = 0;
    deviceConfig.mode = 0;
    deviceConfig.clock_speed_hz = 1000000;
    deviceConfig.spics_io_num = GPIO_NUM_5;
    deviceConfig.queue_size = 1; 
    // {phase}_bits:
    // -    There is no delineation of bits to send in the datasheet, so we simply use a raw SPI transfer
    //      No command phase, no address phase, no dummy phase.
    // mode:
    // -    SPI clock mode. No clue how to derive this so I guessed lol.
    // clock_speed_hz:
    // -    The max clock speed as specified by the data sheet is 10 MHz
    // spics_io_num:
    // -    The pin we mapped CSN to
    // queue_size:
    // -    We are not using any async methods, so queue size is 1

    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &deviceConfig, &cc1101));
    // =================== END CONFIGURE DEVICE SECTION ===================== //
    

    // =================== CONFIGURE TRANSACTION SECTION ==================== //
    // We must reset the system when the power supply is turned on (see datasheet: 19.1 Power-On Start-Up Sequence)
    strobe_reset(cc1101);
    // We must provide a transaction struct when we communicate with the cc1101
    spi_transaction_t partnum_register = {};
    // 0xF0 is the header byte of the PARTNUM Register
    // 1 (Read) 1 (Burst Bit for Status Registers) 11 0000 (Address 0x30)
    uint8_t tx[2] = {0xF0, 0x00}; 
    // Filling out a buffer for the slave response
    uint8_t rx[2] = {0x00, 0x00};
    partnum_register.tx_buffer = tx;
    partnum_register.rx_buffer = rx;
    // We are sending two bytes (16 bits), the second is a garbage byte just to be able to receive the response
    partnum_register.length = 16; 
    ESP_ERROR_CHECK(spi_device_polling_transmit(cc1101, &partnum_register));
    // Expect NOT to see 0x00 for register value.  0x00 For Status Byte means OK.
    ESP_LOGI("CC1101", "Status Byte: 0x%02X, PARTNUM Register value: 0x%02X", rx[0], rx[1]);

    spi_transaction_t version_register = {};
    uint8_t tx_v[2] = {0xF1, 0x00};
    uint8_t rx_v[2] = {0x00, 0x00};
    version_register.tx_buffer = tx_v;
    version_register.rx_buffer = rx_v;
    version_register.length = 16;
    ESP_ERROR_CHECK(spi_device_polling_transmit(cc1101, &version_register));
    // Expect NOT to see 0x00 for register value.  0x00 For Status Byte means OK.
    ESP_LOGI("CC1101", "Status Byte: 0x%02X, VERSION Register value: 0x%02X", rx_v[0], rx_v[1]);
    // ================= END CONFIGURE TRANSACTION SECTION ================= //

}