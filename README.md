# ESP32-CC1101

## Introduction

This project demonstrates how to interface a CC1101 RF transceiver with an ESP32 using the ESP-IDF SPI Master driver. In this demo, we will be accessing a status register inside the CC1101 (and performing a few other operations). A successful read of this register will confirm we have set up our devices to communicate successfully. The relevant code can be found in `main/main.cpp`. 
> Note: We use ESP-IDF here for full control and learning purposes, but Arduino can be a simpler option for long-term development.

Writing this firmware can be accomplished in five steps:
- Acquiring prerequisites (hardware, software)
- Wiring the appropriate pins from the CC1101 to the ESP32
- Initialize an SPI bus using ESP-IDF  
- Register the CC1101 as a device on that bus  
- Perform SPI transactions to validate communication  

This README will reference the [ESP32 documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/spi_master.html) and the official [TI CC1101 transceiver datasheet](https://www.ti.com/lit/ds/symlink/cc1101.pdf). Basic programming experience, familiarity with the [SPI interface](https://www.analog.com/en/resources/analog-dialogue/articles/introduction-to-spi-interface.html), and development board knowledge (Raspberry Pi, Arduino, ESP32) will be helpful to follow along.


## Table of Contents

1. [Prerequisites](#1-prerequisites)
   - [Hardware](#hardware)
   - [Software](#software)
2. [Wiring](#2-wiring)
3. [Initialize an SPI Bus](#3-initialize-an-spi-bus)
   - [Method: `spi_bus_initialize()`](#method-spi_bus_initialize)
   - [Determining Values](#determining-spi_bus_initialize-parameters)
4. [Register a Device](#4-register-a-device)
   - [Method: `spi_bus_add_device()`](#method-spi_bus_add_device)
   - [Determining Values](#determining-spi_bus_add_device-parameters)
5. [Register Access in the CC1101](#5-register-access-in-the-cc1101)
    - [SPI Accessible Types](#spi-accessible-types)
   - [Expected Transmit Format](#expected-transmit-format)
6. [Interact with the Device](#6-interact-with-the-device)
   - [Method: `spi_device_polling_transmit()`](#method-spi_device_polling_transmit)
   - [Determining Values](#determining-spi_device_polling_transmit-parameters)
   - [CC1101 Initialization Procedure](#cc1101-initialization-procedure)

# 1. Prerequisites

## Hardware

- CC1101 transceiver: The CC1101 is a low cost, low power sub-1 GHz RF transceiver designed for wireless applications in the 300-348 MHz, 387-464 MHz, and 779-928 MHz ISM/SRD bands. Commonly used with microcontrollers like Arduino, ESP8266, and the Flipper Zero for sub-GHz communication, it supports FSK, GFSK, MSK, and ASK modulation.

- ESP32: The ESP32 is a microcontroller with integrated Wi-Fi and Bluetooth, manufactured by Espressif Systems. It is widely used in IoT (Internet of Things) projects due to its powerful 32-bit dual-core processor, high performance, and versatility in smart home and wearable devices.

- Breadboard wires: Breadboard wires (or jumper wires) are flexible or solid core wires with pins on the ends used to create temporary, solderless connections on a breadboard for prototyping circuits.
## Software

- [Visual Studio Code](https://code.visualstudio.com/): A lightweight, extensible source code editor used to write and manage ESP32 projects.

- [ESP-IDF (Espressif IoT Development Framework)](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/): The official development framework for ESP32, providing the toolchain, build system, drivers, and APIs.

- [Espressif IDF Extension for VS Code](https://marketplace.visualstudio.com/items?itemName=espressif.esp-idf-extension): An extension that integrates ESP-IDF into VS Code, enabling build, flash, monitor, and project management features.
- [Python 3.x](https://www.python.org): Ensure your Python version is compatible with the latest version of ESP-IDF.

# 2. Wiring

### CC1101 Pinout
 ![CC1101 Pinout](assets/cc1101_pinout.png)
### ESP32 pinout
 ![ESP32 Pinout](assets/esp32_pinout.png)

> Note: The pinout can change based on the type of ESP32 and CC1101 module you own. Verify your board's pinout before wiring.

### Required SPI Connections

| CC1101 Pin | ESP32 Pin |
|------------|-----------|
| VCC        | 3.3V      |
| GND        | GND       |
| CSn        | GPIO 5    |
| MOSI (SI)  | GPIO 23   |
| MISO (SO)  | GPIO 19   |
| SCK        | GPIO 18   |
| GDO0       | GPIO 4    |
| GDO2       | Optional / Not Used |
> [!WARNING]
> Ensure VCC on the CC1101 is connected to 3.3V only. Applying 5V can damage the chip.

Once everything is wired up and the prerequisites are complete, we can begin writing firmware using ESP-IDF. If you're new to ESP-IDF, there are several videos online that can assist with setting up a project from scratch. I recommend watching [this one](https://www.youtube.com/watch?v=oHHOCdmLiII) and reading the official documentation for [getting started with ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html). After your environment is ready, open `main.cpp` and we will begin implementing the SPI configuration.
# 3. Initialize an SPI Bus

### Method: `spi_bus_initialize()`

We initialize the SPI bus using:

- [`spi_bus_initialize()`](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/spi_master.html#_CPPv418spi_bus_initialize17spi_host_device_tPK16spi_bus_config_t14spi_dma_chan_t)

This function requires:

- [`spi_host_device_t`](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/spi_master.html#_CPPv417spi_host_device_t)
- [`spi_bus_config_t`](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/spi_master.html#_CPPv416spi_bus_config_t)
- [`spi_dma_chan_t`](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/spi_master.html#_CPPv414spi_dma_chan_t)

```cpp
#include <iostream>
#include "esp_log.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"


extern "C" void app_main(void) {
    spi_bus_config_t busConfig = {};
    busConfig.mosi_io_num = GPIO_NUM_23;
    busConfig.miso_io_num = GPIO_NUM_19;
    busConfig.sclk_io_num = GPIO_NUM_18;
    busConfig.quadwp_io_num = -1; 
    busConfig.quadhd_io_num = -1;
    spi_bus_initialize(SPI3_HOST, &busConfig, SPI_DMA_DISABLED);
    ...
}
```
> Note: We can wrap our spi functions with an error handler to ensure errors propagate correctly and get logged to the console such as: `ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &busConfig, SPI_DMA_DISABLED));`.

### Determining spi_bus_initialize parameters
- SPI3_HOST
    - The SPI peripheral you are selecting. There are four SPI peripherals on the classic ESP32. Two are tied to internal ESP32 operations, while `SPI2_HOST` and `SPI3_HOST` are available for public interfacing.
- busConfig
    - mosi_io_num: The GPIO pin that connects the MOSI pin.
    - miso_io_num: The GPIO pin that connects the MISO pin.
    - sclk_io_num: The GPIO pin that connects the SCLK pin.
    - quadwp & quadhd: Set to -1 indicating we are not using these.
- SPI_DMA_DISABLED
    - Controls whether the SPI driver uses Direct Memory Access for transfers. DMA can be disabled for small and simple transfers. 
> Note: If we were to set SPI_DMA_CH_AUTO, we would have to change how we manage memory such as using `uint8_t* tx = (uint8_t*) heap_caps_malloc(64, MALLOC_CAP_DMA);`. Further reading on DMA is recommended.

# 4. Register a Device 

### Method: `spi_bus_add_device()`

We register the CC1101 using:

- [`spi_bus_add_device()`](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/spi_master.html#_CPPv418spi_bus_add_device17spi_host_device_tPK29spi_device_interface_config_tP19spi_device_handle_t)

This function requires:

- [`spi_host_device_t`](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/spi_master.html#_CPPv417spi_host_device_t)
- [`spi_device_interface_config_t`](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/spi_master.html#_CPPv429spi_device_interface_config_t)
- [`spi_device_handle_t`](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/spi_master.html#_CPPv419spi_device_handle_t)


```cpp

extern "C" void app_main(void) {
    ...
    spi_device_interface_config_t deviceConfig = {};
    spi_device_handle_t cc1101; 
    deviceConfig.command_bits = 0; 
    deviceConfig.address_bits = 0;
    deviceConfig.dummy_bits = 0;
    deviceConfig.clock_speed_hz = 1000000;
    deviceConfig.spics_io_num = GPIO_NUM_5;
    deviceConfig.queue_size = 1; 
    deviceConfig.mode = 0;

    spi_bus_add_device(SPI3_HOST, &deviceConfig, &cc1101);
    ...
}
```

###  Determining spi_bus_add_device parameters

- SPI3_HOST
    - Use the same host that you specified in the bus configuration step.
- deviceConfig
    - command, address, & dummy bits: The CC1101 does not have any phases specified in a transfer (see section 10: 4-wire Serial Configuration and Data Interface in the CC1101 datasheet).
    - clock_speed_hz: Table 22 in the CC1101 datasheet specifies the max frequency as 6-10 MHz depending on the action. This value should be lower than that.
    - spics_io_num: The GPIO pin we wired CSn to. Use -1 if you want to control the chip select manually. If you are to control it manually, read section 10 of the CC1101 datasheet where it specifies the CSn pin values.
    - queue_size: Set to 1 as our program is only using synchronous methods (such as `spi_device_polling_transmit()`).
    - mode: The SPI mode is determined by a combination of the Clock Polarity (CPOL) and the Clock Phase (CPHA). From the diagram (figure 15 in the CC1101 datasheet), we can see the SCLK line starts and idles low. So the CPOL is zero. We can also see that the lines indicate data is sampled on the rising edge of the SCLK signal, meaning the CPHA is zero. A combination of CPOL = 0 and CPHA = 0 means the SPI mode is 0. ![CC1101 Pinout](assets/timing_transfer.png)
- cc1101
    - An arbitrary name that should correspond to the device you are using. We will reference this in our transactions. 

# 5. Register Access in the CC1101

### SPI accessible types
The CC1101 exposes three main SPI-accessible types: configuration registers, status registers, and command strobes. Configuration registers (`0x00–0x2E`) are read/write and control radio parameters like frequency, modulation, and packet behavior. Status registers (`0x30–0x3D` when accessed with Burst=`1`) are read-only and report internal state information such as `PARTNUM`, `VERSION`, `RSSI`, and `FIFO` status. Command strobes (`0x30–0x3D` when accessed with Burst=`0`) are not registers, but actually single-byte instructions that immediately trigger actions inside the radio, such as reset (`SRES`), enter RX (`SRX`), enter TX (`STX`), or flush FIFOs (`SFTX/SFRX`). See the datasheet sections on FIFO and burst transfers for multi-byte transactions.

### Expected Transmit Format
The CC1101 does not have separate phases for sending bytes (no separate command phase, address phase, etc). It shifts a single bit in and out of the MISO and MOSI lines every clock pulse. The CC1101 expects our transmit buffer to follow this format: 

| Bit Position | Field Name | Width | Description | Values |
|--------------|------------|--------|------------|--------|
| 7 | R/W | 1 bit | Determines if operation is read or write | `0` = Write<br>`1` = Read |
| 6 | Burst | 1 bit | Determines single or multi-byte access | `0` = Single access<br>`1` = Burst access |
| 5–0 | Address | 6 bits | Register address or command strobe | `0x00 – 0x3F` |

- Bit position 7 tells the CC1101 if we are reading an address or writing to an address.
- Bit position 6 specifies if we are using single or multi-byte access. There is also a special use case for this bit: if a register is overloaded, this specifies if we want to access the value at a status register by setting the bit to `1` or we want to send a command strobe by setting this bit to `0`. 
    - For example, address `0x30` contains both the command strobe for resetting the device and the location where the `PARTNUM` value lives. If we just send the byte `0x30`, we would activate the reset sequence on the device. If we send the byte `0xF0` (which is `0x30` with a burst bit set to `1` at bit 6), we would receive back the `PARTNUM` value. 
- Bit position 5-0 is the address that we want to interact with. The first two bits in the byte address are not included and replaced by the R/W and burst bits. Below are some relevant addresses with different command strobes (Table 42) and status register values (Table 44). 

<div align="center">
<strong>Table 42: Command Strobes</strong><br>
<img src="assets/command_strobes.png" width="600"/>
<br><br>
<strong>Table 44: Status Registers</strong><br>
<img src="assets/status_registers.png" width="600"/>

</div>

> [!IMPORTANT]
> When transmitting bytes to the CC1101, the device will always respond with a Chip Status Byte when it receives data from the master. Since the SPI protocol is a full duplex, the slave can only send bits while the master clocks it. 
> For more information, see sections 10.1 and 10.2 on the CC1101 datasheet. Further reading about the [SPI protocol](https://www.analog.com/en/resources/analog-dialogue/articles/introduction-to-spi-interface.html) is recommended if a full-duplex SPI is unfamiliar.
> 
>  Example: To read the value in the `PARTNUM` register, we would send `1` (read) `1` (burst bit for overloaded register) `110000` (the address where this register is located). `1111 0000` = `0xF0`. Since we can only receive bits while the master is transmitting, we would send two bytes `0xF0 0x00` and receive two bytes corresponding to the Chip Status Byte and the actual register value. Sending only the byte `0xF0` would give us the chip status byte, but not the actual register value. `0x00` functions as a dummy byte meant to give time (clock-pulses) for the slave to send back the requested data.
# 6. Interact with the Device

### Method: `spi_device_polling_transmit()`

We perform transactions using:

- [`spi_device_polling_transmit()`](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/spi_master.html#_CPPv427spi_device_polling_transmit19spi_device_handle_tP17spi_transaction_t)

This function requires:

- [`spi_device_handle_t`](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/spi_master.html#_CPPv419spi_device_handle_t)
- [`spi_transaction_t`](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/spi_master.html#_CPPv417spi_transaction_t)

```cpp

extern "C" void app_main(void) {
    ...
    spi_transaction_t version_register = {};
    uint8_t tx_v[2] = {0xF1, 0x00};
    uint8_t rx_v[2] = {0x00, 0x00};
    version_register.tx_buffer = tx_v;
    version_register.rx_buffer = rx_v;
    version_register.length = 16;
    spi_device_polling_transmit(cc1101, &version_register);
    ...
}
```
> Note: This functionality has been refactored into helper functions in `main.cpp`.
### Determining spi_device_polling_transmit parameters 
- cc1101
    - The device name we created earlier in our process.
- version_register
    - tx_v: The Bytes we want to send to the CC1101. our first byte is `0xF1`. This corresponds to the `VERSION` register in the CC1101 (see Table 44 above or in the datasheet). The second byte is a dummy byte used to clock out the register value from the slave. This needs to be included, as every status register read will return two bytes: a chip status byte and the register value byte. In order to receive 2 bytes, we must send 2 bytes as well (due to the nature of the SPI protocol being a full-duplex).
    - rx_v: This buffer will be filled with the response of the slave. Again, we include two bytes in the buffer because that is what we expect to receive when we send two bytes.
    - length: We are sending two bytes, so that equals 16 bits.

After calling this method, simply logging out the version_register receive buffer will show us the value contained inside the `VERSION` register. As stated before, the first byte is a chip status byte. So we will receive a chip status byte located in rx_v[0] and the actual register value in rx_v[1]. The expected value in the `VERSION` register will be `0x14`. 

### CC1101 Initialization Procedure
Section 19.1 of the datasheet specifies the required sequence for powering up the CC1101. The system must be reset every time you turn on the power supply.

The government-approved method of accomplishing this is as follows:
- Pull CSn LOW, then drive it HIGH again
- Wait for MISO to go LOW
- Send `SRES`

This would require you to set spics_io_num to -1 when adding a device to the bus. Then, you would have to control the CSn manually. As specified by the data sheet in section 10.1, CSn will have to stay pulled low during any SPI transaction. This behavior is visualized in figure 15 of the datasheet and displayed above (in `assets/transfer_timing.png`)

> [!TIP]
> Alternatively, you can try to send the `SRES` strobe right away. After sending `SRES`, you can either wait a few ms for the crystal oscillator to stabilize, or you can follow by flushing the transmit buffer (which you can only do in idle mode) as there are some cases where the system starts in a state with `TXFIFO_UNDERFLOW` (see Table 23 in the datasheet). So the entire startup sequence will be to send the command strobes `SRES`, `SIDLE`, and `SFTX` in that order. After this sequence, your device should be ready to use. See `strobe_reset` in `main.cpp`.



































