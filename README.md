# STM32-Buoy
STM32 Nucleo-L496ZG-P board using an ICM20948 9-axis sensor to record, process, and transmit wave spectra data.

# Dependencies
https://github.com/TExley/STM32-ICM20948-Driver
https://github.com/LonelyWolf/stm32/tree/master/nrf24l01

# Clock Configuration
LPUART1 set to LSE.

# Pinout & Configuration
LPUART1 enabled in asynchronous mode with a 9600b/s baud rate.<br>
I2C1 enabled in I2C mode at 100kHz.
