#ifndef __NRF24_HAL_H
#define __NRF24_HAL_H

// Author's page:
// https://github.com/LonelyWolf/stm32/tree/master/nrf24l01

// Hardware abstraction layer for NRF24L01+ transceiver (hardware depended functions)
// GPIO pins definition
// GPIO pins initialization and control functions
// SPI transmit functions


// Peripheral libraries
#include "stm32l4xx_hal.h"

// CE (chip enable) pin (PB11)
#define nRF24_CE_PORT              GPIOD
#define nRF24_CE_PIN               GPIO_PIN_14
#define nRF24_CE_L                 HAL_GPIO_WritePin(nRF24_CE_PORT, nRF24_CE_PIN, GPIO_PIN_RESET)
#define nRF24_CE_H                 HAL_GPIO_WritePin(nRF24_CE_PORT, nRF24_CE_PIN, GPIO_PIN_SET)

// CSN (chip select negative) pin (PB12)
#define nRF24_CSN_PORT             GPIOD
#define nRF24_CSN_PIN              GPIO_PIN_15
#define nRF24_CSN_L                HAL_GPIO_WritePin(nRF24_CSN_PORT, nRF24_CSN_PIN, GPIO_PIN_RESET)
#define nRF24_CSN_H                HAL_GPIO_WritePin(nRF24_CSN_PORT, nRF24_CSN_PIN, GPIO_PIN_SET)

// IRQ pin (PB10)
#define nRF24_IRQ_PORT             GPIOC
#define nRF24_IRQ_PIN              GPIO_PIN_0

// Macros for the RX on/off
#define nRF24_RX_ON                nRF24_CE_H
#define nRF24_RX_OFF               nRF24_CE_L

#define nRF24_SPI_TIMEOUT          (uint32_t) 20

// Function prototypes
void nRF24_HAL_Init(SPI_HandleTypeDef* hspi);
uint8_t nRF24_LL_RW(uint8_t data);

#endif // __NRF24_HAL_H
