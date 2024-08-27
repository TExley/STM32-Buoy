#include "nrf24_hal.h"

// SPI peripheral
SPI_HandleTypeDef* spih;

// Set the GPIO pins of the nRF24L01 transceiver
void nRF24_HAL_Init(SPI_HandleTypeDef* hspi) {
	spih = hspi;
	nRF24_CSN_H;
	nRF24_CE_L;
}

// Low level SPI transmit/receive function (hardware depended)
// input:
//   data - value to transmit via SPI
// return: value received from SPI
uint8_t nRF24_LL_RW(uint8_t data) {
	uint8_t rxData;

	HAL_SPI_TransmitReceive(spih, &data, &rxData, 1, nRF24_SPI_TIMEOUT);

	// Return received byte
	return rxData;
}
