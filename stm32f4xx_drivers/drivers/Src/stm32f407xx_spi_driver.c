/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Aug 26, 2025
 *      Author: himethaluthwala
 */

#include "stm32f407xx_spi_driver.h"


/*
 * Function name:			SPI_Init
 *
 * Description:				Initialises the SPI peripheral
 *
 * Parameter 1:				Handler that contains base address of the SPI peripheral and configuration settings
 *
 * Return:					None
 *
 * Notes:					None
 *
 */

void SPI_Init(SPI_Handle_t *pSPIHandle) {

	// configure the SPI_CR1 register

	uint32_t tempreg = 0;

	// configure the device mode
	tempreg |= (pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR);

	// configure bus config
	if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUSCONFIG_FD) {
		// bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	} else if (pSPIHandle->SPI_Config.SPI_DeviceMode == SPI_BUS_CONFIG_HD) {
		// bidi mode should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	} else if (pSPIHandle->SPI_Config.SPI_DeviceMode == SPI_BUS_CONFIG_S_RXONLY) {
		// bidi mode should be cleared and RXONLY bit must be set
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	// configure the SPI serial clock speed (baud rate)
	tempreg |= (pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR);

	// configure the DFF
	tempreg |= (pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF);

	// configure the CPOL
	tempreg |= (pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL);

	// configure the CPHA
	tempreg |= (pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA);

	pSPIHandle->pSPIx->CR1 = tempreg;
}


/*
 * Function name:			SPI_DeInit
 *
 * Description:				Deinitialises the SPI peripheral
 *
 * Parameter 1:				Base address of the SPI peripheral
 *
 * Return:					None
 *
 * Notes:					None
 *
 */

void SPI_DeInit(SPI_RegDef_t *pSPIx) {
	// TODO
}


/*
 * Function name:			SPI_PeriClockControl
 *
 * Description:				Enables the SPI peripheral clock
 *
 * Parameter 1:				Base address of the SPI peripheral
 * Parameter 2:				Enable (1) or disable (0)
 *
 * Return:					None
 *
 * Notes:					None
 *
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {

	if (EnorDi == ENABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI4)
		{
			SPI3_PCLK_EN();
		}
		else if (pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
	}
	else
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
		else if (pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}
	}
}

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len) {

}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len) {

}

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {

}
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority) {

}
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle) {

}
