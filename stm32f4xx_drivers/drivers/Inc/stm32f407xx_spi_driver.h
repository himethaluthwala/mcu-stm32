/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Aug 26, 2025
 *      Author: himethaluthwala
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_


#include "stm32f407xx.h"


/********************************************************************************/

/* SPI configuration structure */

/********************************************************************************/

typedef struct
{
	uint8_t SPI_DeviceMode;					// possible valves from @SPI_DEVICEMODES
	uint8_t SPI_BusConfig;					// possible values from @SPI_BUSCONFIG
	uint8_t SPI_SclkSpeed;					// possible values from @SPI_CLK_SPEED
	uint8_t SPI_DFF;						// possible values from @SPI_CPOL
	uint8_t SPI_CPOL;						// possible values from @SPI_CPHA
	uint8_t SPI_CPHA;						// possible values from @SPI_SSM
	uint8_t SPI_SSM;						// 0 or 1
}SPI_Config_t;


/********************************************************************************/

/* Handle structure for SPI peripheral */

/********************************************************************************/

typedef struct
{
	SPI_RegDef_t *pSPIx;					// pointer to hold the base address of SPI peripheral
	SPI_Config_t SPI_Config;				// structure that holds SPI configuration settings
}SPI_Handle_t;


/* @SPI_DEVICEMODES */

#define SPI_DEVICE_MODE_MASTER			1
#define SPI_DEVICE_MODE_SLAVE			0


/* @SPI_BUSCONFIG */

#define SPI_BUS_CONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2
#define SPI_BUS_CONFIG_S_RXONLY			3


/* @SPI_CLK_SPEED */

#define SPI_SCLK_SPEED_DIV2				0
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7


/* @SPI_DATA_FORMAT */

#define SPI_DFF_8BITS					0
#define SPI_DFF_16BITS					1


/* @SPI_CPOL */

#define SPI_CPOL_LOW					0
#define SPI_CPOL_HIGH					1


/* @SPI_CPHA */

#define SPI_CPHA_LOW					0
#define SPI_CPHA_HIGH					1


/* @SPI_SSM */

#define SPI_SSM_DI						0
#define SPI_SSM_EN						1


/********************************************************************************/

/* APIs supported by this driver */

/********************************************************************************/

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);



#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
