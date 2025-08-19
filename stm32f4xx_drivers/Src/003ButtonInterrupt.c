/*
 * 003ButtonInterrupt.c
 *
 *  Created on: Aug 19, 2025
 *      Author: himethaluthwala
 */

#include <string.h>
#include "stm32f407xx.h"

#define BTN_PRESSED		1


void delay(void)
{
	for (uint32_t i = 0; i < 250000; i++);	// gives ~200ms delay with system clock of 16 MHz
}


int main(void)
{
	GPIO_Handle_t GPIO_LED, GPIO_Btn;

	// clear memory values of structures
	memset(&GPIO_LED, 0, sizeof(GPIO_LED));
	memset(&GPIO_Btn, 0, sizeof(GPIO_Btn));

	// Green LED connected to PD12
	GPIO_LED.pGPIOx = GPIOD;
	GPIO_LED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_LED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_LED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_LED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_LED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GPIO_LED);

	// User button connected to PA0
	GPIO_Btn.pGPIOx = GPIOA;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;		// rising edge trigger since user button pulled to high when pressed
	GPIO_Btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GPIO_Btn);

	// Button IRQ configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);

	return 0;
}

void EXTI0_IRQHandler(void)
{
	delay();	// 200ms delay to avoid button de-bouncing
	GPIO_IRQHandling(GPIO_PIN_NO_0);
	GPIO_ToggleOuputPin(GPIOD, GPIO_PIN_NO_12);
}
