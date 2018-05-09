/*==============================================================================
>File name:  	stm32f10x.h
>Brief:         header for Init periph functions of projectONE	
>Author:        LL
>Date:          07.05.2018
>Version:       1.0
==============================================================================*/

#ifndef STM32_INITS_H_
#define STM32_INITS_H_

#include "Main.h"

#include "stm32f10x.h"					//STD_PERIPH_DRIVER
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_tim.h"		
#include "stm32f10x_exti.h"
#include "misc.h"						//interrupts and NVIC table

//----------------------------= [P]rototypes ; =--------------------------------
void SetSysClockTo_72(void);
void GPIO_init(void);

void USART1_init(void);
int8_t USART1_SEND(volatile char *pbuf);

int8_t USART2_init(void);
int8_t USART2_SEND(volatile char *pbuf);   

void SPI2_MASTER_init (void);
void SPI_Send(uint16_t data);	//send data via MOSI SPI									
int8_t SPI2_Read_BMI160 (uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);	//read data from reg_addr from BMI160 sensor
int8_t SPI2_Write_BMI160 (uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);	//write data to reg_addr to BMI160 sensor

#endif	/* STM32_INITS_H_ */
