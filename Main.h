/*==============================================================================
>File name:  	Main.h
>Brief:         Contains all projectONE setting via defines	
>Author:        LL
>Date:          07.05.2018
>Version:       1.0
==============================================================================*/

#ifndef MAIN_H_
#define MAIN_H_

#define SPI_SCK_PIN   GPIO_Pin_13     	// PB13		SCK
#define SPI_MISO_PIN  GPIO_Pin_14     	// PB14	    MISO
#define SPI_CSB_PIN   GPIO_Pin_12    	// PB12    	NSS
#define SPI_MOSI_PIN  GPIO_Pin_15	 	// PB15	    MOSI
#define SPI_GPIO_PORT GPIOB

#define RX_BUF_SIZE 80
#define TX_BUF_SIZE 80

#define	RTC_EN								//enable current time indication in terminal
#undef DMA_EN

#undef Madgwick
#undef Mahony
#define Madgwick 

#define __BYTE_ORDER__ __ORDER_LITTLE_ENDIAN__		//stm32f103 has Little endian byte order *watch test_byteorder func


#endif /* MAIN_H_ */
                                     
	




