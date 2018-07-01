/*==============================================================================
>File name:  	Main.h
>Brief:         Contains all projectONE setting via defines	
                You can choose initialization style of: 
                - USART1  /  USART1_TX + DMAch4
                - USART2  /  USART2_TX + DMAch7
                - SPI2    /  SPI2_TX + DMAch5  SPI2_RX + DMAch4
                using define/undef mechanics (watch lines 43,44).
                NOTE!!Be carefull cuz SPI2_RX and USART1_TX uses same DMA channel
                
                Also it is possible to activate (using line 42):
                - RealTimeClock
                
                In additional feature different filters cant be used:
                - Madgwick
                - Mahony
                watch lines 55,56
                
>Author:        LL
>Date:          07.05.2018
>Version:       1.0
==============================================================================*/

#ifndef MAIN_H_
#define MAIN_H_

#define SPI_SCK_PIN   		GPIO_Pin_13     // 	PB13	SCK
#define SPI_MISO_PIN  		GPIO_Pin_14     // 	PB14	MISO
#define BMI160_CSB_PIN   	GPIO_Pin_12    	// 	PB12    BMI160 NSS
#define SPI_MOSI_PIN  		GPIO_Pin_15	 	// 	PB15	MOSI
#define SPI_GPIO_PORT 		GPIOB

#define BMM150_CSB_PIN		GPIO_Pin_1		//	PA1		BMM150 NSS
#define BMM150_CSB_PORT		GPIOA

#define DEBUG_PIN			GPIO_Pin_0		//	PA0		DEBUG PIN
#define DEBUG_PORT			GPIOA

#define LED_PIN				GPIO_Pin_13		//	PC13	LED PIN
#define LED_PORT			GPIOC

#define BLE_STATE_PIN		GPIO_Pin_1		//	PB1		BLE STATE PIN
#define BLE_STATE_PORT		GPIOB

#define INT1_EXTI_PIN       GPIO_Pin_0      //  PB0     INT1 from BMI160
#define INT1_EXTI_PORT      GPIOB

#define RX_BUF_SIZE 80
#define TX_BUF_SIZE 80
#define DMA_BUF_SIZE 80

#define	RTC_EN								//enable current time indication in terminal
#define DMA_USART2_EN                       //enable USART2_TX via DMA controller
#define DMA_USART1_EN                       //enable USART1_TX via DMA controller

#define Madgwick                            //enable Madgwick filter    
#undef  Mahony                              //enable Mahony filter                 

#define COMPENSATE_MAGNI                    //enable temperature compensation of magnetometr values

#define __BYTE_ORDER__ __ORDER_LITTLE_ENDIAN__		//stm32f103 has Little endian byte order *watch test_byteorder func


#endif /* MAIN_H_ */
     
     
     








