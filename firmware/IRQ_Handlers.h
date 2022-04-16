/*==============================================================================
>File name:  	IRQ_HANDLERS.h
>Brief:         Header for IRQ_HANDLERS.c	
>Author:        LL
>Date:          08.05.2018
>Version:       1.0
==============================================================================*/
#ifndef IRQ_HANDLERS_H_
#define IRQ_HANDLERS_H_

#include "Main.h"

#include <stdbool.h>
#include "stm32f10x_Inits.h"
#include "Menu.h"

extern volatile char USART1_RxBuf[RX_BUF_SIZE];		//buffer for data Recived from PC's Terminal 
extern volatile char USART2_RxBuf[RX_BUF_SIZE];
extern volatile char USART1_TxBuf[TX_BUF_SIZE];
extern volatile char USART2_TxBuf[TX_BUF_SIZE];

extern volatile char RX1i;		//index of data buffer
extern volatile char RX2i;
#endif /* IRQ_HANDLERS_H_ */
