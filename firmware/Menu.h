/*==============================================================================
>File name:  	Menu.h
>Brief:         Header for Menu.c display MainMenu via USART by Terminal	
>Author:        LL
>Date:          07.05.2018
>Version:       1.0
==============================================================================*/
#ifndef MAIN_MENU_H_
#define MAIN_MENU_H_

#include "Main.h"

#include <string.h>					
#include <stdio.h>
#include <stdbool.h>
#include <ctype.h>

#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "Sensors_func.h"
#include "HC-05.h"
#include "RTC_init.h"

typedef uint8_t (*typedef_pSendfunc)(volatile char *pbuf);

extern volatile bool RX1_FLAG_END_LINE;		//message from terminal comes
extern volatile bool FLAG_END_LINE;
extern volatile bool RX2_FLAG_END_LINE;		//message-echo from hc-05 comes

struct MainMenu_Struct
{
USART_InitTypeDef* usart2Struct;			//pointer at USART2 config structure
typedef_pSendfunc terminalSend;				//pointer at Send to terminal function
typedef_pSendfunc hc05Send;					//pointer at Send to HC-05 function			
};

//----------------------------= ;[P]rototypes =---------------------------------

void MainMenu_startMess (struct MainMenu_Struct *MainMenu);		//Sends to terminal starting messages
void MainMenu_checkAnswer (struct MainMenu_Struct *MainMenu);	//Watching if answer from HC-05 comes
void MainMenu_checkInput (struct MainMenu_Struct *MainMenu);	//Watching for input stream (via terminal) from PC
void str_toupper (char *str);
void Clear_Buffer(volatile char *pbuf);
static void Display_Array (typedef_pSendfunc send,char *arr[]);		//Displays array in terminal
static void Display_Time (typedef_pSendfunc send);						//Displays current time if RTC initialized in terminal
_Bool MainMenu_RTC_init (void);									//Initialize RTC for HC-05 terminal use
static inline void bmi160_menu (struct MainMenu_Struct *MainMenu);
static inline void GyroDatarate_menu (struct MainMenu_Struct *MainMenu);
static inline void GyroBW_menu (struct MainMenu_Struct *MainMenu);
static inline void GyroRange_menu (struct MainMenu_Struct *MainMenu);
static inline void AccelDatarate_menu (struct MainMenu_Struct *MainMenu);
static inline void AccelBW_menu (struct MainMenu_Struct *MainMenu);
static inline void AccelRange_menu (struct MainMenu_Struct *MainMenu);
static inline void USART2_menu (struct MainMenu_Struct *MainMenu);
static inline void BLE_menu (struct MainMenu_Struct *MainMenu);
static inline void bmm150_menu (struct MainMenu_Struct *MainMenu);
static inline void bmm150Preset_menu (struct MainMenu_Struct *MainMenu);
static inline void bmm150Datarate_menu (struct MainMenu_Struct *MainMenu);
static inline void bmm150XYrep_menu (struct MainMenu_Struct *MainMenu);
static inline void bmm150Zrep_menu (struct MainMenu_Struct *MainMenu);
static inline void Main_menu (struct MainMenu_Struct *MainMenu);

#endif	/* MAIN_MENU_H_ */
