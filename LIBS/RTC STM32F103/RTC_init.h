/*------------------------------------------------------------------------------
Библиотека подключения часов реального времени (RTC) к проекту. 
12.01.2018
версия 1.0
функции: 
- инизализации RTC (тактирование от внешнего нискочастотного кварца LSE 
32768 Гц);
- перевода значений счетчика часов в формат даты и времени;
- обратный перевод даты и времени в значение для счетчика;
- преобразование в формат дней недели,месяца,года,времени
------------------------------------------------------------------------------*/

#ifndef RTC_INIT_H_
#define RTC_INIT_H_

#include "stm32f10x_rtc.h"
#include "stm32f10x_pwr.h"
#include "stm32f10x.h"
#include <string.h>
#include <stdio.h>

//структура данных с часов
typedef struct
{
    uint8_t RTC_Hours;
    uint8_t RTC_Minutes;
    uint8_t RTC_Seconds;
    uint8_t RTC_Date;
    uint8_t RTC_Wday;
    uint8_t RTC_Month;
    uint16_t RTC_Year;
} RTC_DateTimeTypeDef;
 
//>>>>>>>>>>>>>>>>>>>>>>>> Прототипы функций <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
/**
  * @brief   init RTC with Internal RC generator
  * @note   
  * @param  
  * @retval 1 - SUCCESS, 0 - FAILURE
  */
unsigned char RTC_Init_LSI(void);
/**
  * @brief   init RTC with EXTERNAL OSCILLATOR 32768 HZ
  * @note   
  * @param  
  * @retval 1 - SUCCESS, 0 - FAILURE
  */
unsigned char RTC_Init_LSE(void);
/**
  * @brief   Reads data from RTC and fill sctructure with its values
  * @note   
  * @param  *pbuf - pointer to structure to fill with time values
  * @retval 1 - SUCCESS, 0 - FAILURE
  */
_Bool RTC_GetTime (RTC_DateTimeTypeDef *pbuf);

// Get current date
void RTC_GetDateTime(uint32_t RTC_Counter, RTC_DateTimeTypeDef* RTC_DateTimeStruct);
// Convert Date to Counter
uint32_t RTC_GetRTC_Counter(RTC_DateTimeTypeDef* RTC_DateTimeStruct);
// Функция генерирует в буфере дату собственного формата
void RTC_GetMyFormat(RTC_DateTimeTypeDef* RTC_DateTimeStruct, char * buffer);

/*>>>>>>>>>>>>>>>>>>>>>>>> ПРИМЕР ИСПОЛЬЗОВАНИЯ <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

int main(void)
{
    char RTC_buffer[80] = {'\0'};
    RTC_DateTimeTypeDef RTC_DateTime;
 
    SetSysClockToHSE();
    usart_init();
 
    if (RTC_Init_LSE() == 1) 
	{
        // Если первая инициализация RTC устанавливаем начальную дату, например 22.09.2016 14:30:00
        RTC_DateTime.RTC_Date = 22;
        RTC_DateTime.RTC_Month = 9;
        RTC_DateTime.RTC_Year = 2016;
 
        RTC_DateTime.RTC_Hours = 14;
        RTC_DateTime.RTC_Minutes = 30;
        RTC_DateTime.RTC_Seconds = 00;
		
		RTC_WaitForLastTask();
        RTC_SetCounter(RTC_GetRTC_Counter(&RTC_DateTime));
    }
 
    while(1)
    {
        RTC_GetTime(&RTC_DateTime);
		sprintf(RTC_buffer, "  %02d:%02d:%02d : ",       
        RTC_DateTime.RTC_Hours, RTC_DateTime.RTC_Minutes, RTC_DateTime.RTC_Seconds);
		USART1_SEND(RTC_buffer);

    }
}
*/
#endif	/* RTC_INIT_H_ */
/* END OF FILE */
