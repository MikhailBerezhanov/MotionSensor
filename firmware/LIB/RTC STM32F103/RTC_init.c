/*==============================================================================
>Project name:  RTC driver for STM32F103
>Brief:         
>Author:        
>Date:          23.04.2018
>Version:       2.0
==============================================================================*/
#include "RTC_init.h"

static uint32_t RTC_Counter = 0;	//counter of RTC hardware

/**
  * @brief   init RTC with Internal RC generator
  * @note   
  * @param  
  * @retval 1 - SUCCESS, 0 - FAILURE
  */
unsigned char RTC_Init_LSI(void)
{	
    // Включить тактирование модулей управления питанием и управлением резервной областью
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
    // Разрешить доступ к области резервных данных
    PWR_BackupAccessCmd(ENABLE);
    // Если RTC выключен - инициализировать
    if ((RCC->BDCR & RCC_BDCR_RTCEN) != RCC_BDCR_RTCEN)
    {
        // Сброс данных в резервной области
        RCC_BackupResetCmd(ENABLE);
        RCC_BackupResetCmd(DISABLE);
 
        // Установить источник тактирования внутренний кварц LSI
		RCC_LSICmd(ENABLE);
        while ((RCC->CSR & RCC_CSR_LSIRDY) == 0) {}
        RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
		
		// Включаем RTC
        RCC_RTCCLKCmd(ENABLE);
			
		RTC_ClearFlag(RTC_FLAG_RSF);	//Дропнуть флаг синхонизации
		RTC_WaitForSynchro();			//подождать синхронизацию регистров
			
        RTC_SetPrescaler(0x7FFF); // Устанавливаем делитель, чтобы часы считали секунды
									//32767 для счета секунд
		//PWR_BackupAccessCmd(DISABLE);
			
        return 1;
    }
    else return 0;
}
//==============================================================================

/**
  * @brief   init RTC with External OSCILLATOR 32768 HZ
  * @note   
  * @param  
  * @retval 1 - SUCCESS, 0 - FAILURE
  */
unsigned char RTC_Init_LSE(void)
{	
	// Включить тактирование модулей управления питанием и управлением резервной областью
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
    // Разрешить доступ к области резервных данных
    PWR_BackupAccessCmd(ENABLE);
    // Если RTC выключен - инициализировать
    if ((RCC->BDCR & RCC_BDCR_RTCEN) != RCC_BDCR_RTCEN)
    {
     //выполнить сброс области резервных данных
      RCC_BackupResetCmd(ENABLE);
      RCC_BackupResetCmd(DISABLE);

      // Установить источник тактирования кварц 32768
        RCC_LSEConfig(RCC_LSE_ON);
        while ((RCC->BDCR & RCC_BDCR_LSERDY) != RCC_BDCR_LSERDY) ;
            
        RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
        
        RCC_RTCCLKCmd(ENABLE);
		
		RTC_ClearFlag(RTC_FLAG_RSF);	
		RTC_WaitForSynchro();			//подождать синхронизацию регистров
			
        RTC_SetPrescaler(0x7FFF); 		// Установить делитель
          
        return 1;
   }
   else return 0;
}

_Bool RTC_GetTime (RTC_DateTimeTypeDef *pbuf)
{
RTC_Counter = RTC_GetCounter();	
RTC_GetDateTime(RTC_Counter, pbuf);
return SUCCESS;
}


//==============================================================================
/*
* Как конвертировать дату можно прочитать здесь:
* https://ru.m.wikipedia.org/wiki/%D0%AE%D0%BB%D0%B8%D0%B0%D0%BD%D1%81%D0%BA%D0%B0%D1%8F_%D0%B4%D0%B0%D1%82%D0%B0
*/
 
// (UnixTime = 00:00:00 01.01.1970 = JD0 = 2440588)
#define JULIAN_DATE_BASE    2440588
 
//------------------------------Get current date--------------------------------
void RTC_GetDateTime(uint32_t RTC_Counter, RTC_DateTimeTypeDef* RTC_DateTimeStruct) 
{
    unsigned long time;
    unsigned long t1, a, b, c, d, e, m;
    int year = 0;
    int mon = 0;
    int wday = 0;
    int mday = 0;
    int hour = 0;
    int min = 0;
    int sec = 0;
    uint64_t jd = 0;;
    uint64_t jdn = 0;
 
    jd = ((RTC_Counter+43200)/(86400>>1)) + (2440587<<1) + 1;
    jdn = jd>>1;
 
    time = RTC_Counter;
    t1 = time/60;
    sec = time - t1*60;
 
    time = t1;
    t1 = time/60;
    min = time - t1*60;
 
    time = t1;
    t1 = time/24;
    hour = time - t1*24;
 
    wday = jdn%7;
 
    a = jdn + 32044;
    b = (4*a+3)/146097;
    c = a - (146097*b)/4;
    d = (4*c+3)/1461;
    e = c - (1461*d)/4;
    m = (5*e+2)/153;
    mday = e - (153*m+2)/5 + 1;
    mon = m + 3 - 12*(m/10);
    year = 100*b + d - 4800 + (m/10);
 
    RTC_DateTimeStruct->RTC_Year = year;
    RTC_DateTimeStruct->RTC_Month = mon;
    RTC_DateTimeStruct->RTC_Date = mday;
    RTC_DateTimeStruct->RTC_Hours = hour;
    RTC_DateTimeStruct->RTC_Minutes = min;
    RTC_DateTimeStruct->RTC_Seconds = sec;
    RTC_DateTimeStruct->RTC_Wday = wday;
}
 
//---------------------------Convert Date to Counter----------------------------
uint32_t RTC_GetRTC_Counter(RTC_DateTimeTypeDef* RTC_DateTimeStruct) 
{
    uint8_t a;
    uint16_t y;
    uint8_t m;
    uint32_t JDN;
 
    a=(14-RTC_DateTimeStruct->RTC_Month)/12;
    y=RTC_DateTimeStruct->RTC_Year+4800-a;
    m=RTC_DateTimeStruct->RTC_Month+(12*a)-3;
 
    JDN=RTC_DateTimeStruct->RTC_Date;
    JDN+=(153*m+2)/5;
    JDN+=365*y;
    JDN+=y/4;
    JDN+=-y/100;
    JDN+=y/400;
    JDN = JDN -32045;
    JDN = JDN - JULIAN_DATE_BASE;
    JDN*=86400;
    JDN+=(RTC_DateTimeStruct->RTC_Hours*3600);
    JDN+=(RTC_DateTimeStruct->RTC_Minutes*60);
    JDN+=(RTC_DateTimeStruct->RTC_Seconds);
 
    return JDN;
}

//-----------------------------Conver data to MyFormat--------------------------
void RTC_GetMyFormat(RTC_DateTimeTypeDef* RTC_DateTimeStruct, char * buffer) 
{
    const char WDAY0[] = "Monday";
    const char WDAY1[] = "Tuesday";
    const char WDAY2[] = "Wednesday";
    const char WDAY3[] = "Thursday";
    const char WDAY4[] = "Friday";
    const char WDAY5[] = "Saturday";
    const char WDAY6[] = "Sunday";
    const char * WDAY[7]={WDAY0, WDAY1, WDAY2, WDAY3, WDAY4, WDAY5, WDAY6};
 
    const char MONTH1[] = "January";
    const char MONTH2[] = "February";
    const char MONTH3[] = "March";
    const char MONTH4[] = "April";
    const char MONTH5[] = "May";
    const char MONTH6[] = "June";
    const char MONTH7[] = "July";
    const char MONTH8[] = "August";
    const char MONTH9[] = "September";
    const char MONTH10[] = "October";
    const char MONTH11[] = "November";
    const char MONTH12[] = "December";
    const char * MONTH[12]={MONTH1, MONTH2, MONTH3, MONTH4, MONTH5, MONTH6, MONTH7, MONTH8, MONTH9, MONTH10, MONTH11, MONTH12};
 
    sprintf(buffer, "%s %d %s %04d",
            WDAY[RTC_DateTimeStruct->RTC_Wday],
            RTC_DateTimeStruct->RTC_Date,
            MONTH[RTC_DateTimeStruct->RTC_Month -1],
            RTC_DateTimeStruct->RTC_Year);
}
/* END OF FILE */
//==============================================================================
