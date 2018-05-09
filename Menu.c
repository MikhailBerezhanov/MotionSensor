/*==============================================================================
>File name:  	MainMenu source file
>Brief:         Here is source code for all functions to display menu in terminal 
                and work with HC-05, BMI160, BMM150 via terminal. 
>Note:          This file uses SPL driver for stm32f103xx and flags of 
                recieved messages from USART interfaceses. As default USART2 is 
                used to comunicate with HC-05. USART2_CONFIG code might be
                changed if another USART module is used (Cmd function)
>Author:        LL
>Date:          07.05.2018
>Version:       1.0
==============================================================================*/

#include "Menu.h"

RTC_DateTimeTypeDef RTC_DateTime;
extern volatile char currTime[9];
extern volatile char currDate[11];
extern int8_t rsltBMI160;
extern int8_t rsltBMM150;

volatile char USART1_RxBuf[RX_BUF_SIZE] = {'\0'};		//buffers for data Recived from PC's Terminal 
volatile char USART2_RxBuf[RX_BUF_SIZE] = {'\0'};
volatile char USART1_TxBuf[TX_BUF_SIZE] = {'\0'};
volatile char USART2_TxBuf[TX_BUF_SIZE] = {'\0'};

volatile char RX1i;		//index of data buffer
volatile char RX2i;

static unsigned short x = 0x0001;
static char buf[80]={'\0'};		//for strings from input stream
static uint32_t num = 0;		//for variables from input stream
static char RTC_buffer[80] = {'\0'};
static bool AT_CONFIG_FLAG = false;
static bool USART2_CONFIG_FLAG = false;

static char *menu[20] = 	//array of pointers to menu strings
{
"\n\r\t1.<bmi160 status> to get accel+gyro sensor status\n\r",
"\t2.<at conf> to enter AT_COMMANDS menu\n\r",
"\t3.<led on>to turn led(PC13) ON\n\r",
"\t4.<led off> to turn led (PC13) OFF\n\r",
"\t5.<bmm150 status> to get magni sensor status\n\r",
"\t6.\"dis hc-05\" to DISABLE power of HC-05\n\r",
"\t7.<help> to see this menu again\n\r",
"\t8.<usart2 conf>to enter USART2 settings\n\r",
"\t9.Exit\n\r",
NULL
};

static char *AT_commands[80]=
{
"\n\r\t1.\"at exit\" to exit AT_CONFIG mode\n\r",
"\t2.\"atsend <***>\" to send <***> ATcommand one of following:\n\r",
"\n\r\t<AT>",		"\t\t<AT+VERSION?>\n\r",	
"\t<AT+RESET>",		"\t<AT+ADDR?>\n\r",		
"\t<AT+ORGL>",		"\t<AT+NAME?>",				"\t<AT+NAME=ИМЯ>\n\r",
"\t<AT+RMAAD>",		"\t<AT+ROLE?>",				"\t<AT+ROLE=РОЛЬ>\n\r",		
"\t<AT+INIT>",		"\t<AT+CLASS?>",			"\t<AT+CLASS=ТИП>\n\r",
"\t<AT+INQ>",		"\t<AT+IAC?>",				"\t<AT+IAC=КОД>\n\r",		
"\t<AT+INQC>",		"\t<AT+INQM?>",				"\t<AT+INQM=РЕЖИМ,КОЛ,ВРЕМЯ>\n\r",
"\t<AT+DISC>",		"\t<AT+PSWD?>",				"\t<AT+PSWD=КОД>\n\r",
					"\t\t\t<AT+UART?>",			"\t<AT+UART=СКОР,СТОП,ПРОВ>\n\r",
					"\t\t\t<AT+CMODE?>",		"\t<AT+CMODE=РЕЖИМ>\n\r",
					"\t\t\t<AT+BIND?>",			"\t<AT+BIND=АДРЕС>\n\r",
					"\t\t\t<AT+POLAR?>",		"\t<AT+POLAR=ЛОГ,ЛОГ>\n\r",
												"\t\t\t\t\t<AT+PIO=НОМЕР,УРОВЕНЬ>\n\r",
					"\t\t\t<AT+MPIO?>",			"\t<AT+MPIO=ЧИСЛО>\n\r",
					"\t\t\t<AT+IPSCAN?>",		"\t<AT+IPSCAN=А,Б,В,Г>\n\r",
					"\t\t\t<AT+SNIFF?>",		"\t<AT+SNIFF=А,Б,В,Г>\n\r",
												"\t\t\t\t\t<AT+ENSNIFF=АДРЕС>\n\r",
												"\t\t\t\t\t<AT+EXSNIFF=АДРЕС>\n\r",
					"\t\t\t<AT+SENM?>",			"\t<AT+SENM=СЕКРЕТ,ШИФР>\n\r",
												"\t\t\t\t\t<AT+PMSAD=АДРЕС>\n\r",
												"\t\t\t\t\t<AT+FSAD=АДРЕС>\n\r",
					"\t\t\t<AT+ADCN?>",			"\t<AT+PAIR=АДРЕС,ТАЙМАУТ>\n\r",
					"\t\t\t<AT+MRAD?>",			"\t<AT+LINK=АДРЕС>\n\r",
					"\t\t\t<AT+STATE?>",		"\t<AT+SENM=СЕКРЕТ,ШИФР>\n\r",	
NULL	
};

static char *usart2_menu[80] =
{
"\n\r\t1.Get started\n\r",
"\t2.\"status\" to see current USART2 setting\n\r",
"\t3.\"baudrate <***>\" to set new <***> baudrate \n\r",
"\tHC-05 supports one of the following value:\n\r",
"\t<4800>", 	"\t\t<38400>",		"\t\t<23400>\n\r",		
"\t<9600>",		"\t\t<57600>",		"\t\t<460800>\n\r",
"\t<19200>",	"\t\t<115200>",		"\t<921600>\n\r",
"\t<1382400>\n\r", 	
"\t4.\"parity <***>\" to set/reset parity\n\r", 
"\t<0> - None, <1> - Odd, <2> - Even\n\r",
"\t5.\"stopbit <***>\" to set STOP_BIT\n\r",
"\t<0> - 1bit, <1> - 2bits\n\r",	
"\t6.\"usart2 exit\" to exit USART2_CONFIG\n\r",	
NULL
};

/**
  * @brief   Sends to terminal starting messages
  * @note   
  * @param  
  * @retval 
  */
void MainMenu_startMess (struct MainMenu_Struct *MainMenu)	
{	
	MainMenu->terminalSend("\n\n\r Hello LL, This is a MotionSensor config APP\n\r");
	
	/* DATE TIME and C VERSION */
	sprintf((char*)USART1_TxBuf, "\r\n Compiling Date: %s",currDate);			
    MainMenu->terminalSend(USART1_TxBuf);			
	sprintf((char*)USART1_TxBuf, "\tTime: %s\r\n",currTime);			
    MainMenu->terminalSend(USART1_TxBuf);
	
		/*test_byteorder of machine (stm32f103)
			sprintf((char*)USART1_TxBuf, "\n\r\tSize of x: %d", sizeof(x));
			USART1_SEND(USART1_TxBuf);
			sprintf((char*)USART1_TxBuf, "\n\r\tByte order: %s", *((unsigned char*) &x) == 0? "Big endian\n\r" : "Little endian\n\r");
			USART1_SEND(USART1_TxBuf);
		//-------------------------------------*/
	
	MainMenu->terminalSend(" You can try smth from Menu below:\n\r");
	Display_Array(MainMenu->terminalSend,menu);	
	MainMenu->terminalSend("\r\n\t:: Current MotionSensor\tstatus ::");
	if (MainMenu_RTC_init()) MainMenu->terminalSend("\r\n\t> RTC init\t\tSUCCESSFULL\r\n");
	else MainMenu->terminalSend("\r\n\t> RTC init\tF A I L E D\r\n");
	if (rsltBMI160 != BMI160_OK) MainMenu->terminalSend("\r\n\t> BMI160 INIT:\t\tF A I L E D\r\n");
	else MainMenu->terminalSend("\t> BMI160 init:\t\tSUCCESSFULL\r\n");
	if (rsltBMM150 == BMM150_OK) MainMenu->terminalSend("\t> BMM150 init:\t\tSUCCESSFULL\r\n");
	else MainMenu->terminalSend("\t> BMM150 init:\t\tF A I L E D\r\n");

	MainMenu->terminalSend("\n\r>Type message: ");
	
}

/**
  * @brief   Watching if answer from HC-05 comes
  * @note    Uses flag that =true when USARTx_RX interrupt comes	
  * @param  
  * @retval 
  */
void MainMenu_checkAnswer (struct MainMenu_Struct *MainMenu)
{		
	//Message from HC-05 module recieved
		if (RX2_FLAG_END_LINE)
		{
			RX2_FLAG_END_LINE = false;
			
			if (USART2_CONFIG_FLAG) 
			{
				MainMenu->terminalSend("\n\r>USART2_CONFIG/");
				Display_Time(MainMenu->terminalSend);
				MainMenu->terminalSend("Answer is: ");
				MainMenu->terminalSend(USART2_RxBuf);
			}
			else if (AT_CONFIG_FLAG) 
			{
				MainMenu->terminalSend("\n\r>AT_CONFIG/");
				Display_Time(MainMenu->terminalSend);
				MainMenu->terminalSend("Answer is: ");
				MainMenu->terminalSend(USART2_RxBuf);		
			}
			else
			{
				Display_Time(MainMenu->terminalSend);
				MainMenu->terminalSend("Answer is: ");
				MainMenu->terminalSend(USART2_RxBuf);	//echo to terminal		
			}
			Clear_Buffer(USART2_RxBuf);
		}
}

/**
  * @brief   Watching for input stream (via terminal) from PC
  * @note    Uses flag that =true when USARTx_RX interrupt comes	
  * @param  
  * @retval 
  */
void MainMenu_checkInput (struct MainMenu_Struct *MainMenu)
{
    /* Message from PC's terminal recieved */
		if (RX1_FLAG_END_LINE)
		{
			RX1_FLAG_END_LINE = false;
            
                /* CONFIG MODE */
				if (USART2_CONFIG_FLAG)
				{
					
					sscanf((char*)USART1_RxBuf,"%s%d",buf,&num);
					
					if (strncmp((char*)USART1_RxBuf,"usart2 exit",10)==0)	
					{
					MainMenu->terminalSend("\n\r>USART2_CONFIG/");
					Display_Time(MainMenu->terminalSend);	
					MainMenu->terminalSend("Command comes: USART2_EXIT\n\r");
					USART2_CONFIG_FLAG = false;
					MainMenu->terminalSend("\n\r>Type message: ");	
					}
					
					else if (strncmp((char*)USART1_RxBuf,"status",6)==0)
					{
					sprintf((char*)USART1_TxBuf,"\n\rUSART2_BaudRate : %d\n\r",MainMenu->usart2Struct->USART_BaudRate);	
					MainMenu->terminalSend(USART1_TxBuf);	
					sprintf((char*)USART1_TxBuf,"USART2_Parity : %d\n\r",MainMenu->usart2Struct->USART_Parity);	
					MainMenu->terminalSend(USART1_TxBuf);
					sprintf((char*)USART1_TxBuf,"USART2_Stopbit : %d\n\r",MainMenu->usart2Struct->USART_StopBits);	
					MainMenu->terminalSend(USART1_TxBuf);		
					Clear_Buffer(USART1_TxBuf);	
					MainMenu->terminalSend("\n\r>USART2_CONFIG/Type message: ");	
					}
					
					else if (strncmp(buf,"baudrate",8)==0)
					{
					Clear_Buffer(buf);	
					USART_Cmd(USART2, DISABLE);	
					MainMenu->usart2Struct->USART_BaudRate = num;
					USART_Init(USART2, MainMenu->usart2Struct);			
					USART_Cmd(USART2, ENABLE);
					MainMenu->terminalSend("\n\r>USART2_CONFIG/");
					Display_Time(MainMenu->terminalSend);
					sprintf((char*)USART1_TxBuf,"new USART2_Baudrate : %d\n\r",num);	
					MainMenu->terminalSend(USART1_TxBuf);
					Clear_Buffer(USART1_TxBuf);	
					MainMenu->terminalSend("\n\r>USART2_CONFIG/Type message: ");	
					}
					
					else if (strncmp(buf,"parity",6)==0)
					{
					Clear_Buffer(buf);	
					USART_Cmd(USART2, DISABLE);	
						switch (num)
						{
						case 0: MainMenu->usart2Struct->USART_Parity = USART_Parity_No; break;
						case 1:	MainMenu->usart2Struct->USART_Parity = USART_Parity_Odd; break;
						case 2: MainMenu->usart2Struct->USART_Parity = USART_Parity_Even; break;
						}
					USART_Init(USART2, MainMenu->usart2Struct);			
					USART_Cmd(USART2, ENABLE);
					MainMenu->terminalSend("\n\r>USART2_CONFIG/");
					Display_Time(MainMenu->terminalSend);
					sprintf((char*)USART1_TxBuf,"new USART2_Parity : %d\n\r",num);	
					MainMenu->terminalSend(USART1_TxBuf);
					Clear_Buffer(USART1_TxBuf);	
					MainMenu->terminalSend("\n\r>USART2_CONFIG/Type message: ");	
					}
					
					else if (strncmp(buf,"stopbit",7)==0)
					{
					Clear_Buffer(buf);	
					USART_Cmd(USART2, DISABLE);	
						switch (num)
						{
						case 0: MainMenu->usart2Struct->USART_StopBits = USART_StopBits_1; break;
						case 1:	MainMenu->usart2Struct->USART_StopBits = USART_StopBits_2; break;
						}
					USART_Init(USART2, MainMenu->usart2Struct);			
					USART_Cmd(USART2, ENABLE);
					MainMenu->terminalSend("\n\r>USART2_CONFIG/");
					Display_Time(MainMenu->terminalSend);
					sprintf((char*)USART1_TxBuf,"new USART2_Stopbit : %d\n\r",num);	
					MainMenu->terminalSend(USART1_TxBuf);
					Clear_Buffer(USART1_TxBuf);	
					MainMenu->terminalSend("\n\r>USART2_CONFIG/Type message: ");		
					}
					
					else 
					{
					MainMenu->terminalSend("\n\r>USART2_CONFIG/");
					Display_Time(MainMenu->terminalSend);
					MainMenu->terminalSend(USART1_RxBuf);
					MainMenu->terminalSend("\n\r>USART2_CONFIG/Type message: ");
					}
					
				}
                
                /* AT COMMANDS MODE */
				else if (AT_CONFIG_FLAG)
				{	
					
					if (strncmp((char*)USART1_RxBuf,"at exit",7)==0)	
					{
					MainMenu->terminalSend("\n\r>AT_CONFIG/");
					Display_Time(MainMenu->terminalSend);	
					MainMenu->terminalSend("Command comes: AT_EXIT\n\r");
					AT_CONFIG_FLAG = false;
					GPIO_SetBits(GPIOC,GPIO_Pin_13);
					MainMenu->terminalSend("\n\r>Type message: ");	
					}
					else 
					{
						sscanf((char*)USART1_RxBuf,"%s%s",buf,USART2_TxBuf);	//scanf input stream
						
						if (strncmp(buf,"atsend", 6) == 0)
						{
							Clear_Buffer(buf);
							//send data from terminal to HC-05
							if(MainMenu->hc05Send(strcat((char*)USART2_TxBuf, "\r\n"))==SUCCESS)
							{
							MainMenu->terminalSend("\n\r>AT_CONFIG/");
							Display_Time(MainMenu->terminalSend);
							MainMenu->terminalSend("You have send to HC-05: ");	
							MainMenu->terminalSend(USART2_TxBuf);
							Clear_Buffer(USART2_TxBuf);	
							}
						}
						else 
						{
						MainMenu->terminalSend("\n\r>AT_CONFIG/");
						Display_Time(MainMenu->terminalSend);
						MainMenu->terminalSend(USART1_RxBuf);
						MainMenu->terminalSend("\n\r>AT_CONFIG/Type message: ");
						}
					}
									
				}
				
                /* NOMAL MODE */
				else
				{
				MainMenu->terminalSend("\n\r>");
				Display_Time(MainMenu->terminalSend);
				MainMenu->terminalSend(USART1_RxBuf);
					
					if (strncmp((char*)USART1_RxBuf,"led off",7)==0)
					{
					MainMenu->terminalSend("\n\r");
					Display_Time(MainMenu->terminalSend);
					MainMenu->terminalSend("Command comes: LED_OFF\n\r");	
					GPIOC->BSRR = GPIO_Pin_13;	
					}
				
					if (strncmp((char*)USART1_RxBuf,"led on",6)==0)
					{
					MainMenu->terminalSend("\n\r");
					Display_Time(MainMenu->terminalSend);
					MainMenu->terminalSend("Command comes: LED_ON\n\r");
					GPIOC->BRR = GPIO_Pin_13;
					}
					
					if (strncmp((char*)USART1_RxBuf,"at conf",7)==0)
					{
					AT_CONFIG_FLAG = true;
					GPIO_ResetBits(GPIOC,GPIO_Pin_13);
					MainMenu->terminalSend("\n\r");
					Display_Time(MainMenu->terminalSend);
					MainMenu->terminalSend("Mode activated: AT_CONFIG\n\r");
					Display_Array(MainMenu->terminalSend,AT_commands);	
					MainMenu->terminalSend("\n\r>AT_CONFIG/Type message: ");
					}
					
					if (strncmp((char*)USART1_RxBuf,"help",4)==0)
					{
					MainMenu->terminalSend("\n\r");
					Display_Time(MainMenu->terminalSend);
					MainMenu->terminalSend("Command comes: HELP\n\r");
					Display_Array(MainMenu->terminalSend,menu);	
					}
					
					if (strncmp((char*)USART1_RxBuf,"usart2 conf",11)==0)
					{
					USART2_CONFIG_FLAG = true;
					MainMenu->terminalSend("\n\r");
					Display_Time(MainMenu->terminalSend);
					MainMenu->terminalSend("Command comes: USART2_CONFIG\n\r");
					Display_Array(MainMenu->terminalSend,usart2_menu);	
					MainMenu->terminalSend("\n\r>USART2_CONFIG/Type message: ");
					}
					
					if (strncmp((char*)USART1_RxBuf,"bmi160 status",13)==0)
					{
					Display_Time(MainMenu->terminalSend);
					MainMenu->terminalSend("Command comes: BMI160_STATUS\n\r");
					BMI160_display_status();
					}		
					
				if (!AT_CONFIG_FLAG && !USART2_CONFIG_FLAG) MainMenu->terminalSend("\n\r>Type message: ");
				}
			Clear_Buffer(USART1_RxBuf);
		}	
}

/**
  * @brief   
  * @note   
  * @param  
  * @retval 
  */
_Bool MainMenu_RTC_init (void)
{
#ifdef RTC_EN
uint8_t chour;		//current time
uint8_t cmin;
uint8_t csec;
	
chour = 10*(currTime[0]-48) + (currTime[1]-48);		// (-48) ASCII -> dec
cmin = 10*(currTime[3]-48) + (currTime[4]-48);	
csec = 10*(currTime[6]-48) + (currTime[7]-48);	

RTC_Init_LSE();	
	//if (RTC_Init_LSE()==1) 
	{
        // Если первая инициализация RTC устанавливаем начальную дату, например 22.09.2016 14:30:00
        RTC_DateTime.RTC_Date = 12;
        RTC_DateTime.RTC_Month = 01;
        RTC_DateTime.RTC_Year = 2018;
 
        RTC_DateTime.RTC_Hours = chour;
        RTC_DateTime.RTC_Minutes = cmin ;
        RTC_DateTime.RTC_Seconds = csec;
 
        RTC_WaitForLastTask();
        RTC_SetCounter(RTC_GetRTC_Counter(&RTC_DateTime));
    }
	
return SUCCESS;	
#endif	
	return 0;	
}

/**
  * @brief      Clears buffer
  * @param      *pbuf - pointer at buffer that must be cleared   
  * @retval     None
  */
void Clear_Buffer(volatile char *pbuf) 
{
	if (pbuf == USART1_RxBuf)	RX1i = 0;	//clear Rx index
	if (pbuf == USART2_RxBuf)	RX2i = 0;	//
	
    if (pbuf != NULL)		//null-pointer check
	{
		while (*pbuf)
		{
		*pbuf = '\0';
		pbuf++;			
		}
	}
	//*pbuf = '\0';
}

/**
  * @brief      Displays array in terminal
  * @note   
  * @param      send - pointer at function send to terminal, 
                *arr - pointer to pointer for string in array	
  * @retval     None
  */
void Display_Array (typedef_pSendfunc send,char *arr[])
{	
	int16_t idx = 0;
	while (arr[idx])	send(arr[idx++]);
}


/**
  * @brief      Displays current time in terminal if RTC initialized
  * @note   
  * @param      send - pointer at function send to terminal
  * @retval     None
  */
void Display_Time (typedef_pSendfunc send)
{
	#ifdef RTC_EN
	RTC_GetTime(&RTC_DateTime);
	sprintf(RTC_buffer, "%02d:%02d:%02d : ",RTC_DateTime.RTC_Hours, RTC_DateTime.RTC_Minutes, RTC_DateTime.RTC_Seconds);
	send(RTC_buffer);	
	#endif
}

