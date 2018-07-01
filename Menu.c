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
char rsltRTC =0;
extern volatile char currTime[9];
extern volatile char currDate[11];
extern struct bmi160_dev bmi160;
extern struct bmm150_dev bmm150;
extern struct Sensors_common_status sensor_status;
struct MainMenu_Struct MainMenu;
extern int8_t rsltBMI160;
extern int8_t rsltBMM150;

volatile char USART1_RxBuf[RX_BUF_SIZE] = {'\0'};		//buffers for data Recived from PC's Terminal 
volatile char USART2_RxBuf[RX_BUF_SIZE] = {'\0'};
volatile char USART1_TxBuf[TX_BUF_SIZE] = {'\0'};
volatile char USART2_TxBuf[TX_BUF_SIZE] = {'\0'};

volatile char RX1i;		//index of data buffer
volatile char RX2i;

//static unsigned short x = 0x0001;
static char buf[80]={'\0'};		//for strings from input stream
static uint32_t num = 0;		//for variables from input stream
#ifdef RTC_EN
static char RTC_buffer[80] = {'\0'};
#endif
static bool AT_CONFIG_FLAG = false;
static bool USART2_CONFIG_FLAG = false;
static bool BMI160_CONFIG_FLAG = false;
static bool BMM150_CONFIG_FLAG = false;
static bool G_ODR_CONFIG_FLAG = false;
static bool G_BW_CONFIG_FLAG = false;
static bool G_RANGE_CONFIG_FLAG = false;
static bool A_ODR_CONFIG_FLAG = false;
static bool A_BW_CONFIG_FLAG = false;
static bool A_RANGE_CONFIG_FLAG = false;
static bool BMM150_DATARATE_CONFIG_FLAG = false;
static bool BMM150_XYREP_CONFIG_FLAG = false;
static bool BMM150_ZREP_CONFIG_FLAG = false;
static bool BMM150_PRESET_CONFIG_FLAG = false;

extern volatile bool MADGWICK_FILTER_FLAG;
extern volatile bool MAHONY_FILTER_FLAG;   

static char *menu[20] = 	//array of pointers to menu strings
{
"\n\r\t\t-=::: Main Menu :::=-\n\r\n\r"
"\t1. < BMI160_conf >\tto set accel+gyro sensor settings\n\r",
"\t2. < BMM150_conf >\tto get magni sensor settings\n\r",
"\t3. < LED_on >\t\tto turn led(PC13) ON\n\r",
"\t4. < LED_off >\t\tto turn led (PC13) OFF\n\r",
"\t5. < BLE_conf >\t\tto enter AT_COMMANDS menu\n\r",
"\t6. < USART2_conf >\tto enter USART2 settings\n\r",
"\t7. < START_RAW >\tto enter raw measurement mode\n\r",
"\t8. < STOP >\t\tto exit any measurement mode\n\r",
"\t9. < START_MADGWICK >\tto enter Madgwick filtered mode\n\r",
"\t10.< START_MAHONY >\tto enter Mahony filtered mode\n\r",    
"\t11.< HELP >\t\tto see this menu again\n\r",    
"\t   < EXIT >\n\r",
NULL
};

static char *bmi160Menu[30] = 	//array of pointers to menu strings
{
"\n\r\t\t-=::: BMI160 CONFIG MENU :::=-\n\r\n\r",
"\t1. < STATUS > \t\tto see current settings of BMI160\n\r",
"\t2. < GYRO_SET_odr > \tto set gyro datarate [Hz]:\n\r",		
"\t3. < GYRO_SET_range > \tto set gyro range [DPS]:\n\r",	
"\t4. < GYRO_SET_bw > \tto set gyro bandwidth:\n\r",			
"\t5. < ACCEL_SET_odr > \tto set accel datarate [Hz]:\n\r",					
"\t6. < ACCEL_SET_range > \tto set accel range [G]:\n\r",	
"\t7. < ACCEL_SET_bw > \tto set accel bandwidth:\n\r",
"\t   < SLEEP_MODE > \tto set accel and gyro SLEEP mode \r\n",	
"\t   < SUSPEND_MODE > \tto set accel and gyro SUSPEND mode \r\n",	    
"\t   < NORMAL_MODE > \tto set accel and gyro NORMAL mode \r\n",
"\t   < RESET > \t\tto soft reset BMI160\n\r",
"\t   < HELP > \t\tto see this menu again\n\r",	
"\t   < EXIT > \t\tto Exit\n\r",
NULL
};

static char *GyroRange[6] =
{
"\n\r\t 125 ",	" 250 ",	" 500 ",	" 1000 ",	" 2000\n\r",
NULL
};

static char *GyroODR[9] =
{
"\n\r\t 25 ",	"\t 50 ",	"\t 100 ",	"\t 200\r\n",	
"\t 400 ",	"\t 800 ",	"\t 1600 ",	"\t 3200\r\n",
NULL
};

static char *GyroBW[4] =
{
"\n\r\t1.OSR4_MODE", 	
"\n\r\t2.OSR2_MODE",
"\n\r\t3.NORMAL_MODE\r\n",
NULL
};

static char *AccelRange[5] =
{
"\n\r\t 2 ",	"\t 4 ", 	"\t 8 ",	"\t 16\n\r",
NULL
};

static char *AccelODR[13] =
{
"\n\r\t 0_78 ",	"\t 1_56 ",	"\t 3_12 ",	"\t 6_25\n\r",	
"\t 12_5 ",	"\t 25 ",	"\t 50 ",	"\t 100\n\r",
"\t 200 ",	"\t 400 ",	"\t 800 ",	"\t 1600\n\r",
NULL
};

static char *AccelBW[9] =
{
"\n\r\t1.OSR4_AVG1\n\r",
"\t2.OSR2_AVG2\n\r",	
"\t3.NORMAL_AVG4\n\r",
"\t4.RES_AVG8\n\r",	
"\t5.RES_AVG16\n\r",	
"\t6.RES_AVG32\n\r",
"\t7.RES_AVG64\n\r",	
"\t8.RES_AVG128\n\r",
NULL
};

static char *bmm150Menu[16]=
{
"\n\r\t\t-=::: BMM150 CONFIG MENU :::=-\n\r",
"\n\r\t1. < STATUS >\t\tto see current setting of BMM150\n\r",
"\t2. < SET_PRESET >\tto set one of available preset\n\r",    
"\t3. < SET_DATARATE >\tto set datarate of BMM150\n\r",
"\t4. < SET_XY-rep >\tto set XY-repetition of BMM150\n\r",
"\t5. < SET_Z-rep>\t\tto set Z-repetition of BMM150\n\r", 
"\t   < ADVANCED_SELFTEST >to perform andvanced selftest\n\r",    
"\t   < SLEEP_MODE >\tto set sleep mode\n\r",
"\t   < SUSPEND_MODE >\tto set suspend mode\n\r",
"\t   < NORMAL_MODE >\tto set normal mode\n\r", 
"\t   < FORCED_MODE >\tto make fast measure and go sleep\n\r",
"\t   < RESET >\t\tto perform soft reset of BMM150\n\r",
"\t   < HELP >\t\tto see this menu again\n\r",
"\t   < EXIT >\t\tto exit\n\r",    
NULL 
};

static char *bmm150PresetsMenu[5] =
{
"\n\r\n\r\t1.LOWPOWER",
"\n\r\t2.REGULAR",
"\n\r\t3.HIGHACCURACY",
"\n\r\t4.ENHANCED\n\r",    
NULL    
};

static char *bmm150DatarateMenu[20]=
{
"\n\r\t2","\t6","\t8", "\t10\n\r",
"\t15","\t20","\t25", "\t30\n\r",    
NULL    
};    


static char *bmm150XYrepMenu[20]=
{
"\n\r\t 1 , 3 , 5 , 7 , ... , 511\n\r",        
NULL    
};  

static char *bmm150ZrepMenu[20]=
{
"\n\r\t 1 , 2 , 3 , 4 , ... , 256\n\r",        
NULL   
};            
        
static char *AT_commands[80]=
{
"\n\r\t\t-=::: BLE CONFIG MENU :::=-\n\r",
"\n\r\t1. < exit >\tto exit AT_CONFIG mode\n\r",
"\t2. < atsend ***>to send <***> ATcommand one of following:\n\r",
"\n\r\t< AT >",		"\t\t< AT+VERSION? >\n\r",	
"\t< AT+RESET >",	"\t< AT+ADDR? >\n\r",		
"\t< AT+ORGL >",	"\t< AT+NAME? >",		    "\t< AT+NAME=ИМЯ >\n\r",
"\t< AT+RMAAD >",	"\t< AT+ROLE? >",		    "\t< AT+ROLE=РОЛЬ >\n\r",		
"\t< AT+INIT >",	"\t< AT+CLASS? >",		    "\t< AT+CLASS=ТИП >\n\r",
"\t< AT+INQ >",		"\t< AT+IAC? >",		    "\t< AT+IAC=КОД >\n\r",		
"\t< AT+INQC >",	"\t< AT+INQM? >",		    "\t< AT+INQM=РЕЖИМ,КОЛ,ВРЕМЯ >\n\r",
"\t< AT+DISC >",	"\t< AT+PSWD? >",		    "\t< AT+PSWD=КОД >\n\r",
					"\t\t\t< AT+UART? >",		"\t< AT+UART=СКОР,СТОП,ПРОВ >\n\r",
					"\t\t\t< AT+CMODE? >",		"\t< AT+CMODE=РЕЖИМ >\n\r",
					"\t\t\t< AT+BIND? >",		"\t< AT+BIND=АДРЕС >\n\r",
					"\t\t\t< AT+POLAR? >",		"\t< AT+POLAR=ЛОГ,ЛОГ >\n\r",
												"\t\t\t\t\t< AT+PIO=НОМЕР,УРОВЕНЬ >\n\r",
					"\t\t\t< AT+MPIO? >",		"\t< AT+MPIO=ЧИСЛО >\n\r",
					"\t\t\t< AT+IPSCAN? >",		"\t< AT+IPSCAN=А,Б,В,Г >\n\r",
					"\t\t\t< AT+SNIFF? >",		"\t< AT+SNIFF=А,Б,В,Г >\n\r",
												"\t\t\t\t\t< AT+ENSNIFF=АДРЕС >\n\r",
												"\t\t\t\t\t< AT+EXSNIFF=АДРЕС >\n\r",
					"\t\t\t< AT+SENM? >",		"\t< AT+SENM=СЕКРЕТ,ШИФР >\n\r",
												"\t\t\t\t\t< AT+PMSAD=АДРЕС >\n\r",
												"\t\t\t\t\t< AT+FSAD=АДРЕС >\n\r",
					"\t\t\t< AT+ADCN? >",		"\t< AT+PAIR=АДРЕС,ТАЙМАУТ >\n\r",
					"\t\t\t< AT+MRAD? >",		"\t< AT+LINK=АДРЕС >\n\r",
					"\t\t\t< AT+STATE? >",		"\t< AT+SENM=СЕКРЕТ,ШИФР >\n\r",	
NULL	
};

static char *usart2_menu[80] =
{
"\n\r\t\t-=::: USART2 CONFIG MENU :::=-\n\r",
"\n\r\t1.< STATUS >\t\tto see current USART2 setting\n\r",
"\t2.< BAUDRATE ***>\tto set new <***> baudrate \n\r",
"\tHC-05 supports one of the following value:\n\r",
"\t\t 4800 ", 	    "\t\t 38400 ",		"\t\t 23400 \n\r",		
"\t\t 9600 ",		"\t\t 57600 ",		"\t\t 460800 \n\r",
"\t\t 19200 ",	    "\t\t 115200 ",		"\t 921600 \n\r",
"\t\t 1382400 \n\r", 	
"\t3.< PARITY ***>\t\tto set/reset parity\n\r", 
"\t\t 0  - None,  1  - Odd,  2  - Even\n\r",
"\t5.< STOPBIT ***>\tto set STOP_BIT\n\r",
"\t\t 0  - 1bit,  1  - 2bits\n\r",	
"\t  < EXIT >\t\tto exit USART2_CONFIG\n\r",	
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
	
	MainMenu_RTC_init();
	if (rsltRTC == 1) MainMenu->terminalSend("\r\n\t> RTC init:\t\tSUCCESSFULL\r\n");
	else if (rsltRTC == 2) MainMenu->terminalSend("\r\n\t> RTC init:\t\tERROR:TIMEOUT\r\n");
	else if (rsltRTC == 0)MainMenu->terminalSend("\r\n\t> RTC init:\t\tALREADY INITED\r\n");
	else MainMenu->terminalSend("\r\n\t> RTC init:\t\tUNKNOWN ERROR\r\n");
	if (rsltBMI160 != BMI160_OK) MainMenu->terminalSend("\t> BMI160 init:\t\tF A I L E D\r\n");
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
			
			if (AT_CONFIG_FLAG) 
			{
				MainMenu->terminalSend("\n\r>AT_CONFIG/");
				Display_Time(MainMenu->terminalSend);
				MainMenu->terminalSend("Answer is: ");
				MainMenu->terminalSend(USART2_RxBuf);
                MainMenu->terminalSend("\n\r>AT_CONFIG/Type message: ");
			}
			else
			{
				Display_Time(MainMenu->terminalSend);
				MainMenu->terminalSend("Message comes: ");
				MainMenu->terminalSend(USART2_RxBuf);	//echo to terminal
                MainMenu->terminalSend("\n\rType message: ");
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
			
				/* BMI160 CONFIG MODE */
				if (BMI160_CONFIG_FLAG) bmi160_menu(MainMenu);
				/* Gyro datarate config */
				else if (G_ODR_CONFIG_FLAG) GyroDatarate_menu(MainMenu);
				/* G_BW CONFIG MODE */
				else if (G_BW_CONFIG_FLAG) GyroBW_menu(MainMenu);
				/* G_RANGE CONFIG MODE */
				else if (G_RANGE_CONFIG_FLAG) GyroRange_menu(MainMenu);	
				/* A_DATARATE CONFIG MODE */
				else if (A_ODR_CONFIG_FLAG) AccelDatarate_menu(MainMenu);
                /* A_RANGE CONFIG MODE */
				else if (A_RANGE_CONFIG_FLAG) AccelRange_menu(MainMenu);
                /* A_BW CONFIG MODE */
				else if (A_BW_CONFIG_FLAG) AccelBW_menu(MainMenu);
                /* BMM150 CONFIG MODE */
                else if (BMM150_CONFIG_FLAG) bmm150_menu(MainMenu);
                /* BMM150 Preset CONFIG */
                else if (BMM150_PRESET_CONFIG_FLAG) bmm150Preset_menu(MainMenu);
                /* BMM150 DATARATE CONFIG */
                else if (BMM150_DATARATE_CONFIG_FLAG) bmm150Datarate_menu(MainMenu);
                /* BMM150 XYREP CONFIG */
                else if (BMM150_XYREP_CONFIG_FLAG) bmm150XYrep_menu(MainMenu);
                /* BMM150 ZREP CONFIG */
                else if (BMM150_ZREP_CONFIG_FLAG) bmm150Zrep_menu(MainMenu);
				/* USART2 CONFIG MODE */
				else if (USART2_CONFIG_FLAG) USART2_menu(MainMenu);                
                /* AT COMMANDS MODE */
				else if (AT_CONFIG_FLAG) BLE_menu(MainMenu);
                /* NORMAL MODE */
				else Main_menu(MainMenu);
			
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

rsltRTC = RTC_Init_LSE();	

        RTC_DateTime.RTC_Date = 12;
        RTC_DateTime.RTC_Month = 01;
        RTC_DateTime.RTC_Year = 2018;
 
        RTC_DateTime.RTC_Hours = chour;
        RTC_DateTime.RTC_Minutes = cmin ;
        RTC_DateTime.RTC_Seconds = csec;
 
        RTC_WaitForLastTask();
        RTC_SetCounter(RTC_GetRTC_Counter(&RTC_DateTime));
		
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
static void Display_Array (typedef_pSendfunc send,char *arr[])
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
static void Display_Time (typedef_pSendfunc send)
{
	#ifdef RTC_EN
	RTC_GetTime(&RTC_DateTime);
	sprintf(RTC_buffer, "%02d:%02d:%02d : ",RTC_DateTime.RTC_Hours, RTC_DateTime.RTC_Minutes, RTC_DateTime.RTC_Seconds);
	send(RTC_buffer);	
	#endif
}

/**
  * @brief      Upper string
  * @note   
  * @param      *str - pointer at string 
  * @retval     pointer at the start of modified string
  */
void str_toupper (char *str)
{
     while(*str)
     {
        *str = (char)toupper(*str);
        str++;
     }
}



/**
  * @brief      Displays BMI160 menu
  * @note   
  * @param      *MainMenu - pointer at structure Menu
  * @retval     None
  */
static inline void bmi160_menu (struct MainMenu_Struct *MainMenu)
{
    switch (atoi((char*)USART1_RxBuf))
    {
        /*status*/
        case 1: Display_Time(MainMenu->terminalSend);
        MainMenu->terminalSend("Command comes: BMI160_STATUS\n\r");
        BMI160_display_status();
        MainMenu->terminalSend("\n\r\n\r>BMI160_CONFIG/Type message: ");
        break;                   
        /*gyro set datarate*/
        case 2: G_ODR_CONFIG_FLAG = true;
        BMI160_CONFIG_FLAG = false;
        Display_Time(MainMenu->terminalSend);
        MainMenu->terminalSend("Command comes: GYRO_set_DATARATE");
        MainMenu->terminalSend("\n\r\n\r\t:: Available G_Datarates [Hz]::");       
        Display_Array(MainMenu->terminalSend,GyroODR);
        MainMenu->terminalSend("\n\r>BMI160_CONFIG/Enter value: ");
        break;
        /*gyro set range*/
        case 3: G_RANGE_CONFIG_FLAG = true;
        BMI160_CONFIG_FLAG = false;
        Display_Time(MainMenu->terminalSend);
        MainMenu->terminalSend("Command comes: GYRO_set_RANGE");
        MainMenu->terminalSend("\n\r\n\r\t:: Available G_RANGE [DPS]::");       
        Display_Array(MainMenu->terminalSend,GyroRange);
        MainMenu->terminalSend("\n\r>BMI160_CONFIG/Enter value: ");	
        break;
        /*gyro set bandwidth*/
        case 4: G_BW_CONFIG_FLAG = true;
        BMI160_CONFIG_FLAG = false;
        Display_Time(MainMenu->terminalSend);
        MainMenu->terminalSend("Command comes: GYRO_set_BANDWIDTH");
        MainMenu->terminalSend("\n\r\n\r\t:: Available G_BANDWIDTH ::");       
        Display_Array(MainMenu->terminalSend,GyroBW);
        MainMenu->terminalSend("\n\r>BMI160_CONFIG/Enter value: ");
        break;
        /*accel set datarate*/
        case 5: A_ODR_CONFIG_FLAG = true;
        BMI160_CONFIG_FLAG = false;
        Display_Time(MainMenu->terminalSend);
        MainMenu->terminalSend("Command comes: ACCEL_set_DATARATE");
        MainMenu->terminalSend("\n\r\n\r\t:: Available A_DATARATE [Hz]::");       
        Display_Array(MainMenu->terminalSend,AccelODR);
        MainMenu->terminalSend("\n\r>BMI160_CONFIG/Enter value: ");
        break;
        /*accel set range*/
        case 6: A_RANGE_CONFIG_FLAG = true;
        BMI160_CONFIG_FLAG = false;
        Display_Time(MainMenu->terminalSend);
        MainMenu->terminalSend("Command comes: ACCEL_set_RANGE");
        MainMenu->terminalSend("\n\r\n\r\t:: Available A_RANGE [G]::");       
        Display_Array(MainMenu->terminalSend,AccelRange);
        MainMenu->terminalSend("\n\r>BMI160_CONFIG/Enter value: ");
        break;
        /*accel set bandwidth*/
        case 7: A_BW_CONFIG_FLAG = true;
        BMI160_CONFIG_FLAG = false;
        Display_Time(MainMenu->terminalSend);
        MainMenu->terminalSend("Command comes: ACCEL_set_BANDWIDTH");
        MainMenu->terminalSend("\n\r\n\r\t:: Available A_BANDWIDTH ::");       
        Display_Array(MainMenu->terminalSend,AccelBW);
        MainMenu->terminalSend("\n\r>BMI160_CONFIG/Enter value: ");
        break;

        default: 
        if (strncmp((char*)USART1_RxBuf,"NORMAL_MODE",11)==0)
        {
        Display_Time(MainMenu->terminalSend);
        MainMenu->terminalSend("Command comes: BMI160_NORMAL");
        if (BMI160_set_Mode(&bmi160,BMI160_ACCEL_NORMAL_MODE,BMI160_GYRO_NORMAL_MODE)==BMI160_OK)
        MainMenu->terminalSend("\t::SUCCESS::\n\r");
        else MainMenu->terminalSend("\t::FAILED::\n\r");
        Display_Array(MainMenu->terminalSend,bmi160Menu);         
        MainMenu->terminalSend("\n\r>BMI160_CONFIG/Type message: ");
        }
        
        else if (strncmp((char*)USART1_RxBuf,"SUSPEND_MODE",12)==0)
        {
        Display_Time(MainMenu->terminalSend);
        MainMenu->terminalSend("Command comes: BMI160_SUSPEND");
        if (BMI160_set_Mode(&bmi160,BMI160_ACCEL_SUSPEND_MODE,BMI160_GYRO_SUSPEND_MODE)==BMI160_OK)
        MainMenu->terminalSend("\t::SUCCESS::\n\r");
        else MainMenu->terminalSend("\t::FAILED::\n\r");
        Display_Array(MainMenu->terminalSend,bmi160Menu);        
        MainMenu->terminalSend("\n\r>BMI160_CONFIG/Type message: ");
        }
        
        else if (strncmp((char*)USART1_RxBuf,"SLEEP_MODE",10)==0)
        {
        Display_Time(MainMenu->terminalSend);
        MainMenu->terminalSend("Command comes: BMI160_SLEEP");
        if (BMI160_set_Mode(&bmi160,BMI160_ACCEL_LOWPOWER_MODE,BMI160_GYRO_FASTSTARTUP_MODE)==BMI160_OK)
        MainMenu->terminalSend("\t::SUCCESS::\n\r");
        else MainMenu->terminalSend("\t::FAILED::\n\r");
        Display_Array(MainMenu->terminalSend,bmi160Menu);        
        MainMenu->terminalSend("\n\r>BMI160_CONFIG/Type message: ");
        }
        
        else if ((strncmp((char*)USART1_RxBuf,"exit",4)==0) ||
                (strncmp((char*)USART1_RxBuf,"EXIT",4)==0))
        {
        MainMenu->terminalSend("\n\r>BMI160_CONFIG/");
        Display_Time(MainMenu->terminalSend);	
        MainMenu->terminalSend("Command comes: BMI160_EXIT\n\r");
        BMI160_CONFIG_FLAG = false;
        Display_Array(MainMenu->terminalSend,menu);
        MainMenu->terminalSend("\n\r>Type message: ");
        }
        
        else if (strncmp((char*)USART1_RxBuf,"RESET",5)==0)
        {
        MainMenu->terminalSend("Command comes: BMI160_RESET\n\r");
            if (bmi160_soft_reset(&bmi160) == BMI160_OK) 
            MainMenu->terminalSend("\tSoft reset :: SUCCESS ::\n\r");
            else
            MainMenu->terminalSend("\tSoft reset :: FAILED ::\n\r");
        Display_Array(MainMenu->terminalSend,bmi160Menu);            
        MainMenu->terminalSend("\n\r>BMI160_CONFIG/Type message: ");       
        }
        
        else if ((strncmp((char*)USART1_RxBuf,"help",4)==0) ||
                (strncmp((char*)USART1_RxBuf,"HELP",4)==0))
        {
        MainMenu->terminalSend("\n\r");
        Display_Array(MainMenu->terminalSend,bmi160Menu);
        MainMenu->terminalSend("\n\r>BMI160_CONFIG/Type message: ");      
        }
        
        else if (strncmp((char*)USART1_RxBuf,"STATUS",5)==0)
        {
        Display_Time(MainMenu->terminalSend);
        MainMenu->terminalSend("Command comes: BMI160_STATUS\n\r");
        BMI160_display_status();
        MainMenu->terminalSend("\n\r\n\r>BMI160_CONFIG/Type message: ");
        }
        
        else if (strncmp((char*)USART1_RxBuf,"GYRO_SET_odr",12)==0)
        {
        G_ODR_CONFIG_FLAG = true;
        BMI160_CONFIG_FLAG = false;
        Display_Time(MainMenu->terminalSend);
        MainMenu->terminalSend("Command comes: GYRO_set_DATARATE");
        MainMenu->terminalSend("\n\r\n\r\t:: Available G_Datarates [Hz]::");       
        Display_Array(MainMenu->terminalSend,GyroODR);
        MainMenu->terminalSend("\n\r>BMI160_CONFIG/Enter value: ");
        }
        
        else if (strncmp((char*)USART1_RxBuf,"GYRO_SET_range",14)==0)
        {
        G_RANGE_CONFIG_FLAG = true;
        BMI160_CONFIG_FLAG = false;
        Display_Time(MainMenu->terminalSend);
        MainMenu->terminalSend("Command comes: GYRO_set_RANGE");
        MainMenu->terminalSend("\n\r\n\r\t:: Available G_RANGE [DPS]::");       
        Display_Array(MainMenu->terminalSend,GyroRange);
        MainMenu->terminalSend("\n\r>BMI160_CONFIG/Enter value: ");
        }
        
        else if (strncmp((char*)USART1_RxBuf,"GYRO_SET_bw",11)==0)
        {
        G_BW_CONFIG_FLAG = true;
        BMI160_CONFIG_FLAG = false;
        Display_Time(MainMenu->terminalSend);
        MainMenu->terminalSend("Command comes: GYRO_set_BANDWIDTH");
        MainMenu->terminalSend("\n\r\n\r\t:: Available G_BANDWIDTH ::");       
        Display_Array(MainMenu->terminalSend,GyroBW);
        MainMenu->terminalSend("\n\r>BMI160_CONFIG/Enter value: ");
        }
        
        else if (strncmp((char*)USART1_RxBuf,"ACCEL_SET_odr",13)==0)
        {
        A_ODR_CONFIG_FLAG = true;
        BMI160_CONFIG_FLAG = false;
        Display_Time(MainMenu->terminalSend);
        MainMenu->terminalSend("Command comes: ACCEL_set_DATARATE");
        MainMenu->terminalSend("\n\r\n\r\t:: Available A_DATARATE [Hz]::");       
        Display_Array(MainMenu->terminalSend,AccelODR);
        MainMenu->terminalSend("\n\r>BMI160_CONFIG/Enter value: ");   
        }
        
        else if (strncmp((char*)USART1_RxBuf,"ACCEL_SET_range",15)==0)
        {
        A_RANGE_CONFIG_FLAG = true;
        BMI160_CONFIG_FLAG = false;
        Display_Time(MainMenu->terminalSend);
        MainMenu->terminalSend("Command comes: ACCEL_set_RANGE");
        MainMenu->terminalSend("\n\r\n\r\t:: Available A_RANGE [G]::");       
        Display_Array(MainMenu->terminalSend,AccelRange);
        MainMenu->terminalSend("\n\r>BMI160_CONFIG/Enter value: ");
        }
        
        else if (strncmp((char*)USART1_RxBuf,"ACCEL_SET_bw",11)==0)
        {
        A_BW_CONFIG_FLAG = true;
        BMI160_CONFIG_FLAG = false;
        Display_Time(MainMenu->terminalSend);
        MainMenu->terminalSend("Command comes: ACCEL_set_BANDWIDTH");
        MainMenu->terminalSend("\n\r\n\r\t:: Available A_BANDWIDTH ::");       
        Display_Array(MainMenu->terminalSend,AccelBW);
        MainMenu->terminalSend("\n\r>BMI160_CONFIG/Enter value: ");
        }
        
        else
        {
        MainMenu->terminalSend("\n\r>BMI160_CONFIG/");
        Display_Time(MainMenu->terminalSend);
        MainMenu->terminalSend("::Unknown command::");    
        MainMenu->terminalSend("\n\r>BMI160_CONFIG/Type message: ");
        }
        break;      
   }    
}
 


static inline void GyroDatarate_menu (struct MainMenu_Struct *MainMenu)
{
G_ODR_CONFIG_FLAG = false;
					
    sensor_status.accel_pwr_mode = bmi160.accel_cfg.power;
    sensor_status.gyro_pwr_mode = bmi160.gyro_cfg.power;
    BMI160_set_Mode(&bmi160,BMI160_ACCEL_NORMAL_MODE,BMI160_GYRO_NORMAL_MODE);

    switch (atoi((char*)USART1_RxBuf))
    {
    case 25:bmi160.gyro_cfg.odr = BMI160_GYRO_ODR_25HZ;
            if (bmi160_set_sens_conf(&bmi160) == BMI160_OK)
            MainMenu->terminalSend("\n\r>BMI160_CONFIG/::SUCCESS::");
            else MainMenu->terminalSend("\n\r>BMI160_CONFIG/::FAILED::");
            break;

    case 50:bmi160.gyro_cfg.odr = BMI160_GYRO_ODR_50HZ;
            if (bmi160_set_sens_conf(&bmi160) == BMI160_OK)
            MainMenu->terminalSend("\n\r>BMI160_CONFIG/::SUCCESS::");
            else MainMenu->terminalSend("\n\r>BMI160_CONFIG/::FAILED::");
            break;

    case 100:bmi160.gyro_cfg.odr = BMI160_GYRO_ODR_100HZ;
            if (bmi160_set_sens_conf(&bmi160) == BMI160_OK)
            MainMenu->terminalSend("\n\r>BMI160_CONFIG/::SUCCESS::");
            else MainMenu->terminalSend("\n\r>BMI160_CONFIG/::FAILED::");
            break;

    case 200:bmi160.gyro_cfg.odr = BMI160_GYRO_ODR_200HZ;
            if (bmi160_set_sens_conf(&bmi160) == BMI160_OK)
            MainMenu->terminalSend("\n\r>BMI160_CONFIG/::SUCCESS::");
            else MainMenu->terminalSend("\n\r>BMI160_CONFIG/::FAILED::");
            break;

    case 400:bmi160.gyro_cfg.odr = BMI160_GYRO_ODR_400HZ;
            if (bmi160_set_sens_conf(&bmi160) == BMI160_OK)
            MainMenu->terminalSend("\n\r>BMI160_CONFIG/::SUCCESS::");
            else MainMenu->terminalSend("\n\r>BMI160_CONFIG/::FAILED::");
            break;

    case 800:bmi160.gyro_cfg.odr = BMI160_GYRO_ODR_800HZ;
            if (bmi160_set_sens_conf(&bmi160) == BMI160_OK)
            MainMenu->terminalSend("\n\r>BMI160_CONFIG/::SUCCESS::");
            else MainMenu->terminalSend("\n\r>BMI160_CONFIG/::FAILED::");
            break;

    case 1600:bmi160.gyro_cfg.odr = BMI160_GYRO_ODR_1600HZ;
            if (bmi160_set_sens_conf(&bmi160) == BMI160_OK)
            MainMenu->terminalSend("\n\r>BMI160_CONFIG/::SUCCESS::");
            else MainMenu->terminalSend("\n\r>BMI160_CONFIG/::FAILED::");
            break;

    case 3200:bmi160.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
            if (bmi160_set_sens_conf(&bmi160) == BMI160_OK)
            MainMenu->terminalSend("\n\r>BMI160_CONFIG/::SUCCESS::");
            else MainMenu->terminalSend("\n\r>BMI160_CONFIG/::FAILED::");
            break;

    default: MainMenu->terminalSend("\n\r>BMI160_CONFIG/::Incorrect value!");
            break;
    }
    BMI160_set_Mode(&bmi160,sensor_status.accel_pwr_mode,sensor_status.gyro_pwr_mode);
    BMI160_CONFIG_FLAG = true;
    MainMenu->terminalSend("\n\r");
    Display_Array(MainMenu->terminalSend,bmi160Menu);
    MainMenu->terminalSend("\n\r>BMI160_CONFIG/Type message: ");    
}

static inline void GyroBW_menu (struct MainMenu_Struct *MainMenu)
{
G_BW_CONFIG_FLAG = false;
                    
    sensor_status.accel_pwr_mode = bmi160.accel_cfg.power;
    sensor_status.gyro_pwr_mode = bmi160.gyro_cfg.power;
    BMI160_set_Mode(&bmi160,BMI160_ACCEL_NORMAL_MODE,BMI160_GYRO_NORMAL_MODE);

    switch (atoi((char*)USART1_RxBuf))
    {
    case 1:bmi160.gyro_cfg.bw = BMI160_GYRO_BW_OSR4_MODE;
            if (bmi160_set_sens_conf(&bmi160) == BMI160_OK)
            MainMenu->terminalSend("\n\r>BMI160_CONFIG/::SUCCESS::");
            else MainMenu->terminalSend("\n\r>BMI160_CONFIG/::FAILED::");
            break;
    case 2:bmi160.gyro_cfg.bw = BMI160_GYRO_BW_OSR2_MODE;
            if (bmi160_set_sens_conf(&bmi160) == BMI160_OK)
            MainMenu->terminalSend("\n\r>BMI160_CONFIG/::SUCCESS::");
            else MainMenu->terminalSend("\n\r>BMI160_CONFIG/::FAILED::");
            break;        
    case 3:bmi160.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
            if (bmi160_set_sens_conf(&bmi160) == BMI160_OK)
            MainMenu->terminalSend("\n\r>BMI160_CONFIG/::SUCCESS::");
            else MainMenu->terminalSend("\n\r>BMI160_CONFIG/::FAILED::");
            break;        
    default: MainMenu->terminalSend("\n\r>BMI160_CONFIG/::Incorrect value!");
            break;
    } 
    BMI160_set_Mode(&bmi160,sensor_status.accel_pwr_mode,sensor_status.gyro_pwr_mode);
    BMI160_CONFIG_FLAG = true;
    MainMenu->terminalSend("\n\r");
    Display_Array(MainMenu->terminalSend,bmi160Menu);
    MainMenu->terminalSend("\n\r>BMI160_CONFIG/Type message: ");       
}

static inline void GyroRange_menu (struct MainMenu_Struct *MainMenu)
{
 G_RANGE_CONFIG_FLAG = false;
                    
    sensor_status.accel_pwr_mode = bmi160.accel_cfg.power;
    sensor_status.gyro_pwr_mode = bmi160.gyro_cfg.power;
    BMI160_set_Mode(&bmi160,BMI160_ACCEL_NORMAL_MODE,BMI160_GYRO_NORMAL_MODE);

    switch (atoi((char*)USART1_RxBuf))
    {
    case 125:bmi160.gyro_cfg.range = BMI160_GYRO_RANGE_125_DPS;
            if (bmi160_set_sens_conf(&bmi160) == BMI160_OK)
            MainMenu->terminalSend("\n\r>BMI160_CONFIG/::SUCCESS::");
            else MainMenu->terminalSend("\n\r>BMI160_CONFIG/::FAILED::");
            break;
    case 250:bmi160.gyro_cfg.range = BMI160_GYRO_RANGE_250_DPS;
            if (bmi160_set_sens_conf(&bmi160) == BMI160_OK)
            MainMenu->terminalSend("\n\r>BMI160_CONFIG/::SUCCESS::");
            else MainMenu->terminalSend("\n\r>BMI160_CONFIG/::FAILED::");
            break;        
    case 500:bmi160.gyro_cfg.range = BMI160_GYRO_RANGE_500_DPS;
            if (bmi160_set_sens_conf(&bmi160) == BMI160_OK)
            MainMenu->terminalSend("\n\r>BMI160_CONFIG/::SUCCESS::");
            else MainMenu->terminalSend("\n\r>BMI160_CONFIG/::FAILED::");
            break;
    case 1000:bmi160.gyro_cfg.range = BMI160_GYRO_RANGE_1000_DPS;
            if (bmi160_set_sens_conf(&bmi160) == BMI160_OK)
            MainMenu->terminalSend("\n\r>BMI160_CONFIG/::SUCCESS::");
            else MainMenu->terminalSend("\n\r>BMI160_CONFIG/::FAILED::");
            break;
   case 2000:bmi160.gyro_cfg.bw = BMI160_GYRO_RANGE_2000_DPS;
            if (bmi160_set_sens_conf(&bmi160) == BMI160_OK)
            MainMenu->terminalSend("\n\r>BMI160_CONFIG/::SUCCESS::");
            else MainMenu->terminalSend("\n\r>BMI160_CONFIG/::FAILED::");
            break; 
    default: MainMenu->terminalSend("\n\r>BMI160_CONFIG/::Incorrect value!");
            break;
    } 
    BMI160_set_Mode(&bmi160,sensor_status.accel_pwr_mode,sensor_status.gyro_pwr_mode);
    BMI160_CONFIG_FLAG = true;
    MainMenu->terminalSend("\n\r");
    Display_Array(MainMenu->terminalSend,bmi160Menu);
    MainMenu->terminalSend("\n\r>BMI160_CONFIG/Type message: ");      
}

static inline void AccelDatarate_menu (struct MainMenu_Struct *MainMenu)
{
A_ODR_CONFIG_FLAG = false;
                    
    sensor_status.accel_pwr_mode = bmi160.accel_cfg.power;
    sensor_status.gyro_pwr_mode = bmi160.gyro_cfg.power;
    BMI160_set_Mode(&bmi160,BMI160_ACCEL_NORMAL_MODE,BMI160_GYRO_NORMAL_MODE);

    switch (atoi((char*)USART1_RxBuf))
    {
    case 25:bmi160.accel_cfg.odr = BMI160_ACCEL_ODR_25HZ;
            if (bmi160_set_sens_conf(&bmi160) == BMI160_OK)
            MainMenu->terminalSend("\n\r>BMI160_CONFIG/::SUCCESS::");
            else MainMenu->terminalSend("\n\r>BMI160_CONFIG/::FAILED::");
            break;
    case 50:bmi160.accel_cfg.odr = BMI160_ACCEL_ODR_50HZ;
            if (bmi160_set_sens_conf(&bmi160) == BMI160_OK)
            MainMenu->terminalSend("\n\r>BMI160_CONFIG/::SUCCESS::");
            else MainMenu->terminalSend("\n\r>BMI160_CONFIG/::FAILED::");
            break;        
    case 100:bmi160.accel_cfg.odr = BMI160_ACCEL_ODR_100HZ;
            if (bmi160_set_sens_conf(&bmi160) == BMI160_OK)
            MainMenu->terminalSend("\n\r>BMI160_CONFIG/::SUCCESS::");
            else MainMenu->terminalSend("\n\r>BMI160_CONFIG/::FAILED::");
            break;
    case 200:bmi160.accel_cfg.odr = BMI160_ACCEL_ODR_200HZ;
            if (bmi160_set_sens_conf(&bmi160) == BMI160_OK)
            MainMenu->terminalSend("\n\r>BMI160_CONFIG/::SUCCESS::");
            else MainMenu->terminalSend("\n\r>BMI160_CONFIG/::FAILED::");
            break;
    case 400:bmi160.accel_cfg.odr = BMI160_ACCEL_ODR_400HZ;
            if (bmi160_set_sens_conf(&bmi160) == BMI160_OK)
            MainMenu->terminalSend("\n\r>BMI160_CONFIG/::SUCCESS::");
            else MainMenu->terminalSend("\n\r>BMI160_CONFIG/::FAILED::");
            break;
    case 800:bmi160.accel_cfg.odr = BMI160_ACCEL_ODR_800HZ;
            if (bmi160_set_sens_conf(&bmi160) == BMI160_OK)
            MainMenu->terminalSend("\n\r>BMI160_CONFIG/::SUCCESS::");
            else MainMenu->terminalSend("\n\r>BMI160_CONFIG/::FAILED::");
            break; 
    case 1600:bmi160.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
            if (bmi160_set_sens_conf(&bmi160) == BMI160_OK)
            MainMenu->terminalSend("\n\r>BMI160_CONFIG/::SUCCESS::");
            else MainMenu->terminalSend("\n\r>BMI160_CONFIG/::FAILED::");
            break;       
    default: MainMenu->terminalSend("\n\r>BMI160_CONFIG/::Incorrect value!");
            break;
    } 
    BMI160_set_Mode(&bmi160,sensor_status.accel_pwr_mode,sensor_status.gyro_pwr_mode);
    BMI160_CONFIG_FLAG = true;
    MainMenu->terminalSend("\n\r");
    Display_Array(MainMenu->terminalSend,bmi160Menu);
    MainMenu->terminalSend("\n\r>BMI160_CONFIG/Type message: ");       
}

static inline void AccelBW_menu (struct MainMenu_Struct *MainMenu)
{
A_BW_CONFIG_FLAG = false;
                    
    sensor_status.accel_pwr_mode = bmi160.accel_cfg.power;
    sensor_status.gyro_pwr_mode = bmi160.gyro_cfg.power;
    BMI160_set_Mode(&bmi160,BMI160_ACCEL_NORMAL_MODE,BMI160_GYRO_NORMAL_MODE);

    switch (atoi((char*)USART1_RxBuf))
    {
    case 1:bmi160.accel_cfg.bw = BMI160_ACCEL_BW_OSR4_AVG1;
            if (bmi160_set_sens_conf(&bmi160) == BMI160_OK)
            MainMenu->terminalSend("\n\r>BMI160_CONFIG/::SUCCESS::");
            else MainMenu->terminalSend("\n\r>BMI160_CONFIG/::FAILED::");
            break;
    case 2:bmi160.accel_cfg.bw = BMI160_ACCEL_BW_OSR2_AVG2;
            if (bmi160_set_sens_conf(&bmi160) == BMI160_OK)
            MainMenu->terminalSend("\n\r>BMI160_CONFIG/::SUCCESS::");
            else MainMenu->terminalSend("\n\r>BMI160_CONFIG/::FAILED::");
            break;        
    case 3:bmi160.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
            if (bmi160_set_sens_conf(&bmi160) == BMI160_OK)
            MainMenu->terminalSend("\n\r>BMI160_CONFIG/::SUCCESS::");
            else MainMenu->terminalSend("\n\r>BMI160_CONFIG/::FAILED::");
            break;
    case 4:bmi160.accel_cfg.bw = BMI160_ACCEL_BW_RES_AVG8;
            if (bmi160_set_sens_conf(&bmi160) == BMI160_OK)
            MainMenu->terminalSend("\n\r>BMI160_CONFIG/::SUCCESS::");
            else MainMenu->terminalSend("\n\r>BMI160_CONFIG/::FAILED::");
            break;
    case 5:bmi160.accel_cfg.bw = BMI160_ACCEL_BW_RES_AVG16;
            if (bmi160_set_sens_conf(&bmi160) == BMI160_OK)
            MainMenu->terminalSend("\n\r>BMI160_CONFIG/::SUCCESS::");
            else MainMenu->terminalSend("\n\r>BMI160_CONFIG/::FAILED::");
            break;
    case 6:bmi160.accel_cfg.bw = BMI160_ACCEL_BW_RES_AVG32;
            if (bmi160_set_sens_conf(&bmi160) == BMI160_OK)
            MainMenu->terminalSend("\n\r>BMI160_CONFIG/::SUCCESS::");
            else MainMenu->terminalSend("\n\r>BMI160_CONFIG/::FAILED::");
            break;
    case 7:bmi160.accel_cfg.bw = BMI160_ACCEL_BW_RES_AVG64;
            if (bmi160_set_sens_conf(&bmi160) == BMI160_OK)
            MainMenu->terminalSend("\n\r>BMI160_CONFIG/::SUCCESS::");
            else MainMenu->terminalSend("\n\r>BMI160_CONFIG/::FAILED::");
            break;
    case 8:bmi160.accel_cfg.bw = BMI160_ACCEL_BW_RES_AVG128;
            if (bmi160_set_sens_conf(&bmi160) == BMI160_OK)
            MainMenu->terminalSend("\n\r>BMI160_CONFIG/::SUCCESS::");
            else MainMenu->terminalSend("\n\r>BMI160_CONFIG/::FAILED::");
            break;
    default: MainMenu->terminalSend("\n\r>BMI160_CONFIG/::Incorrect value!");
            break;
    } 
    BMI160_set_Mode(&bmi160,sensor_status.accel_pwr_mode,sensor_status.gyro_pwr_mode);
    BMI160_CONFIG_FLAG = true;
    MainMenu->terminalSend("\n\r");
    Display_Array(MainMenu->terminalSend,bmi160Menu);
    MainMenu->terminalSend("\n\r>BMI160_CONFIG/Type message: ");        
}

static inline void AccelRange_menu (struct MainMenu_Struct *MainMenu)
{
A_RANGE_CONFIG_FLAG = false;
                    
    sensor_status.accel_pwr_mode = bmi160.accel_cfg.power;
    sensor_status.gyro_pwr_mode = bmi160.gyro_cfg.power;
    BMI160_set_Mode(&bmi160,BMI160_ACCEL_NORMAL_MODE,BMI160_GYRO_NORMAL_MODE);

    switch (atoi((char*)USART1_RxBuf))
    {
    case 2:bmi160.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
            if (bmi160_set_sens_conf(&bmi160) == BMI160_OK)
            MainMenu->terminalSend("\n\r>BMI160_CONFIG/::SUCCESS::");
            else MainMenu->terminalSend("\n\r>BMI160_CONFIG/::FAILED::");
            break;
    case 4:bmi160.accel_cfg.range = BMI160_ACCEL_RANGE_4G;
            if (bmi160_set_sens_conf(&bmi160) == BMI160_OK)
            MainMenu->terminalSend("\n\r>BMI160_CONFIG/::SUCCESS::");
            else MainMenu->terminalSend("\n\r>BMI160_CONFIG/::FAILED::");
            break;        
    case 8:bmi160.accel_cfg.range = BMI160_ACCEL_RANGE_8G;
            if (bmi160_set_sens_conf(&bmi160) == BMI160_OK)
            MainMenu->terminalSend("\n\r>BMI160_CONFIG/::SUCCESS::");
            else MainMenu->terminalSend("\n\r>BMI160_CONFIG/::FAILED::");
            break;
    case 16:bmi160.accel_cfg.range = BMI160_ACCEL_RANGE_16G;
            if (bmi160_set_sens_conf(&bmi160) == BMI160_OK)
            MainMenu->terminalSend("\n\r>BMI160_CONFIG/::SUCCESS::");
            else MainMenu->terminalSend("\n\r>BMI160_CONFIG/::FAILED::");
            break;
    default: MainMenu->terminalSend("\n\r>BMI160_CONFIG/::Incorrect value!");
            break;
    } 
    BMI160_set_Mode(&bmi160,sensor_status.accel_pwr_mode,sensor_status.gyro_pwr_mode);
    BMI160_CONFIG_FLAG = true;
    MainMenu->terminalSend("\n\r");
    Display_Array(MainMenu->terminalSend,bmi160Menu);
    MainMenu->terminalSend("\n\r>BMI160_CONFIG/Type message: ");       
}

static inline void bmm150_menu (struct MainMenu_Struct *MainMenu)
{
   if(rsltBMM150 == BMM150_OK) 
   {
       switch (atoi((char*)USART1_RxBuf))
       {
        /*status*/
        case 1: Display_Time(MainMenu->terminalSend);
        MainMenu->terminalSend("Command comes: BMM150_STATUS\n\r");
        BMM150_display_status();
        MainMenu->terminalSend("\n\r\n\r>BMI160_CONFIG/Type message: ");
        break;
        /*set preset*/
        case 2: 
            bmm150_get_op_mode(&sensor_status.bmm150_pwr_mode,&bmm150);
            sprintf(buf,"\n\r>OP Mode = %#04x\n\r",sensor_status.bmm150_pwr_mode); 
            MainMenu->terminalSend(buf);        
                        /*normal mode */                        
            if (sensor_status.bmm150_pwr_mode != 0u) 
            {        
                MainMenu->terminalSend("\n\r\n\r\t!! NORMAL mode must be activated !!");
                MainMenu->terminalSend("\n\r\n\r>BMM150_CONFIG/Type message: ");
            }                
            else
            {                
            BMM150_PRESET_CONFIG_FLAG = true;
            BMM150_CONFIG_FLAG = false;
            Display_Time(MainMenu->terminalSend);
            MainMenu->terminalSend("Command comes: PRESET_CONFIG");
            MainMenu->terminalSend("\n\r\n\r\t:: Available BMM150 Presets ::");       
            Display_Array(MainMenu->terminalSend,bmm150PresetsMenu);
            MainMenu->terminalSend("\n\r>BMM150_CONFIG/Enter value: ");
            }                
        break;
        /*set datarate*/
        case 3: 
            bmm150_get_op_mode(&sensor_status.bmm150_pwr_mode,&bmm150);
            sprintf(buf,"\n\r>OP Mode = %#04x\n\r",sensor_status.bmm150_pwr_mode); 
            MainMenu->terminalSend(buf);        
                        /*normal mode */                        
            if (sensor_status.bmm150_pwr_mode != 0u) 
            {        
                MainMenu->terminalSend("\n\r\n\r\t!! NORMAL mode must be activated !!");
                MainMenu->terminalSend("\n\r\n\r>BMM150_CONFIG/Type message: ");
            }                
            else
            { 
                BMM150_DATARATE_CONFIG_FLAG = true;
                BMM150_CONFIG_FLAG = false;
                Display_Time(MainMenu->terminalSend);
                MainMenu->terminalSend("Command comes: DATARATE_CONFIG");
                MainMenu->terminalSend("\n\r\n\r\t:: Available BMM150 Datarates [Hz] ::");       
                Display_Array(MainMenu->terminalSend,bmm150DatarateMenu);
                MainMenu->terminalSend("\n\r>BMM150_CONFIG/Enter value: ");             
            }
        break;
        /*set XY-rep*/
        case 4: 
            bmm150_get_op_mode(&sensor_status.bmm150_pwr_mode,&bmm150);
            sprintf(buf,"\n\r>OP Mode = %#04x\n\r",sensor_status.bmm150_pwr_mode); 
            MainMenu->terminalSend(buf);        
                        /*normal mode */                        
            if (sensor_status.bmm150_pwr_mode != 0u) 
            {        
                MainMenu->terminalSend("\n\r\n\r\t!! NORMAL mode must be activated !!");
                MainMenu->terminalSend("\n\r\n\r>BMM150_CONFIG/Type message: ");
            }                
            else
            {  
                BMM150_XYREP_CONFIG_FLAG = true;
                BMM150_CONFIG_FLAG = false;
                Display_Time(MainMenu->terminalSend);
                MainMenu->terminalSend("Command comes: XY-REP_CONFIG");
                MainMenu->terminalSend("\n\r\n\r\t:: Available BMM150 XY-repetition ::");       
                Display_Array(MainMenu->terminalSend,bmm150XYrepMenu);
                MainMenu->terminalSend("\n\r>BMM150_CONFIG/Enter value: "); 
            }                
        break;
        /*set Z-rep*/
        case 5: 
            bmm150_get_op_mode(&sensor_status.bmm150_pwr_mode,&bmm150);
            sprintf(buf,"\n\r>OP Mode = %#04x\n\r",sensor_status.bmm150_pwr_mode); 
            MainMenu->terminalSend(buf);        
                        /*normal mode */                        
            if (sensor_status.bmm150_pwr_mode != 0u) 
            {        
                MainMenu->terminalSend("\n\r\n\r\t!! NORMAL mode must be activated !!");
                MainMenu->terminalSend("\n\r\n\r>BMM150_CONFIG/Type message: ");
            }                
            else
            {
                BMM150_ZREP_CONFIG_FLAG = true;
                BMM150_CONFIG_FLAG = false;
                Display_Time(MainMenu->terminalSend);
                MainMenu->terminalSend("Command comes: Z-REP_CONFIG");
                MainMenu->terminalSend("\n\r\n\r\t:: Available BMM150 Z-repetition ::");       
                Display_Array(MainMenu->terminalSend,bmm150ZrepMenu);
                MainMenu->terminalSend("\n\r>BMM150_CONFIG/Enter value: ");  
            }                
        break;
        default:
        /* normal mode */
        if (strncmp((char*)USART1_RxBuf,"NORMAL_MODE",11)==0)
        {
        Display_Time(MainMenu->terminalSend);
        MainMenu->terminalSend("Command comes: BMM150_NORMAL");
            if (BMM150_set_Mode(&bmm150,BMM150_NORMAL_MODE)==BMM150_OK)
            MainMenu->terminalSend("\t\t::SUCCESS::\n\r");
            else MainMenu->terminalSend("\t\t::FAILED::\n\r");  
            Display_Array(MainMenu->terminalSend,bmm150Menu);             
            MainMenu->terminalSend("\n\r>BMM150_CONFIG/Type message: ");
        }
        /* suspend mode */
        else if (strncmp((char*)USART1_RxBuf,"SUSPEND_MODE",12)==0)
        {
        Display_Time(MainMenu->terminalSend);
        MainMenu->terminalSend("Command comes: BMM150_SUSPEND");
            if (BMM150_set_Mode(&bmm150,BMM150_SUSPEND_MODE)==BMM150_OK)
            MainMenu->terminalSend("\t\t::SUCCESS::\n\r");
            else MainMenu->terminalSend("\t\t::FAILED::\n\r"); 
            Display_Array(MainMenu->terminalSend,bmm150Menu);            
            MainMenu->terminalSend("\n\r>BMM150_CONFIG/Type message: ");
        }
        /* sleep mode */
        else if (strncmp((char*)USART1_RxBuf,"SLEEP_MODE",10)==0)
        {
        Display_Time(MainMenu->terminalSend);
        MainMenu->terminalSend("Command comes: BMM150_SLEEP");
            if (BMM150_set_Mode(&bmm150,BMM150_SLEEP_MODE)==BMM150_OK)
            MainMenu->terminalSend("\t\t::SUCCESS::\n\r");
            else MainMenu->terminalSend("\t\t::FAILED::\n\r"); 
            Display_Array(MainMenu->terminalSend,bmm150Menu);            
            MainMenu->terminalSend("\n\r>BMM150_CONFIG/Type message: ");
        }
        /* forced mode */
        else if (strncmp((char*)USART1_RxBuf,"FORCED_MODE",11)==0)
        {
        Display_Time(MainMenu->terminalSend);
        MainMenu->terminalSend("Command comes: BMM150_FORCED");
            if (BMM150_set_Mode(&bmm150,BMM150_FORCED_MODE)==BMM150_OK)
            MainMenu->terminalSend("\t\t::SUCCESS::\n\r");
            else MainMenu->terminalSend("\t\t::FAILED::\n\r");
            Display_Array(MainMenu->terminalSend,bmm150Menu);            
            MainMenu->terminalSend("\n\r>BMM150_CONFIG/Type message: ");
        }         
        /* reset */
        else if (strncmp((char*)USART1_RxBuf,"RESET",5)==0)
        {
        Display_Time(MainMenu->terminalSend);
        MainMenu->terminalSend("Command comes: BMM150_RESET");
            if (bmm150_soft_reset(&bmm150)==BMM150_OK)
            {
            bmm150_get_op_mode(&bmm150.settings.pwr_mode,&bmm150); 
            bmm150_get_sensor_settings(&bmm150);
            bmm150.settings.preset_mode = 0;                
            MainMenu->terminalSend("\t\t::SUCCESS::");
            }
            else MainMenu->terminalSend("\t\t::FAILED::"); 
            Display_Array(MainMenu->terminalSend,bmm150Menu);            
            MainMenu->terminalSend("\n\r>BMM150_CONFIG/Type message: ");
        } 
        /* help */
        else if ((strncmp((char*)USART1_RxBuf,"help",4)==0) ||
                (strncmp((char*)USART1_RxBuf,"HELP",4)==0))
        {
        MainMenu->terminalSend("\n\r");
        Display_Array(MainMenu->terminalSend,bmm150Menu);
        MainMenu->terminalSend("\n\r>BMM150_CONFIG/Type message: ");      
        }  
        /*exit */
        else if ((strncmp((char*)USART1_RxBuf,"exit",4)==0) || 
                (strncmp((char*)USART1_RxBuf,"EXIT",4)==0))
        {
        MainMenu->terminalSend("\n\r>BMM150_CONFIG/");
        Display_Time(MainMenu->terminalSend);	
        MainMenu->terminalSend("Command comes: BMM150_EXIT\n\r");
        BMM150_CONFIG_FLAG = false;
        Display_Array(MainMenu->terminalSend,menu);
        MainMenu->terminalSend("\n\r>Type message: ");
        }
        /* STATUS */
        else if (strncmp((char*)USART1_RxBuf,"STATUS",6)==0)
        {
        Display_Time(MainMenu->terminalSend);
        MainMenu->terminalSend("Command comes: BMM150_STATUS\n\r");
        BMM150_display_status();
        MainMenu->terminalSend("\n\r\n\r>BMI160_CONFIG/Type message: ");    
        }
        /* SET_PRESET */
        else if (strncmp((char*)USART1_RxBuf,"SET_PRESET",10)==0)
        {
            bmm150_get_op_mode(&sensor_status.bmm150_pwr_mode,&bmm150);
            sprintf(buf,"\n\r>OP Mode = %#04x\n\r",sensor_status.bmm150_pwr_mode); 
            MainMenu->terminalSend(buf);        
                        /*normal mode */                        
            if (sensor_status.bmm150_pwr_mode != 0u) 
            {        
                MainMenu->terminalSend("\n\r\n\r\t!! NORMAL mode must be activated !!");
                MainMenu->terminalSend("\n\r\n\r>BMM150_CONFIG/Type message: ");
            }                
            else
            {                
            BMM150_PRESET_CONFIG_FLAG = true;
            BMM150_CONFIG_FLAG = false;
            Display_Time(MainMenu->terminalSend);
            MainMenu->terminalSend("Command comes: PRESET_CONFIG");
            MainMenu->terminalSend("\n\r\n\r\t:: Available BMM150 Presets ::");       
            Display_Array(MainMenu->terminalSend,bmm150PresetsMenu);
            MainMenu->terminalSend("\n\r>BMM150_CONFIG/Enter value: ");
            }                  
        }
         /* SET_DATARATE */
        else if (strncmp((char*)USART1_RxBuf,"SET_DATARATE",12)==0)
        {
            bmm150_get_op_mode(&sensor_status.bmm150_pwr_mode,&bmm150);
            sprintf(buf,"\n\r>OP Mode = %#04x\n\r",sensor_status.bmm150_pwr_mode); 
            MainMenu->terminalSend(buf);        
                        /*normal mode */                        
            if (sensor_status.bmm150_pwr_mode != 0u) 
            {        
                MainMenu->terminalSend("\n\r\n\r\t!! NORMAL mode must be activated !!");
                MainMenu->terminalSend("\n\r\n\r>BMM150_CONFIG/Type message: ");
            }                
            else
            { 
                BMM150_DATARATE_CONFIG_FLAG = true;
                BMM150_CONFIG_FLAG = false;
                Display_Time(MainMenu->terminalSend);
                MainMenu->terminalSend("Command comes: DATARATE_CONFIG");
                MainMenu->terminalSend("\n\r\n\r\t:: Available BMM150 Datarates [Hz] ::");       
                Display_Array(MainMenu->terminalSend,bmm150DatarateMenu);
                MainMenu->terminalSend("\n\r>BMM150_CONFIG/Enter value: ");             
            }   
        }
        /* SET_XY-rep */
        else if (strncmp((char*)USART1_RxBuf,"SET_XY-rep",10)==0)
        {
            bmm150_get_op_mode(&sensor_status.bmm150_pwr_mode,&bmm150);
            sprintf(buf,"\n\r>OP Mode = %#04x\n\r",sensor_status.bmm150_pwr_mode); 
            MainMenu->terminalSend(buf);        
                        /*normal mode */                        
            if (sensor_status.bmm150_pwr_mode != 0u) 
            {        
                MainMenu->terminalSend("\n\r\n\r\t!! NORMAL mode must be activated !!");
                MainMenu->terminalSend("\n\r\n\r>BMM150_CONFIG/Type message: ");
            }                
            else
            {  
                BMM150_XYREP_CONFIG_FLAG = true;
                BMM150_CONFIG_FLAG = false;
                Display_Time(MainMenu->terminalSend);
                MainMenu->terminalSend("Command comes: XY-REP_CONFIG");
                MainMenu->terminalSend("\n\r\n\r\t:: Available BMM150 XY-repetition ::");       
                Display_Array(MainMenu->terminalSend,bmm150XYrepMenu);
                MainMenu->terminalSend("\n\r>BMM150_CONFIG/Enter value: "); 
            }                
        }
        /* SET_Z-rep */
        else if (strncmp((char*)USART1_RxBuf,"SET_Z-rep",9)==0)
        {
            bmm150_get_op_mode(&sensor_status.bmm150_pwr_mode,&bmm150);
            sprintf(buf,"\n\r>OP Mode = %#04x\n\r",sensor_status.bmm150_pwr_mode); 
            MainMenu->terminalSend(buf);        
                        /*normal mode */                        
            if (sensor_status.bmm150_pwr_mode != 0u) 
            {        
                MainMenu->terminalSend("\n\r\n\r\t!! NORMAL mode must be activated !!");
                MainMenu->terminalSend("\n\r\n\r>BMM150_CONFIG/Type message: ");
            }                
            else
            {
                BMM150_ZREP_CONFIG_FLAG = true;
                BMM150_CONFIG_FLAG = false;
                Display_Time(MainMenu->terminalSend);
                MainMenu->terminalSend("Command comes: Z-REP_CONFIG");
                MainMenu->terminalSend("\n\r\n\r\t:: Available BMM150 Z-repetition ::");       
                Display_Array(MainMenu->terminalSend,bmm150ZrepMenu);
                MainMenu->terminalSend("\n\r>BMM150_CONFIG/Enter value: ");  
            }                     
        }
        /* ADVANCED_SELFTEST */
        else if (strncmp((char*)USART1_RxBuf,"ADVANCED_SELFTEST",17)==0)
        {
        int8_t rsltAdvSelfTest = -1;
        int32_t adv_self_test_Value = 0;
            
            /* save current sensor's values */ 
            bmm150_get_op_mode(&sensor_status.bmm150_pwr_mode,&bmm150);    
            //bmm150_get_sensor_settings(&bmm150);
            sensor_status.bmm150_data_rate = bmm150.settings.data_rate;
            sensor_status.bmm150_xy_rep = bmm150.settings.xy_rep;
            sensor_status.bmm150_z_rep = bmm150.settings.z_rep;
            sensor_status.bmm150_preset_mode = bmm150.settings.preset_mode;
            
        rsltAdvSelfTest = bmm150_perform_self_test(BMM150_ADVANCED_SELF_TEST, &bmm150, &adv_self_test_Value);
            if (rsltAdvSelfTest == BMM150_OK) 	
            MainMenu->terminalSend("\n\r\t> ADVANCED SELF TEST:\tSUCCESSFULL\r\n");
            else MainMenu->terminalSend("\n\r\t> ADVANCED SELF TEST:\tFAILED\n\r");
        sprintf((char*)USART1_TxBuf, "\tErroR_Code: %d Value: %d\r\n", rsltAdvSelfTest,adv_self_test_Value );			
        MainMenu->terminalSend(USART1_TxBuf);
        //ends with soft reset so Initialization again needed
        BMM150_init();
        BMM150_set_Mode(&bmm150, sensor_status.bmm150_pwr_mode);
            if (BMM150_set_Preset(&bmm150,sensor_status.bmm150_preset_mode)!= BMM150_OK)
            {
                bmm150.settings.data_rate = sensor_status.bmm150_data_rate;
                bmm150.settings.xy_rep = sensor_status.bmm150_xy_rep;
                bmm150.settings.z_rep = sensor_status.bmm150_z_rep;
                set_odr(&bmm150);
                set_xy_rep(&bmm150);
                set_z_rep(&bmm150);
            }
        MainMenu->terminalSend("\n\r>BMM150_CONFIG/Type message: ");    
        }
        
        else
        {
        MainMenu->terminalSend("\n\r>BMM150_CONFIG/");
        Display_Time(MainMenu->terminalSend);
        MainMenu->terminalSend("::Unknown command::");    
        MainMenu->terminalSend("\n\r>BMM150_CONFIG/Type message: ");
        }    
        break;
       }                
   }       
   else
   {
   MainMenu->terminalSend("\n\r\t\t>BMM150 NOT INITED!!\n\r");
   BMM150_CONFIG_FLAG = false;
   } 
}

static inline void bmm150Preset_menu (struct MainMenu_Struct *MainMenu)
{
BMM150_PRESET_CONFIG_FLAG = false;

    {
        switch (atoi((char*)USART1_RxBuf))
        {
            /* LOWPOWER */
        case 1: 
                if (BMM150_set_Preset(&bmm150,BMM150_PRESETMODE_LOWPOWER) == BMM150_OK)
                MainMenu->terminalSend("\n\r\tLOWPOWER preset set ::SUCCESS::");
                else if (BMM150_set_Preset(&bmm150,BMM150_PRESETMODE_LOWPOWER) == -1)
                    MainMenu->terminalSend("\n\r>BMM150_CONFIG/::INCORRECT POWER MODE::");
                else MainMenu->terminalSend("\n\r\tLOWPOWER preset set ::FAILED::");
            break; 
                /* REGULAR */
        case 2:
                if (BMM150_set_Preset(&bmm150,BMM150_PRESETMODE_REGULAR) == BMM150_OK)
                MainMenu->terminalSend("\n\r\tREGULAR preset set ::SUCCESS::");
                else if (BMM150_set_Preset(&bmm150,BMM150_PRESETMODE_LOWPOWER) == -1)
                    MainMenu->terminalSend("\n\r>BMM150_CONFIG/::INCORRECT POWER MODE::");
                else MainMenu->terminalSend("\n\r\tREGULAR preset set ::FAILED::");
            break;
        /* HIGHACCURACY */
        case 3:
                if (BMM150_set_Preset(&bmm150,BMM150_PRESETMODE_HIGHACCURACY) == BMM150_OK)
                MainMenu->terminalSend("\n\r\tHIGHACCURACY preset set ::SUCCESS::");
                else if (BMM150_set_Preset(&bmm150,BMM150_PRESETMODE_LOWPOWER) == -1)
                    MainMenu->terminalSend("\n\r>BMM150_CONFIG/::INCORRECT POWER MODE::");
                else MainMenu->terminalSend("\n\r\tHIGHACCURACY preset set ::FAILED::");
            break;
        /* ENHANCED */
        case 4:
                if (BMM150_set_Preset(&bmm150,BMM150_PRESETMODE_ENHANCED) == BMM150_OK)
                MainMenu->terminalSend("\n\r\tENHANCED preset set ::SUCCESS::");
                else if (BMM150_set_Preset(&bmm150,BMM150_PRESETMODE_LOWPOWER) == -1)
                    MainMenu->terminalSend("\n\r>BMM150_CONFIG/::INCORRECT POWER MODE::");
                else MainMenu->terminalSend("\n\r\tENHANCED preset set ::FAILED::");
            break;
        default:MainMenu->terminalSend("\n\r>BMM150_CONFIG/::Incorrect value!");         
            break;
        }
    }
    BMM150_CONFIG_FLAG = true;
    MainMenu->terminalSend("\n\r");
    Display_Array(MainMenu->terminalSend,bmm150Menu);
    MainMenu->terminalSend("\n\r>BMM150_CONFIG/Type message: ");
}

static inline void bmm150Datarate_menu (struct MainMenu_Struct *MainMenu)
{
    BMM150_DATARATE_CONFIG_FLAG = false;

        switch (atoi((char*)USART1_RxBuf))
        {
        /* 2 Hz */
        case 2: 
            bmm150.settings.data_rate = BMM150_DATA_RATE_02HZ;
                if (set_odr(&bmm150) == BMM150_OK)
                {
                bmm150.settings.preset_mode = 0; 
                MainMenu->terminalSend("\n\r\tDATARATE 2 HZ set ::SUCCESS::");
                }
                else MainMenu->terminalSend("\n\r\tDATARATE 2 HZ set ::FAILED::");
            break; 
        /* 6 Hz */
        case 6:
            bmm150.settings.data_rate = BMM150_DATA_RATE_06HZ;
                if (set_odr(&bmm150) == BMM150_OK)
                {
                bmm150.settings.preset_mode = 0; 
                MainMenu->terminalSend("\n\r\tDATARATE 6 HZ set ::SUCCESS::");
                }
                else MainMenu->terminalSend("\n\r\tDATARATE 6 HZ set ::FAILED::");
            break;
        /* 8 Hz*/
        case 8:
             bmm150.settings.data_rate = BMM150_DATA_RATE_08HZ;
                if (set_odr(&bmm150) == BMM150_OK)
                {
                bmm150.settings.preset_mode = 0; 
                MainMenu->terminalSend("\n\r\tDATARATE 8 HZ set ::SUCCESS::");
                }
                else MainMenu->terminalSend("\n\r\tDATARATE 8 HZ set ::FAILED::");
            break;
        /* 10 Hz */
        case 10:
             bmm150.settings.data_rate = BMM150_DATA_RATE_10HZ;
                if (set_odr(&bmm150) == BMM150_OK)
                {
                bmm150.settings.preset_mode = 0; 
                MainMenu->terminalSend("\n\r\tDATARATE 10 HZ set ::SUCCESS::");
                }
                else MainMenu->terminalSend("\n\r\tDATARATE 10 HZ set ::FAILED::");
            break;
        /* 15 Hz */
        case 15:
             bmm150.settings.data_rate = BMM150_DATA_RATE_15HZ;
                if (set_odr(&bmm150) == BMM150_OK)
                {
                bmm150.settings.preset_mode = 0; 
                MainMenu->terminalSend("\n\r\tDATARATE 15 HZ set ::SUCCESS::");
                }
                else MainMenu->terminalSend("\n\r\tDATARATE 15 HZ set ::FAILED::");
            break;
        /* 20 Hz */
        case 20:
             bmm150.settings.data_rate = BMM150_DATA_RATE_20HZ;
                if (set_odr(&bmm150) == BMM150_OK)
                {
                bmm150.settings.preset_mode = 0; 
                MainMenu->terminalSend("\n\r\tDATARATE 20 HZ set ::SUCCESS::");
                }
                else MainMenu->terminalSend("\n\r\tDATARATE 20 HZ set ::FAILED::");
            break;
        /* 25 Hz */
        case 25:
             bmm150.settings.data_rate = BMM150_DATA_RATE_25HZ;
                if (set_odr(&bmm150) == BMM150_OK)
                {
                bmm150.settings.preset_mode = 0; 
                MainMenu->terminalSend("\n\r\tDATARATE 25 HZ set ::SUCCESS::");
                }
                else MainMenu->terminalSend("\n\r\tDATARATE 25 HZ set ::FAILED::");
            break;
        /* 30 Hz */
        case 30:
             bmm150.settings.data_rate = BMM150_DATA_RATE_30HZ;
                if (set_odr(&bmm150) == BMM150_OK)
                {
                bmm150.settings.preset_mode = 0;  
                MainMenu->terminalSend("\n\r\tDATARATE 30 HZ set ::SUCCESS::");
                }
                else MainMenu->terminalSend("\n\r\tDATARATE 30 HZ set ::FAILED::");
            break;
                
        default:MainMenu->terminalSend("\n\r>BMM150_CONFIG/::Incorrect value!");         
            break;
        }
    BMM150_CONFIG_FLAG = true;
    MainMenu->terminalSend("\n\r");
    Display_Array(MainMenu->terminalSend,bmm150Menu);
    MainMenu->terminalSend("\n\r>BMM150_CONFIG/Type message: ");
}


static inline void bmm150XYrep_menu (struct MainMenu_Struct *MainMenu)
{
BMM150_XYREP_CONFIG_FLAG = false;
    
uint16_t temp = atoi((char*)USART1_RxBuf);
    
    if ((temp % 2 == 0) || (temp > 511) || (temp < 1))
        MainMenu->terminalSend("\n\r>BMM150_CONFIG/::Incorrect value!");

    else 
    {
    uint8_t tempb = (temp-1)/2;
    bmm150.settings.xy_rep = tempb;        
        if (set_xy_rep(&bmm150) == BMM150_OK)
        {
            bmm150.settings.preset_mode = 0;  
            sprintf(buf,"\n\r\t\t%#04x set ::SUCCESS::",tempb);
        }
        else sprintf(buf,"\n\r\t\t%#04x set ::FAILED::",tempb);
            MainMenu->terminalSend(buf);   
    }
    BMM150_CONFIG_FLAG = true;
    MainMenu->terminalSend("\n\r");
    Display_Array(MainMenu->terminalSend,bmm150Menu);
    MainMenu->terminalSend("\n\r>BMM150_CONFIG/Type message: ");
}

static inline void bmm150Zrep_menu (struct MainMenu_Struct *MainMenu)
{
BMM150_ZREP_CONFIG_FLAG = false;
    
uint16_t temp = atoi((char*)USART1_RxBuf);
    
    if ((temp > 256) || (temp < 1))
        MainMenu->terminalSend("\n\r>BMM150_CONFIG/::Incorrect value!");

    else 
    {
    uint8_t tempb = temp-1;
    bmm150.settings.z_rep = tempb;        
        if (set_z_rep(&bmm150) == BMM150_OK)
        {
            bmm150.settings.preset_mode = 0;  
            sprintf(buf,"\n\r\t\t%#04x set ::SUCCESS::",tempb);
        }
        else sprintf(buf,"\n\r\t\t%#04x set ::FAILED::",tempb);
            MainMenu->terminalSend(buf);   
    }
    BMM150_CONFIG_FLAG = true;
    MainMenu->terminalSend("\n\r");
    Display_Array(MainMenu->terminalSend,bmm150Menu);
    MainMenu->terminalSend("\n\r>BMM150_CONFIG/Type message: ");
}


static inline void USART2_menu (struct MainMenu_Struct *MainMenu)
{
sscanf((char*)USART1_RxBuf,"%s%d",buf,&num);
					
    if ((strncmp((char*)USART1_RxBuf,"exit",4)==0) ||
        (strncmp((char*)USART1_RxBuf,"EXIT",4)==0))
    {
    MainMenu->terminalSend("\n\r>USART2_CONFIG/");
    Display_Time(MainMenu->terminalSend);	
    MainMenu->terminalSend("Command comes: USART2_EXIT\n\r");
    USART2_CONFIG_FLAG = false;
    Display_Array(MainMenu->terminalSend,menu);
    MainMenu->terminalSend("\n\r>Type message: ");	
    }
    
    else if (strncmp((char*)USART1_RxBuf,"STATUS",6)==0)
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
    
    else if (strncmp(buf,"BAUDRATE",8)==0)
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
    
    else if (strncmp(buf,"PARITY",6)==0)
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
    
    else if (strncmp(buf,"STOPBIT",7)==0)
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


static inline void BLE_menu (struct MainMenu_Struct *MainMenu)
{
    if (strncmp((char*)USART1_RxBuf,"exit",4)==0)	
    {
    MainMenu->terminalSend("\n\r>AT_CONFIG/");
    Display_Time(MainMenu->terminalSend);	
    MainMenu->terminalSend("Command comes: AT_EXIT\n\r");
    AT_CONFIG_FLAG = false;
    GPIO_SetBits(LED_PORT,LED_PIN);
    Display_Array(MainMenu->terminalSend,menu);
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

static inline void Main_menu (struct MainMenu_Struct *MainMenu)
{      
    MainMenu->terminalSend("\n\r>");
    Display_Time(MainMenu->terminalSend);
    MainMenu->terminalSend(USART1_RxBuf);
        
    switch (atoi((char*)USART1_RxBuf))
    {
        /*BMI160*/
        case 1: 
            if (rsltBMI160 == BMI160_OK)
            {
            BMI160_CONFIG_FLAG = true;
            MainMenu->terminalSend("\n\r");
            Display_Time(MainMenu->terminalSend);
            MainMenu->terminalSend("Command comes: BMI160_CONFIG\n\r");
            Display_Array(MainMenu->terminalSend,bmi160Menu);
            MainMenu->terminalSend("\n\r>BMI160_CONFIG/Type message: ");
            }
            else MainMenu->terminalSend("\n\r\t !!BMI160 NOT INITED !! \n\r");
        break;
        /*BMM150*/
        case 2: 
            if (rsltBMM150 == BMM150_OK)
            {
            BMM150_CONFIG_FLAG = true;
            Display_Time(MainMenu->terminalSend);
            MainMenu->terminalSend("Command comes: BMM150_CONFIG\n\r");
            Display_Array(MainMenu->terminalSend,bmm150Menu);
            MainMenu->terminalSend("\n\r>BMM150_CONFIG/Type message: ");
            }
            else MainMenu->terminalSend("\n\r\t !!BMM150 NOT INITED !! \n\r");
        break;
        /*LED ON*/
        case 3: MainMenu->terminalSend("\n\r");
        Display_Time(MainMenu->terminalSend);
        MainMenu->terminalSend("Command comes: LED_ON\n\r");
        LED_PORT->BRR = LED_PIN;
        break;
        /*LED OFF*/
        case 4: MainMenu->terminalSend("\n\r");
        Display_Time(MainMenu->terminalSend);
        MainMenu->terminalSend("Command comes: LED_OFF\n\r");
        LED_PORT->BSRR = LED_PIN;
        break;
        /*BLE*/               
        case 5: AT_CONFIG_FLAG = true;
        GPIO_ResetBits(LED_PORT,LED_PIN);
        MainMenu->terminalSend("\n\r");
        Display_Time(MainMenu->terminalSend);
        MainMenu->terminalSend("Mode activated: AT_CONFIG\n\r");
        Display_Array(MainMenu->terminalSend,AT_commands);	
        MainMenu->terminalSend("\n\r>AT_CONFIG/Type message: ");
        break;
        /*usart2*/                       
        case 6: USART2_CONFIG_FLAG = true;
        MainMenu->terminalSend("\n\r");
        Display_Time(MainMenu->terminalSend);
        MainMenu->terminalSend("Command comes: USART2_CONFIG\n\r");
        Display_Array(MainMenu->terminalSend,usart2_menu);	
        MainMenu->terminalSend("\n\r>USART2_CONFIG/Type message: ");
        break;
        /*start raw measurement*/
        case 7: 
        LED_PORT->BRR = LED_PIN;
        Display_Time(MainMenu->terminalSend);
        MainMenu->terminalSend("Command comes: START_RAW\n\r");
        BMI160_set_Mode(&bmi160,BMI160_ACCEL_NORMAL_MODE,BMI160_GYRO_NORMAL_MODE);    
        break;
        /*stop any measurement*/
        case 8:
        BMI160_set_Mode(&bmi160,BMI160_ACCEL_SUSPEND_MODE,BMI160_GYRO_SUSPEND_MODE);
        MADGWICK_FILTER_FLAG = false;
        MAHONY_FILTER_FLAG = false;
        LED_PORT->BSRR = LED_PIN;  
        Display_Time(MainMenu->terminalSend);
        MainMenu->terminalSend("Command comes: STOP\n\r");        
        break;
        /* start Madgwick filtered measurement */
        case 9:
        MADGWICK_FILTER_FLAG = true;
        LED_PORT->BRR = LED_PIN;
        Display_Time(MainMenu->terminalSend);
        MainMenu->terminalSend("Command comes: START_MADGWICK\n\r");
        BMI160_set_Mode(&bmi160,BMI160_ACCEL_NORMAL_MODE,BMI160_GYRO_NORMAL_MODE);        
        break;
        /* start Mahony filtered measurement */
        case 10:
        MAHONY_FILTER_FLAG = true;
        LED_PORT->BRR = LED_PIN;
        Display_Time(MainMenu->terminalSend);
        MainMenu->terminalSend("Command comes: START_MAHONY\n\r");
        BMI160_set_Mode(&bmi160,BMI160_ACCEL_NORMAL_MODE,BMI160_GYRO_NORMAL_MODE);   
        break;
        
        /*help*/                
        case 11: MainMenu->terminalSend("\n\r");
        Display_Time(MainMenu->terminalSend);
        MainMenu->terminalSend("Command comes: HELP\n\r");
        Display_Array(MainMenu->terminalSend,menu);	
        break;
        /*exit*/                
            
    }
    
    str_toupper((char*)USART1_RxBuf);    // Make incomming string Upper register
    
    /* HELP */    
    if ((strncmp(((char*)USART1_RxBuf),"HELP",4)==0))
    {
        MainMenu->terminalSend("\n\r");
        Display_Time(MainMenu->terminalSend);
        MainMenu->terminalSend("Command comes: HELP\n\r");
        Display_Array(MainMenu->terminalSend,menu);	
    }
    /* BMI160_conf */
    if (strncmp(((char*)USART1_RxBuf),"BMI160_CONF",11)==0)
    {
        if (rsltBMI160 == BMI160_OK)
            {
            BMI160_CONFIG_FLAG = true;
            MainMenu->terminalSend("\n\r");
            Display_Time(MainMenu->terminalSend);
            MainMenu->terminalSend("Command comes: BMI160_CONFIG\n\r");
            Display_Array(MainMenu->terminalSend,bmi160Menu);
            MainMenu->terminalSend("\n\r>BMI160_CONFIG/Type message: ");
            }
        else MainMenu->terminalSend("\n\r\t !!BMI160 NOT INITED !! \n\r");	
    }
    /* BMM150_conf */
    if (strncmp(((char*)USART1_RxBuf),"BMM150_CONF",11)==0)
    {
         if (rsltBMM150 == BMM150_OK)
            {
            BMM150_CONFIG_FLAG = true;
            Display_Time(MainMenu->terminalSend);
            MainMenu->terminalSend("Command comes: BMM150_CONFIG\n\r");
            Display_Array(MainMenu->terminalSend,bmm150Menu);
            MainMenu->terminalSend("\n\r>BMM150_CONFIG/Type message: ");
            }
            else MainMenu->terminalSend("\n\r\t !!BMM150 NOT INITED !! \n\r");
    }
    /* LED_on */
    if (strncmp(((char*)USART1_RxBuf),"LED_ON",6)==0)
    {
        MainMenu->terminalSend("\n\r");
        Display_Time(MainMenu->terminalSend);
        MainMenu->terminalSend("Command comes: LED_ON\n\r");
        LED_PORT->BRR = LED_PIN;
    }
    /* LED_off */
    if (strncmp(((char*)USART1_RxBuf),"LED_OFF",7)==0)
    {
        MainMenu->terminalSend("\n\r");
        Display_Time(MainMenu->terminalSend);
        MainMenu->terminalSend("Command comes: LED_OFF\n\r");
        LED_PORT->BSRR = LED_PIN;  
    }
    /* BLE_conf */
    if (strncmp(((char*)USART1_RxBuf),"BLE_CONF",8)==0)
    {
        AT_CONFIG_FLAG = true;
        GPIO_ResetBits(LED_PORT,LED_PIN);
        MainMenu->terminalSend("\n\r");
        Display_Time(MainMenu->terminalSend);
        MainMenu->terminalSend("Mode activated: AT_CONFIG\n\r");
        Display_Array(MainMenu->terminalSend,AT_commands);	
        MainMenu->terminalSend("\n\r>AT_CONFIG/Type message: ");    
    }
    /* USART2_conf */
    if (strncmp(((char*)USART1_RxBuf),"USART2_CONF",10)==0)
    {
        USART2_CONFIG_FLAG = true;
        MainMenu->terminalSend("\n\r");
        Display_Time(MainMenu->terminalSend);
        MainMenu->terminalSend("Command comes: USART2_CONFIG\n\r");
        Display_Array(MainMenu->terminalSend,usart2_menu);	
        MainMenu->terminalSend("\n\r>USART2_CONFIG/Type message: ");
    }
    /* Start_RAW */
    if (strncmp(((char*)USART1_RxBuf),"START_RAW",9)==0)
    {
    Display_Time(MainMenu->terminalSend);
    MainMenu->terminalSend("Command comes: START_RAW\n\r");
    LED_PORT->BRR = LED_PIN;
    BMI160_set_Mode(&bmi160,BMI160_ACCEL_NORMAL_MODE,BMI160_GYRO_NORMAL_MODE);    
    }
    /* STOP */
    if (strncmp(((char*)USART1_RxBuf),"STOP",4)==0)
    {
    BMI160_set_Mode(&bmi160,BMI160_ACCEL_SUSPEND_MODE,BMI160_GYRO_SUSPEND_MODE);
    MADGWICK_FILTER_FLAG = false;
    MAHONY_FILTER_FLAG = false;
    LED_PORT->BSRR = LED_PIN;
    Display_Time(MainMenu->terminalSend);
    MainMenu->terminalSend("Command comes: STOP\n\r");
    }
    /* Start Madgwick */
    if (strncmp(((char*)USART1_RxBuf),"START_MADGWICK",14)==0)
    {
    Display_Time(MainMenu->terminalSend);
    MainMenu->terminalSend("Command comes: START_MADGWICK\n\r");
    MADGWICK_FILTER_FLAG = true;
    LED_PORT->BRR = LED_PIN;
    BMI160_set_Mode(&bmi160,BMI160_ACCEL_NORMAL_MODE,BMI160_GYRO_NORMAL_MODE);   
    }
    /* Start Mahony */
    if (strncmp(((char*)USART1_RxBuf),"START_MAHONY",12)==0)
    {
    Display_Time(MainMenu->terminalSend);
    MainMenu->terminalSend("Command comes: START_MAHONY\n\r");
    MAHONY_FILTER_FLAG = true;
    LED_PORT->BRR = LED_PIN;
    BMI160_set_Mode(&bmi160,BMI160_ACCEL_NORMAL_MODE,BMI160_GYRO_NORMAL_MODE);    
    }
    
    if (!AT_CONFIG_FLAG && !USART2_CONFIG_FLAG && !BMI160_CONFIG_FLAG && !BMM150_CONFIG_FLAG) 
        MainMenu->terminalSend("\n\r>Type message: ");
}


