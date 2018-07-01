/*==============================================================================
>Project name:  Project_One
>Brief:         Firmware for getting data from gyro+accel bmi160 BMI160 and
                magnetometr , filter that values with MadgwickAHRS and send via 
                Bluetooth HC-05 and USART1 STM32F103C8 to COM port (Serial_)
>Author:        Berezhanov M.
>Date:          29.01.2018
>Version:       0.1
==============================================================================*/

//----------------------------= #[I]ncludes =-----------------------------------
#include "Main.h"					// all project defines there
#include "Menu.h"					// lib for MainMenu display via USART Terminal 
#include "stm32f10x_Inits.h"		// periph devices libs
#include "Sensors_func.h"			// lib for BMI160 and BMM150
#include "IRQ_Handlers.h"
//-----------------------------= #[D]efines =-----------------------------------

				/*watch Main.h. All project settings defines there*/

//---------------------------= 101[V]ariables =---------------------------------
extern struct bmi160_dev bmi160;
extern struct bmm150_dev bmm150;
extern struct Sensors_common_status sensor_status;

volatile char currTime[9] = __TIME__;
volatile char currDate[11] = __DATE__ ;

extern volatile char USART1_TxBuf[TX_BUF_SIZE];
extern USART_InitTypeDef USART2_InitStructure;
extern int8_t rsltBMI160;
extern int8_t rsltBMM150;
extern struct MainMenu_Struct MainMenu;
extern struct bmm150_raw_mag_data raw;
uint8_t accel_mode;
uint8_t gyro_mode;
uint8_t magni_mode;

volatile bool SENSOR_FLAG_DATA_RECIEVED = false;
volatile bool DISPLAY_SENSOR_DATA_FLAG = false;

//****DEBUG****
_Bool state = 0;
int spi_TxData = 11;

//----------------------------= [P]rototypes ; =--------------------------------


//------------------------------= functions =-----------------------------------
int main (void)                 // >>>_MAIN_<<<
{
//SystemInit();    
//SetSysClockTo_72();   
//SetSysClockToHSE();	
    
USART1_init();	
USART2_init();
/* Main Menu init */
MainMenu.terminalSend = USART1_SEND;
#ifdef DMA_USART2_EN
MainMenu.hc05Send = USART2_SEND_DMA;
#else 
MainMenu.hc05Send = USART2_SEND;
#endif
    
MainMenu.usart2Struct = &USART2_InitStructure;

SPI2_MASTER_init();
 
DBGMCU->CR |= DBGMCU_CR_DBG_SLEEP;  //Enable debug while sleepmode active
    
/* sensors init settings */
//bmm150.settings.pwr_mode  =     BMM150_SUSPEND_MODE;
    
bmm150.settings.data_rate = BMM150_DATA_RATE_10HZ;
bmm150.settings.xy_rep = 0;
bmm150.settings.z_rep = 0; 

sensor_status.gyro_cfg_odr  =   BMI160_GYRO_ODR_400HZ;          //400HZ is MAX for Bluetooth module
sensor_status.gyro_cfg_bw =     BMI160_GYRO_BW_NORMAL_MODE;   
sensor_status.gyro_cfg_range =  BMI160_GYRO_RANGE_250_DPS;
sensor_status.accel_cfg_odr =   BMI160_ACCEL_ODR_400HZ;
sensor_status.accel_cfg_bw =    BMI160_ACCEL_BW_NORMAL_AVG4;  
sensor_status.accel_cfg_range = BMI160_ACCEL_RANGE_2G;
  
rsltBMI160 = BMI160_init();		
rsltBMM150 = BMM150_init();
			
GPIO_init();
	
MainMenu_startMess(&MainMenu);

/*sleep mode config. Sleep after exiting from any NVIC Interrupt comes */
//NVIC_SystemLPConfig(NVIC_LP_SLEEPONEXIT, ENABLE);

	while(1)
	{	
    //__WFI();    // Sleep mode (waiting for Interrupt)      
	} //end of while

}//end of main

