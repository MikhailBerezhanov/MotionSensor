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
#include "BMI160_func.h"			// lib for BMI160 and BMM150
//-----------------------------= #[D]efines =-----------------------------------

				/*watch Main.h. All project settings defines there*/

//---------------------------= 101[V]ariables =---------------------------------
extern struct bmi160_dev bmi160;
extern struct bmi160_int_settg int_config;
extern struct bmi160_pmu_status pmu_status;	
extern struct bmi160_sensor_data accel;
extern struct bmi160_sensor_data gyro;	
extern struct bmi160_common_status sensor_status;
extern struct bmm150_dev bmm150;

extern USART_InitTypeDef USART2_InitStructure;
extern int8_t rslt;

struct MainMenu_Struct MainMenu;

extern volatile char USART1_RxBuf[RX_BUF_SIZE];		//buffer for data Recived from PC's Terminal 
extern volatile char USART2_RxBuf[RX_BUF_SIZE];
extern volatile char USART1_TxBuf[TX_BUF_SIZE];
extern volatile char USART2_TxBuf[TX_BUF_SIZE];

volatile bool RX1_FLAG_END_LINE = false;
volatile bool RX2_FLAG_END_LINE = false;
volatile bool SENSOR_FLAG_DATA_RECIEVED = false;

extern volatile char RX1i;		//index of data buffer
extern volatile char RX2i;
//****DEBUG****
_Bool state = 0;
int spi_TxData = 11;

//----------------------------= [P]rototypes ; =--------------------------------
void MainMenu_init(void);

//------------------------------= functions =-----------------------------------
int main (void)                 // >>>_MAIN_<<<
{
//SystemInit();    
SetSysClockTo_72();
	
USART1_init();	
USART2_init();
USART1_SEND("\r\n Привет,мужик!\r\nЯ готов к работе.\n\r");

MainMenu_init();
	
SPI2_MASTER_init();

rslt = BMI160_init();		

if (rslt != BMI160_OK) USART1_SEND("\r\n\tBMI160 INIT: F A I L E D\r\n");
else USART1_SEND("\r\n\tBMI160 INIT: SUCCESSFULL\r\n");
BMI160_display_status();
	
GPIO_init();
	

if (MainMenu_RTC_init())  USART1_SEND("\r\nReal time activated");
	MainMenu_startMess(&MainMenu);
	
	while(1)
	{
	MainMenu_checkInput(&MainMenu);
	MainMenu_checkAnswer(&MainMenu);	
     /*  
    sprintf(usart_TxBuf,"\r\n MAG DATA X : %d \r\n", bmm150.data.x);
    USART1_SEND(usart_TxBuf);
    sprintf(usart_TxBuf,"\r\n MAG DATA Y : %d \r\n", bmm150.data.y);
    USART1_SEND(usart_TxBuf);
    sprintf(usart_TxBuf,"\r\n MAG DATA Z : %d \r\n", bmm150.data.z);
    USART1_SEND(usart_TxBuf);
    _delay_ms(100);
    */    
		
		//**-------DEBUGGING-------**
	//SPI2_Read_BMI160(1,(BMI160_CHIP_ID_ADDR | BMI160_SPI_RD_MASK),&id,1);	
	//SPI2_Read_BMI160(1,(BMI160_ACCEL_DATA_ADDR | BMI160_SPI_RD_MASK),&id,3);	
	//SPI2_Write_BMI160(1,(BMI160_CHIP_ID_ADDR & BMI160_SPI_WR_MASK),&id,1);
	//SPI_Send(spi_TxData);
		
		if (SENSOR_FLAG_DATA_RECIEVED)
		{
		/*
		SENSOR_FLAG_DATA_RECIEVED = false;
		USART1_SEND("Orientation: ");
		sprintf(usart_TxBuf, "%f  ", fdata.heading );
		USART1_SEND(usart_TxBuf);
		sprintf(usart_TxBuf, "%f  ", fdata.pitch );
		USART1_SEND(usart_TxBuf);
		sprintf(usart_TxBuf, "%f\r\n", fdata.roll );
		USART1_SEND(usart_TxBuf);
		
		BMI160_display_data();	
		_delay_ms(100);
		*/
		
		//_delay_ms(100);
		}
		
		/*
		if (RX1_FLAG_END_LINE) 
		{           
		RX1_FLAG_END_LINE = false;		// Reset END_LINE Flag
		USART1_SEND("\r\nI have received a line:\r\n");
		USART1_SEND(USART1_RxBuf);
		USART1_SEND("\r\n");			//new line
						
			sscanf((char*)USART1_RxBuf,"%s%x",buffer,&spi_TxData);
			if (strncmp((char*)buffer,"spisend", 7) == 0)
			{
			SPI_Send(spi_TxData);
			sprintf(usart_TxBuf, "\r\n\tyou have send: %X", spi_TxData );
			USART1_SEND(usart_TxBuf);			
			USART1_SEND("\r\n");
			}
			
		Clear_Buffer(USART1_RxBuf);				//clear RX buffer
		}
		*/
	} //end of while

}//end of main


/**
  * @brief  IRQ Handler of BMI160 Dataready interrupt(comes to PB0)
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void) 
{
	/* Make sure that interrupt flag is set */
	if (EXTI_GetITStatus(EXTI_Line0) != RESET) 
	{
		GPIOA->BSRR = GPIO_Pin_0;		//**DEBUG**
		
		//SPI2_Read_BMI160(1,(BMI160_CHIP_ID_ADDR | BMI160_SPI_RD_MASK),&sensor_status.id,12);
		//bmi160_get_sensor_data(BMI160_BOTH_ACCEL_AND_GYRO,&accel,&gyro,&bmi160);
		//SPI2_Write_BMI160 (1,(BMI160_CHIP_ID_ADDR | BMI160_SPI_RD_MASK),&sensor_status.id,1);
		//BMI160_OptGet_data(&accel,&gyro);
		//filter_data(&fdata,&accel,&gyro);
		//BMI160_sendto_HC05();
				//SENSOR_FLAG_DATA_RECIEVED = 1;
		/* Clear interrupt flag */
		EXTI->PR = EXTI_Line0;
		
		GPIOA->BRR = GPIO_Pin_0;		//**DEBUG**	
	}
}

/**
  * @brief  IRQ Handler of Recieve USART1 
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)
{
    if ((USART1->SR & USART_FLAG_RXNE) != (u16)RESET)
    {
            USART1_RxBuf[RX1i] = USART_ReceiveData(USART1);

            if (USART1_RxBuf[RX1i] != 13) 
			{
                if (RX1i > RX_BUF_SIZE) Clear_Buffer(USART1_RxBuf); 
            }
            else RX1_FLAG_END_LINE = true;
			
            //Echo Чтобы видеть в терминале что вводится
            USART_SendData(USART1, USART1_RxBuf[RX1i]);
			RX1i++;
    }
}

/**
  * @brief   IRQ Handler of Recieve USART2 
  * @note   
  * @param  
  * @retval 
  */
void USART2_IRQHandler(void)
{
    if ((USART2->SR & USART_FLAG_RXNE) != (u16)RESET)
    {
            USART2_RxBuf[RX2i] = USART_ReceiveData(USART2);

            if (USART2_RxBuf[RX2i] != 13) 
			{
                if (RX2i > RX_BUF_SIZE) Clear_Buffer(USART2_RxBuf); 
            }
            else RX2_FLAG_END_LINE = true;	
            //Indication - LED OFF
			GPIOC->BSRR = GPIO_Pin_13;	
			RX2i++;
    }		
}

/**
  * @brief   Fills MainMenu structure with necessary values and functions
  * @note   
  * @param  
  * @retval 
  */
void MainMenu_init(void)
{
MainMenu.terminalSend = USART1_SEND;
MainMenu.hc05Send = USART2_SEND;
MainMenu.usart2Struct = &USART2_InitStructure;
}
