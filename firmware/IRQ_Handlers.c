/*==============================================================================
>File name:  	IRQ_HANDLERS.c
>Brief:         Handlers of interrups here. watch NVIC table in misc.h	
								and startup_stm32f10x_md.s for handlers
>Author:        LL
>Date:          08.05.2018
>Version:       1.0
==============================================================================*/

#include "IRQ_Handlers.h"

volatile bool RX1_FLAG_END_LINE = false;
volatile bool RX2_FLAG_END_LINE = false;
volatile bool MADGWICK_FILTER_FLAG = false;
volatile bool MAHONY_FILTER_FLAG = false;
volatile bool BLE_CONNECTION_FLAG = false;

extern struct MainMenu_Struct MainMenu;
extern struct bmi160_dev bmi160;
extern struct bmi160_sensor_data accel;
extern struct bmi160_sensor_data gyro;
extern struct bmm150_dev bmm150;
extern struct bmm150_raw_mag_data raw;
//extern volatile bool SENSOR_FLAG_DATA_RECIEVED;
//extern volatile bool DISPLAY_SENSOR_DATA_FLAG;
extern filtered_data fdata;

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
            DEBUG_PORT->BSRR = DEBUG_PIN; 		//**DEBUG** set bit (High)
		
        if (!BLE_CONNECTION_FLAG)               //No Bluetooth connection
        {    
            if (MADGWICK_FILTER_FLAG)
            {
            BMI160_OptGet_FilteredData(&fdata);
            sprintf((char*)USART1_TxBuf,"\n\rroll %f, pitch %f, yaw %f", fdata.roll, fdata.pitch, fdata.heading);
            #ifdef DMA_USART1_EN
            USART1_SEND_DMA((char*)USART1_TxBuf);           
            #else
            USART_SENDoptimized(USART1,(char*)USART1_TxBuf);
            #endif
            }
            else if (MAHONY_FILTER_FLAG)
            {
            }
            else Read_Display_SensorData(USART1, &bmm150);      //Raw mode
        }
        else
        {
            if (MADGWICK_FILTER_FLAG)
            {
            BMI160_OptGet_FilteredData(&fdata);
            sprintf((char*)USART2_TxBuf,"\n\rroll %f, pitch %f, yaw %f", fdata.roll, fdata.pitch, fdata.heading);
            #ifdef DMA_USART2_EN
            USART2_SEND_DMA((char*)USART2_TxBuf);           
            #else
            USART_SENDoptimized(USART2,(char*)USART2_TxBuf);
            #endif
            }
            else if (MAHONY_FILTER_FLAG)
            {
            }
            else Read_Display_SensorData(USART2, &bmm150);      //Raw mode
        }  
				//SENSOR_FLAG_DATA_RECIEVED = 1;
		/* Clear interrupt flag */
		EXTI->PR = EXTI_Line0;
		
            DEBUG_PORT->BRR = DEBUG_PIN;		//**DEBUG**	reset bit (Low)
	}
}

/**
  * @brief  IRQ Handler of waking up from BLE connection astablished
  * @param  None
  * @retval None
  */
void EXTI1_IRQHandler(void) 
{
    /* Make sure that interrupt flag is set */
    if (EXTI_GetITStatus(EXTI_Line1) != RESET) 
	{
		/* RISING */
        if (GPIO_ReadInputDataBit(BLE_STATE_PORT, BLE_STATE_PIN))
		{		
			LED_PORT->BRR = LED_PIN;
            /* To get measurement values of accel+gyro set normal mode both 
            NOTE: when Normal mode activated INT1 BMI160 DRDY interrupts are generated. 
            When lowpower/suspend mode is active - no interrups generated */
			//BMI160_set_Mode(&bmi160,BMI160_ACCEL_NORMAL_MODE,BMI160_GYRO_NORMAL_MODE); 

            BLE_CONNECTION_FLAG = true;
            MainMenu.terminalSend("\n\r>Bluetooth connection established\r\n");
            MainMenu.terminalSend = USART2_SEND;
            MainMenu.terminalSend("\n\r>Bluetooth connection established\r\n");    
		}
			
		/* FALLING */
        if (GPIO_ReadInputDataBit(BLE_STATE_PORT, BLE_STATE_PIN) == 0) 
		{
			LED_PORT->BSRR = LED_PIN;
            /* to save energy set sensors back to suspend/sleep mode */
			//BMI160_set_Mode(&bmi160,BMI160_ACCEL_SUSPEND_MODE,BMI160_GYRO_SUSPEND_MODE);
            
            BLE_CONNECTION_FLAG = false;
            MainMenu.terminalSend("\n\r>Bluetooth connection lost\r\n");
            MainMenu.terminalSend = USART1_SEND;
            MainMenu.terminalSend("\n\r>Bluetooth connection lost\r\n");
		}
        /* Clear interrupt flag */
        EXTI_ClearITPendingBit(EXTI_Line1);
    }		
}

/**
  * @brief  IRQ Handler of Recieve USART1 from PC
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)
{    
    if ((USART1->SR & USART_FLAG_RXNE) != (u16)RESET)
    {         
            USART1_RxBuf[RX1i] = USART_ReceiveData(USART1);

                if (USART1_RxBuf[RX1i++] != 13) 
                {
                    if (RX1i > RX_BUF_SIZE) Clear_Buffer(USART1_RxBuf); 
                }
                else 
                {
                    RX1_FLAG_END_LINE = true;
                    MainMenu_checkInput(&MainMenu);
                }         
            //Echo Чтобы видеть в терминале что вводится
            //USART_SendData(USART1, USART1_RxBuf[RX1i]);	         
    }
}

/**
  * @brief   IRQ Handler of Recieve USART2 from BLUETOOTH module
  * @note   
  * @param  
  * @retval 
  */
void USART2_IRQHandler(void)
{
    if ((USART2->SR & USART_FLAG_RXNE) != (u16)RESET)
    {
        if (!BLE_CONNECTION_FLAG)           // No Bluetooth Connection
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
			
		MainMenu_checkAnswer(&MainMenu);
        }
        
        else                                // Bluetooth Connection active       
        {
        USART1_RxBuf[RX1i] = USART_ReceiveData(USART2);

            if (USART1_RxBuf[RX1i++] != 13) 
            {
                if (RX1i > RX_BUF_SIZE) Clear_Buffer(USART1_RxBuf); 
            }
            else 
            {
                RX1_FLAG_END_LINE = true;               
                MainMenu_checkInput(&MainMenu);
            }    
        }
    }		
}

/**
  * @brief   DMA USART1_TX channel IRQ handler
  * @note   
  * @param  
  * @retval 
*/  
void DMA1_Channel4_IRQHandler(void)
{
    DMA_ClearITPendingBit(DMA1_IT_TC4);
    DMA_Cmd(DMA1_Channel4, DISABLE);
}


/**
  * @brief   USART2_TX DMA IRQ Handler
  * @note   
  * @param  
  * @retval 
  
void DMA1_Channel7_IRQHandler(void)
{
    DMA_ClearITPendingBit(DMA1_IT_TC7);
    DMA_Cmd(DMA1_Channel7, DISABLE);
}
*/
