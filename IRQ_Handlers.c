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

