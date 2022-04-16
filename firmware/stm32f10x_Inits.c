/*==============================================================================
>File name:  	stm32f10x.c
>Brief:         source for Init periph functions of projectONE		
>Author:        LL
>Date:          07.05.2018
>Version:       1.0
==============================================================================*/

#include "stm32f10x_Inits.h"

USART_InitTypeDef USART2_InitStructure;

#ifdef DMA_USART2_EN
static char DMA_USART2_buffer[DMA_BUF_SIZE];
#endif

#ifdef DMA_USART1_EN
static char DMA_USART1_buffer[DMA_BUF_SIZE];
#endif


/*=================================================================================
------------------- INIT STM32 FUNCTIONS of MotionSensor project ------------------
=================================================================================*/


/**
  * @brief  Set clock freq of core to HSE 8 MHz (no PLL used)
  * @param  None
  * @retval None
  */
void SetSysClockToHSE(void)
{
    ErrorStatus HSEStartUpStatus;
 
    /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration -----------------------------*/
    /* RCC system reset(for debug purpose) */
    RCC_DeInit();
 
    /* Enable HSE */
    RCC_HSEConfig(RCC_HSE_ON);
 
    /* Wait till HSE is ready */
    HSEStartUpStatus = RCC_WaitForHSEStartUp();
 
    if (HSEStartUpStatus == SUCCESS)
    {
        /* Enable Prefetch Buffer */
        //FLASH_PrefetchBufferCmd( FLASH_PrefetchBuffer_Enable);
 
        /* Flash 0 wait state */
        //FLASH_SetLatency( FLASH_Latency_0);
 
        /* HCLK = SYSCLK */
        RCC_HCLKConfig( RCC_SYSCLK_Div1);
 
        /* PCLK2 = HCLK */
        RCC_PCLK2Config( RCC_HCLK_Div1);
 
        /* PCLK1 = HCLK */
        RCC_PCLK1Config(RCC_HCLK_Div1);
 
        /* Select HSE as system clock source */
        RCC_SYSCLKConfig(RCC_SYSCLKSource_HSE);
 
        /* Wait till HSE is used as system clock source */
        while (RCC_GetSYSCLKSource() != 0x04)
        {
        }
    }
    else
    { /* If HSE fails to start-up, the application will have wrong clock configuration.
     User can add here some code to deal with this error */
 
        /* Go to infinite loop */
        while (1)
        {
        }
    }
}

/**
  * @brief  Set clock freq of core to 72 MHz (*9 PLL)
  * @param  None
  * @retval None
  */
void SetSysClockTo_72(void)
{
    ErrorStatus HSEStartUpStatus;
    /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration */
    /* RCC system reset(for debug purpose) */
    RCC_DeInit();
 
    /* Enable HSE */
    RCC_HSEConfig( RCC_HSE_ON);
 
    /* Wait till HSE is ready */
    HSEStartUpStatus = RCC_WaitForHSEStartUp();
 
    if (HSEStartUpStatus == SUCCESS)
    {
        /* Enable Prefetch Buffer */
        //FLASH_PrefetchBufferCmd( FLASH_PrefetchBuffer_Enable);
 
        /* Flash 2 wait state */
        //FLASH_SetLatency( FLASH_Latency_2);
 
        /* HCLK = SYSCLK */
        RCC_HCLKConfig( RCC_SYSCLK_Div1);
 
        /* PCLK2 = HCLK */
        RCC_PCLK2Config( RCC_HCLK_Div1);
 
        /* PCLK1 = HCLK/2 */
        RCC_PCLK1Config( RCC_HCLK_Div2);
 
        /* PLLCLK = 8MHz * 9 = 72 MHz */
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
 
        /* Enable PLL */
        RCC_PLLCmd( ENABLE);
 
        /* Wait till PLL is ready */
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
        {
        }
 
        /* Select PLL as system clock source */
        RCC_SYSCLKConfig( RCC_SYSCLKSource_PLLCLK);
 
        /* Wait till PLL is used as system clock source */
        while (RCC_GetSYSCLKSource() != 0x08)
        {
        }
    }
    else
    { /* If HSE fails to start-up, the application will have wrong clock configuration.
     User can add here some code to deal with this error */
 
        /* Go to infinite loop */
        while (1)
        {
        }
    }
}

/**
  * @brief  Initialize STM32's GPIOs 
  * @param  None
  * @retval None
  */
void GPIO_init(void)
{
	// Configure the GPIOs 
    GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	
	/* PC13 LED pin */ 
	GPIO_InitStructure.GPIO_Pin = LED_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
    GPIO_Init(LED_PORT, &GPIO_InitStructure);
	GPIO_SetBits(LED_PORT,LED_PIN);		//turn led OFF
	
	/* PA0 DEBUG pin */
    
	GPIO_InitStructure.GPIO_Pin = DEBUG_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
    GPIO_Init(DEBUG_PORT, &GPIO_InitStructure);
	GPIO_ResetBits(DEBUG_PORT,DEBUG_PIN);
		
	/* PB1 BLE STATE input pin */
	GPIO_InitStructure.GPIO_Pin = BLE_STATE_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
    GPIO_Init(BLE_STATE_PORT, &GPIO_InitStructure);
	GPIO_ResetBits(BLE_STATE_PORT,BLE_STATE_PIN);
	/* Add IRQ vector to NVIC */
	/* PB1 is connected to EXTI_Line1, which has EXTI1_IRQn vector */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	/* Priority group 2: 0-3 PreempPriority levels, 0-3 SubPriority lvls */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    /* Set priority */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	/* Set sub priority */
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	/* Enable interrupt */
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	/* Add to NVIC */
	NVIC_Init(&NVIC_InitStructure);

	/* Tell system that you will use PB1 for EXTI_Line1 */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);

	/* PB1 is connected to EXTI_Line1 */
	EXTI_InitStructure.EXTI_Line = EXTI_Line1;
	/* Enable interrupt */
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	/* Interrupt mode */
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	/* Triggers on rising and falling edge */
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	/* Add to EXTI */
	EXTI_Init(&EXTI_InitStructure);
	
	
	/* PB0 (EXTI line 0) to recieve interrups DRDY from BMI160 */ 
    GPIO_InitStructure.GPIO_Pin = INT1_EXTI_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	//PB.0 EXTI_LINE0
    GPIO_Init(INT1_EXTI_PORT, &GPIO_InitStructure);
	
	/* Add IRQ vector to NVIC */
	/* PB0 is connected to EXTI_Line0, which has EXTI0_IRQn vector */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	/* Set priority */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   //can be interrupted by 0 priority
	/* Set sub priority */
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	/* Enable interrupt */
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	/* Add to NVIC */
	NVIC_Init(&NVIC_InitStructure);

	/* Tell system that you will use PB0 for EXTI_Line0 */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);

	/* PB0 is connected to EXTI_Line0 */
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	/* Enable interrupt */
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	/* Interrupt mode */
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	/* Triggers on rising and falling edge */
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	/* Add to EXTI */
	EXTI_Init(&EXTI_InitStructure);
}


/**
  * @brief  Initialize STM32's USART1 peripheral module
  * @param  None
  * @retval None
  */
#ifdef DMA_USART1_EN
/**
  * @brief   
  * @note   
  * @param  
  * @retval 
  */
void USART1_init(void)
{
    /* Enable USART1 and GPIOA clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
 
    /* DMA */
    DMA_InitTypeDef DMA_InitStruct;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
    DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)&DMA_USART1_buffer[0];
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralDST; //usart - destination
    DMA_InitStruct.DMA_BufferSize = sizeof(DMA_USART1_buffer);
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;   // don't increment address of USART DaraRegister (DR)
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;            //increment adress of buffer units
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStruct.DMA_Priority = DMA_Priority_High;
    DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel4, &DMA_InitStruct);   //USART1_TX - 4th DMA channel
 
    /* NVIC Configuration */
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    /* Enable the USARTx Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
 
    /* Configure the GPIOs */
    GPIO_InitTypeDef GPIO_InitStructure;
 
    /* Configure USART1 Tx (PA.09) as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
 
    /* Configure USART1 Rx (PA.10) as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
 
    /* Configure the USART1 */
    USART_InitTypeDef USART_InitStructure; 
 
    /* USART1 configuration ------------------------------------------------------*/
    /* USART1 configured as follow:
        - BaudRate = 115200 baud
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
        - USART Clock disabled
        - USART CPOL: Clock is active low
        - USART CPHA: Data is captured on the middle
        - USART LastBit: The clock pulse of the last data bit is not output to
            the SCLK pin
     */
    USART_InitStructure.USART_BaudRate = 921600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
 
    USART_Init(USART1, &USART_InitStructure);
 
    /* Enable USART1 */
    USART_Cmd(USART1, ENABLE);
 
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
    DMA_Cmd(DMA1_Channel4, ENABLE);

    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
    NVIC_EnableIRQ(DMA1_Channel4_IRQn);
 
 
    /* Enable the USART1 Receive interrupt: this interrupt is generated when the
    USART1 receive data register is not empty */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);    
}

/**
  * @brief   Send databuffer via DMA channel 4 ( USART1_TX )
  * @note   
  * @param  
  * @retval 
  */
void USART1_SEND_DMA(char *pbuf)	
{
    /* wait when DR bacomes empty (whole buffer will be transmitted) */
    while ((USART1->SR & USART_FLAG_TC) == (u16)RESET);
    
    strcpy(DMA_USART1_buffer,pbuf);   
    /* Restart DMA Channel*/
    DMA_Cmd(DMA1_Channel4, DISABLE);
    DMA1_Channel4->CNDTR = strlen(pbuf);
    DMA_Cmd(DMA1_Channel4, ENABLE);    
    
}


/* USART1_init function realisation without using DMA channel */
#else   
void USART1_init(void)
{
    // Enable USART1 and GPIOA clock 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
 
    // NVIC Configuration
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    // Enable the USARTx Interrupt 
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
	/**USART2 GPIO Configuration  
	PA9   ------> USART1_TX
	PA10  ------> USART1_RX 
	*/
    // Configure the GPIOs 
    GPIO_InitTypeDef GPIO_InitStructure;
 
    // Configure USART1 Tx (PA.09) as alternate function push-pull 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
 
    // Configure USART1 Rx (PA.10) as input floating 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
 
    // Configure the USART1 
    USART_InitTypeDef USART_InitStructure;
 
    // USART1 configuration --------------------------------------------------
    // USART1 configured as follow:
    //   - BaudRate = 115200 baud 1382400 // 921600 // 2000000
    //   - Word Length = 8 Bits
    //   - One Stop Bit
    //   - No parity
    //   - Hardware flow control disabled (RTS and CTS signals)
    //   - Receive and transmit enabled
    //   - USART Clock disabled
    //   - USART CPOL: Clock is active low
    //   - USART CPHA: Data is captured on the middle
    //   - USART LastBit: The clock pulse of the last data bit is not output to
    //     the SCLK pin
     
    USART_InitStructure.USART_BaudRate = 921600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
 
    USART_Init(USART1, &USART_InitStructure);
 
    // Enable USART1 
    USART_Cmd(USART1, ENABLE);
 
    // Enable the USART1 Receive interrupt: this interrupt is generated when the
    // USART1 receive data register is not empty 
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}


#endif  //USART1 init


/**
  * @brief  sending buffer via USART1 
  * @param  *pbuf - pointer at buf
  * @retval None
  */
uint8_t USART1_SEND(volatile char *pbuf)
{
    while (*pbuf)
    {
        USART_SendData(USART1, *pbuf++);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);	//wait whole transmission
    }
return SUCCESS;
}   


#ifdef DMA_USART2_EN
/**
  * @brief   
  * @note   
  * @param  
  * @retval 
  */
void USART2_init(void)
{
    /* Enable USART1 and GPIOA clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
 
    /* DMA */
    DMA_InitTypeDef DMA_InitStruct;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
    DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)&DMA_USART2_buffer[0];
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStruct.DMA_BufferSize = sizeof(DMA_USART2_buffer);
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStruct.DMA_Priority = DMA_Priority_Low;
    DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel7, &DMA_InitStruct);
 
    /* NVIC Configuration */
    NVIC_InitTypeDef NVIC_InitStructure;
    /* Enable the USARTx Interrupt */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
 
    /* Configure the GPIOs */
    GPIO_InitTypeDef GPIO_InitStructure;
 
    // Configure USART2 Tx (PA.02) as alternate function push-pull 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
 
    // Configure USART2 Rx (PA.03) as input floating 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
 
    /* Configure the USART2 */ 
 
    /* USART2 configuration ------------------------------------------------------*/
    /* USART2 configured as follow:
        - BaudRate = 115200 baud 921600 \\ 1382400 \\ 38400 - proging speed
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
        - USART Clock disabled
        - USART CPOL: Clock is active low
        - USART CPHA: Data is captured on the middle
        - USART LastBit: The clock pulse of the last data bit is not output to
            the SCLK pin
     */
    USART2_InitStructure.USART_BaudRate = 921600; 
    USART2_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART2_InitStructure.USART_StopBits = USART_StopBits_1;
    USART2_InitStructure.USART_Parity = USART_Parity_No;
    USART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART2_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
 
    USART_Init(USART2, &USART2_InitStructure);
 
    /* Enable USART2 */
    USART_Cmd(USART2, ENABLE);
 
    USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
    DMA_Cmd(DMA1_Channel7, ENABLE);
 
    //DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE);
    //NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  
    /* Enable the USART1 Receive interrupt: this interrupt is generated when the
    USART2 receive data register is not empty */
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}

/**
  * @brief   
  * @note   
  * @param  
  * @retval 
  */
uint8_t USART2_SEND_DMA(volatile char *pbuf)	
{
    /* wait when DR bacomes empty (whole buffer will be transmitted) */
    while ((USART2->SR & USART_FLAG_TC) == (u16)RESET);
    
    strcpy(DMA_USART2_buffer, (char*)pbuf);
    /* Restart DMA Channel*/
    DMA_Cmd(DMA1_Channel7, DISABLE);
    DMA1_Channel7->CNDTR = strlen((char*)pbuf);
    DMA_Cmd(DMA1_Channel7, ENABLE);
return SUCCESS;
}

#else

/**
  * @brief   	USART2 initializing
  * @note    	USART2_InitStructure must be glodal cuz it is used in HC-05.c to show status	
  * @param  
  * @retval 	SUCCESS
  */
void USART2_init(void)
{
	// Enable USART2 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	
    // NVIC Configuration
    NVIC_InitTypeDef NVIC_InitStructure;
    // Enable the USART2 Interrupt 
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    // Enable the USART1 Receive interrupt: this interrupt is generated when the
    // USART2 receive data register is not empty 
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	
	/**USART2 GPIO Configuration  
	PA2   ------> USART2_TX
	PA3   ------> USART2_RX 
	*/
    // Configure the GPIOs 
    GPIO_InitTypeDef GPIO_InitStructure;
 
    // Configure USART1 Tx (PA.02) as alternate function push-pull 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
 
    // Configure USART2 Rx (PA.03) as input floating 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
 
    // Configure the USART2  
    USART2_InitStructure.USART_BaudRate = 1382400;//921600;
    USART2_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART2_InitStructure.USART_StopBits = USART_StopBits_1;
    USART2_InitStructure.USART_Parity = USART_Parity_No;
    USART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART2_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART2_InitStructure);
 
    // Enable USART2 
    USART_Cmd(USART2, ENABLE);	
}
#endif //USART2 init

/**
  * @brief   
  * @note   
  * @param  
  * @retval 
  */
uint8_t USART2_SEND(volatile char *pbuf)
{
    while (*pbuf)
    {
        USART_SendData(USART2, *pbuf++);
        while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);	//wait whole transmission
    }
return SUCCESS;
}

/**
  * @brief  sending data via USART1 fast speed
  * @param  *pbuf - pointer at buf
  * @retval None
  */
__forceinline void USART_SENDoptimized (USART_TypeDef* USARTx, const char *pbuf)
{
    
    while (*pbuf)
    {
        USARTx->DR = (*pbuf++ & (uint16_t)0x01FF);
        while((USARTx->SR & USART_FLAG_TC) == RESET);	//wait whole transmission
    }
  
}

/**
  * @brief  initialize SPI2 at MASTER mode
  * @param  None
  * @retval None
  */
void SPI2_MASTER_init (void)
{
	SPI_I2S_DeInit(SPI2);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	//GPIO_DeInit(SPI_GPIO_PORT);
	
    GPIO_InitTypeDef PortSettings;
	
	/* MOSI and SCK */
    PortSettings.GPIO_Speed = GPIO_Speed_50MHz;		
    PortSettings.GPIO_Pin = SPI_MOSI_PIN | SPI_SCK_PIN;
    PortSettings.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(SPI_GPIO_PORT,&PortSettings);

	/* MISO */
	PortSettings.GPIO_Pin = SPI_MISO_PIN;				
	PortSettings.GPIO_Mode = GPIO_Mode_IN_FLOATING;			
	PortSettings.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI_GPIO_PORT, &PortSettings);	
	
	/* NSS - CSB */
    PortSettings.GPIO_Speed = GPIO_Speed_50MHz;		
    PortSettings.GPIO_Pin =  BMI160_CSB_PIN;
    PortSettings.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(SPI_GPIO_PORT,&PortSettings);
	GPIO_SetBits(SPI_GPIO_PORT,BMI160_CSB_PIN);
	/* PA1 CSB for BMM150 SPI intrf */
	PortSettings.GPIO_Pin = GPIO_Pin_1;
    PortSettings.GPIO_Mode = GPIO_Mode_Out_PP;
    PortSettings.GPIO_Speed = GPIO_Speed_50MHz;	
    GPIO_Init(GPIOA, &PortSettings);
	GPIO_SetBits(GPIOA,GPIO_Pin_1);
	
	SPI_InitTypeDef SPI;
	
	SPI_StructInit(&SPI);	//Fills each SPI_InitStruct member with its default value	
	
    SPI.SPI_Mode = SPI_Mode_Master;
    SPI.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; //(APB1 = 36 MHz) SCLK = 9 MHz
    SPI.SPI_Direction = SPI_Direction_2Lines_FullDuplex ;
    SPI.SPI_CPOL = SPI_CPOL_High;					//CPOL_High (1) - ????\_/?\_/?\_/?TTT , CPOL_Low (0) - ___/?\__/?\__/?\___
    SPI.SPI_CPHA = SPI_CPHA_2Edge;				//CPHA_2Edge(1) - ?????? ?????, 			 CPHA_1Edge(0) - ???????? ?????
    SPI.SPI_DataSize = SPI_DataSize_8b;
    SPI.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI.SPI_NSS = SPI_NSS_Soft;
    SPI_Init(SPI2,&SPI);

    //SPI_NSSInternalSoftwareConfig(SPI2,SPI_NSSInternalSoft_Reset);
    SPI_Cmd(SPI2,ENABLE);
}

/**
  * @brief  send data via SPI2 (common method) 
  * @param  data - ???? ??? 2 ????? (16???) ?????? ??? ????????
  * @retval None
  */
void SPI_Send(uint16_t data) 
{
	SPI_I2S_SendData(SPI2, data);  // ???????? ???? ?????? ? ??????? ?? ????????
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET); // ????, ???????????? SPI_DR
}






