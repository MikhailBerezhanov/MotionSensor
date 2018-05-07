/*==============================================================================
>File name:  	BMI160_func.c
>Brief:         Source code for init and work with BMI160,BMM150 in projectONE	
>Author:        LL
>Date:          07.05.2018
>Version:       1.0
==============================================================================*/

#include "BMI160_func.h"
#include "stm32f10x_Inits.h"
#include "Menu.h"

//---------------------------= 101[V]ariables =--------------------------------
struct bmi160_dev bmi160;
struct bmi160_int_settg int_config;
struct bmi160_pmu_status pmu_status;	
struct bmi160_sensor_data accel;
struct bmi160_sensor_data gyro;	
struct bmi160_common_status sensor_status;
struct bmm150_dev bmm150;
int8_t rslt = BMI160_OK;
filtered_data fdata;

unsigned short x = 0x0001;
extern char USART1_TxBuf[TX_BUF_SIZE];
int8_t rsltBMI160secIntrf;
int8_t rsltBMM150init;

/**
  * @brief  Initialize BMI160 to connect to MCU via SPI (4-wire)
  * @param  None
  * @retval SUCCESS = BMI160_OK, FAILURE = 0
  */
int8_t BMI160_init (void)
{
/* You may assign a chip select identifier to be handled later */
	bmi160.id = 0;
	bmi160.interface = BMI160_SPI_INTF;
	bmi160.read = SPI2_Read_BMI160;
	bmi160.write = SPI2_Write_BMI160;
	bmi160.delay_ms = _delay_ms;

	/* Configure device structure for auxiliary sensor parameter */
	bmi160.aux_cfg.aux_sensor_enable = 1; 						// auxiliary sensor enable
	bmi160.aux_cfg.aux_i2c_addr = BMI160_AUX_BMM150_I2C_ADDR; 	// auxiliary sensor address
	bmi160.aux_cfg.manual_enable = 1; 							// setup mode enable
	bmi160.aux_cfg.aux_rd_burst_len = 2;						// burst read of 2 byte
	
	if (bmi160_init(&bmi160) == BMI160_OK)
/* After the above function call, accel_cfg and gyro_cfg parameters in the device 
structure are set with default values, found in the datasheet of the bmi160 */
	{	
	/* Fill configurations for sensors gyro and accel */	
	bmi160.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;			//Powermode
	bmi160.accel_cfg.range = BMI160_ACCEL_RANGE_2G;				//Range settings
	bmi160.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;			//Bandwidth
	bmi160.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;				//Output data rate	
	
	bmi160.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;	
	bmi160.gyro_cfg.range =	BMI160_GYRO_RANGE_250_DPS;
	bmi160.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;	
	bmi160.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;	
	
		/* Configure the BMM150 device structure by 
		mapping user_aux_read and user_aux_write */
		bmm150.read = user_aux_read;
		bmm150.write = user_aux_write;
		bmm150.dev_id = BMM150_DEFAULT_I2C_ADDRESS; 
		/* Ensure that sensor.aux_cfg.aux_i2c_addr = bmm150.id
		for proper sensor operation */
		bmm150.delay_ms = _delay_ms;
		bmm150.intf = BMM150_I2C_INTF;	
			
		/* Initialize the secondary interface in BMI160 and
        Initialize the auxiliary sensor interface */
		rsltBMI160secIntrf = bmi160_aux_init(&bmi160);
        /* Auxiliary sensor is enabled and can be accessed from this point */
        
        /* Configure the desired settings in auxiliary BMM150 sensor 
        * using the bmm150 APIs */

        /* Initialising the bmm150 sensor */
        rsltBMM150init = bmm150_init(&bmm150);

        /* Set the power mode and preset mode to enable Mag data sampling */
        bmm150.settings.pwr_mode = BMM150_NORMAL_MODE;
        rslt = bmm150_set_op_mode(&bmm150);

		bmm150.settings.preset_mode= BMM150_PRESETMODE_LOWPOWER;
		rslt = bmm150_set_presetmode(&bmm150);	
        
        if (bmi160_set_sens_conf(&bmi160) == BMI160_OK) 			//set above configurations of sensors
			if (BMI160_INT_config() == BMI160_OK) return BMI160_OK;	//Initialize INT1 dataready interrupt
           
	}
return 0;
}

/**
  * @brief   wrapper function to match the signature of bmm150.read
  * @param   None
  * @retval  SUCCESS = BMI160_OK
  */
int8_t user_aux_read(uint8_t id, uint8_t reg_addr, uint8_t *aux_data, uint16_t len)
{
	int8_t rslt;
	
	/* Discarding the parameter id as it is redundant*/
    rslt = bmi160_aux_read(reg_addr, aux_data, len, &bmi160);

	return rslt;
}

/**
  * @brief   wrapper function to match the signature of bmm150.write
  * @param   None
  * @retval  SUCCESS = BMI160_OK
  */
int8_t user_aux_write(uint8_t id, uint8_t reg_addr, uint8_t *aux_data, uint16_t len)
{
	int8_t rslt;
	
	/* Discarding the parameter id as it is redundant */
	rslt = bmi160_aux_write(reg_addr, aux_data, len, &bmi160);

	return rslt;
}

/**
  * @brief   Config INT1 BMI160 pin to generate DataReady interrupt
  * @param   None
  * @retval  SUCCESS = BMI160_OK, FAILURE = 0
  */
int8_t BMI160_INT_config (void)
{
/* Select the Interrupt channel/pin */
int_config.int_channel = BMI160_INT_CHANNEL_1;							// Interrupt channel/pin 1

/* Select the Interrupt type */
int_config.int_type = BMI160_ACC_GYRO_DATA_RDY_INT;						// Choosing interrupt
/* Select the interrupt channel/pin settings */
int_config.int_pin_settg.output_en = BMI160_ENABLE;						// Enabling interrupt pins to act as output pin
int_config.int_pin_settg.output_mode = BMI160_DISABLE;					// Choosing push-pull mode for interrupt pin
int_config.int_pin_settg.output_type = BMI160_DISABLE;					// Choosing active low output
int_config.int_pin_settg.edge_ctrl = BMI160_ENABLE;						// Choosing edge triggered output
int_config.int_pin_settg.input_en = BMI160_DISABLE;						// Disabling interrupt pin to act as input
int_config.int_pin_settg.latch_dur = BMI160_LATCH_DUR_312_5_MICRO_SEC;	// non-latched output

/* Set the data ready interrupt */ 
if (bmi160_set_int_config(&int_config, &bmi160) == BMI160_OK)	return BMI160_OK;
else return 0;
}


/**
  * @brief   Displays via UASRT the results of init and internal registers status BMI160
  * @param   None
  * @retval  SUCCESS = BMI160_OK
  */
int8_t BMI160_display_status (void)
{
	/* DATE TIME and C VERSION */
	sprintf(USART1_TxBuf, "\r\n\tDate: %s", __DATE__ );			
    USART1_SEND(USART1_TxBuf);			
	sprintf(USART1_TxBuf, "\tTime: %s", __TIME__ );			
    USART1_SEND(USART1_TxBuf);
	sprintf(USART1_TxBuf, "\t\tC Version: %ld", __STDC_VERSION__ );			
    USART1_SEND(USART1_TxBuf);	
		
		//test_byteorder of machine (stm32f103)
			sprintf(USART1_TxBuf, "\n\r\tSize of x: %d", sizeof(x));
			USART1_SEND(USART1_TxBuf);
			sprintf(USART1_TxBuf, "\n\r\tByte order: %s", *((unsigned char*) &x) == 0? "Big endian\n\r" : "Little endian\n\r");
			USART1_SEND(USART1_TxBuf);
		//-------------------------------------
	
	/* Getting bmi160 ID */ 
	SPI2_Read_BMI160(1,(BMI160_CHIP_ID_ADDR | BMI160_SPI_RD_MASK),&sensor_status.id,1);
	//bmi160_get_regs(BMI160_CHIP_ID_ADDR,&id,1,&bmi160);
	sprintf(USART1_TxBuf, "\r\n\tSensor ID: %#X\r\n", sensor_status.id );			
    USART1_SEND(USART1_TxBuf);																		
		
	/* GYRO self test */				
	if ( bmi160_perform_self_test(BMI160_GYRO_ONLY, &bmi160) == BMI160_W_GYRO_SELF_TEST_FAIL) 	
	USART1_SEND(" \r\n\tGYRO SELF TEST: F A I L.\r\n");
	else USART1_SEND(" \r\n\tGYRO SELF TEST: OK!!\r\n");
	//ends with soft reset so Initialization again needed
	BMI160_init();

	/* ACCEl self test */	
	if ( bmi160_perform_self_test(BMI160_ACCEL_ONLY, &bmi160) == BMI160_W_ACCEl_SELF_TEST_FAIL) 	
	USART1_SEND(" \tACCELSELF TEST: F A I L.\r\n");
	else USART1_SEND(" \r\n\tACCEL SELF TEST: OK!!\r\n");
	//ends with soft reset so Initialization again needed			
	BMI160_init();

	/* Error status check */
	bmi160_get_regs(BMI160_ERROR_REG_ADDR, &sensor_status.err_status, 1, &bmi160);				
	sprintf(USART1_TxBuf, "\r\n\tError status: %#X\r\n", sensor_status.err_status );		
    USART1_SEND(USART1_TxBuf);			

	/* Temperature1 status check */
	bmi160_get_regs(0x20, &sensor_status.temperature1, 1, &bmi160);				
	sprintf(USART1_TxBuf, "\r\n\tTemperature: %#X\r\n", sensor_status.temperature1 );		
    USART1_SEND(USART1_TxBuf);
	
	/* Temperature2 status check */
	bmi160_get_regs(0x20, &sensor_status.temperature2, 1, &bmi160);				
	sprintf(USART1_TxBuf, "\tTemperature: %#X\r\n", sensor_status.temperature2 );		
    USART1_SEND(USART1_TxBuf);			

	/* Power mode status check */
	bmi160_get_regs(BMI160_PMU_STATUS_ADDR, &sensor_status.pwr_status, 1, &bmi160);				
	sprintf(USART1_TxBuf, "\r\n\tPWR status: %#X\r\n", sensor_status.pwr_status );		
    USART1_SEND(USART1_TxBuf);			

	/* bmi160 status check */				
	bmi160_get_regs(BMI160_STATUS_ADDR,&sensor_status.comm_status,1,&bmi160);
	sprintf(USART1_TxBuf, "\r\n\tStatus: %#X\r\n", sensor_status.comm_status );		
    USART1_SEND(USART1_TxBuf);			

	/* GYRO and ACCEL power mode check */		
	bmi160_get_power_mode(&pmu_status,&bmi160);
	sprintf(USART1_TxBuf, "\r\n\tGyro PWR mode: %#X\r\n", pmu_status.gyro_pmu_status );		
    USART1_SEND(USART1_TxBuf);			
	sprintf(USART1_TxBuf, "\tAccel PWR mode: %#X\r\n", pmu_status.accel_pmu_status );		
    USART1_SEND(USART1_TxBuf);			

	/* Getting data values from sensors */
	bmi160_get_sensor_data(BMI160_BOTH_ACCEL_AND_GYRO,&accel,&gyro,&bmi160);
	sprintf(USART1_TxBuf, "\r\n\tX_Gyro value: %d\r\n", gyro.x );			
    USART1_SEND(USART1_TxBuf);				
	sprintf(USART1_TxBuf, "\tY_Gyro value: %d\r\n", gyro.y );			
    USART1_SEND(USART1_TxBuf);			
	sprintf(USART1_TxBuf, "\tZ_Gyro value: %d\r\n", gyro.z );			
    USART1_SEND(USART1_TxBuf);			
	sprintf(USART1_TxBuf, "\r\n\tX_Accel value: %d\r\n", accel.x );			
    USART1_SEND(USART1_TxBuf);			
	sprintf(USART1_TxBuf, "\tY_Accel value: %d\r\n", accel.y );			
    USART1_SEND(USART1_TxBuf);			
	sprintf(USART1_TxBuf, "\tZ_Accel value: %d\r\n", accel.z );			
    USART1_SEND(USART1_TxBuf);	

	/* BMI160 secondary interface initialization results */
	if (rsltBMI160secIntrf == BMI160_OK)
	USART1_SEND("\r\n\tBMI160 secondary interface activated SUCCESSFULLY\r\n");
	else USART1_SEND("\r\n\tFALIED to actived BMI160 secondary interface\r\n");
	
	/* BMM150 initialization results */
	if (rsltBMM150init == BMM150_OK)
	USART1_SEND("\r\n\tBMM150 initialized SUCCESSFULLY\r\n");
	else USART1_SEND("\r\n\tFALIED to initialize BMM150\r\n");
	
return (BMI160_OK);
}


/**
  * @brief   Read accel and gyro from BMI160 data and display it via terminal
  * @param   None
  * @retval  Sucsess = BMI160_OK
  */
int8_t BMI160_display_data (void)
{			
			//BMI160_OptGet_data(&accel,&gyro);
			//rslt = bmi160_get_sensor_data(BMI160_BOTH_ACCEL_AND_GYRO,&accel,&gyro,&bmi160);
			
			sprintf(USART1_TxBuf, "\r\n\tGyro value: %d ", gyro.x );			
      USART1_SEND(USART1_TxBuf);			
			sprintf(USART1_TxBuf, "%d ", gyro.y );			
      USART1_SEND(USART1_TxBuf);			
			sprintf(USART1_TxBuf, "%d \r\n", gyro.z );			
      USART1_SEND(USART1_TxBuf);			
			sprintf(USART1_TxBuf, "\r\n\tAccel value: %d ", accel.x );			
      USART1_SEND(USART1_TxBuf);			
			sprintf(USART1_TxBuf, "%d ", accel.y );			
      USART1_SEND(USART1_TxBuf);			
			sprintf(USART1_TxBuf, "%d\r\n", accel.z );			
      USART1_SEND(USART1_TxBuf);			
			
			if (rslt == BMI160_OK) return (BMI160_OK);
			return 0;
}

/**
  * @brief	Reading from BMI160 registers
  *	@note		????????? ?????????? ?????????? ? ?????????????? ?????? ?????? ? ????????, 
						????? ????????????? ?????? ???????????? ???????. ??????? ?????? - ??????????
						? ??????? ???????, ?????? - ?????? ??????????????? ? ????????.
  * @param  id - not used , reg_addr - address of register to read, 
						*data - pointer of data that has been read, len - No. of bytes to read
  * @retval SUCCESS = BMI160_OK
  */
int8_t SPI2_Read_BMI160(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{ 
	/******** ???????????????? ??? *********					********** ?????? **********/	
	
	SPI2->DR;		//??????? ??????? ????? ????, ?? ??? ???? ?? ????????
	//drop CSB pin = start conversation
	GPIOB->BRR = SPI_CSB_PIN;									//GPIO_ResetBits(SPI_GPIO_PORT,SPI_CSB_PIN);	
	
	SPI2->DR = reg_addr;										//SPI_I2S_SendData(SPI2,reg_addr);
	
	while ((SPI2->SR & SPI_I2S_FLAG_TXE) == (uint16_t)RESET){}	//while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	
	/* ????????? ?????????? ? ~2 ????, ?? ?????? ??????-?? ???????? ???????????, ???? ???????????? ?????? while ??? SPI2->DR; ? ??????
		?????? ???????. ???????? ?????? ?? ???????? SPI RX ???????? ????? ??????*/	
	
  while ((SPI2->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);	//while (SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_RXNE) == RESET){}	
	//clear Rx buffer after sending BMI160 reg_adr byte
	SPI2->DR;													//SPI_I2S_ReceiveData(SPI2);
		
	for (register uint16_t i=0; i < len; i++)
	{
	//send CLK for MISO data
	SPI2->DR = 0x00;											//SPI_I2S_SendData(SPI2,0x00);	
	while ((SPI2->SR & SPI_I2S_FLAG_TXE) == (uint16_t)RESET){}	//while (SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE) == RESET){}
 		
	while ((SPI2->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);	//while (SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_RXNE) == RESET){}
		
	//get data from BMI160 reg
	*data = (uint8_t)SPI2->DR;									//*data = (uint8_t)SPI_I2S_ReceiveData(SPI2);
	data++;
	}
	
	//Stop conversation
	GPIOB->BSRR = SPI_CSB_PIN;									//GPIO_SetBits(SPI_GPIO_PORT,SPI_CSB_PIN);
	return BMI160_OK; 
}

/**
  * @brief	?????? ?????? ? ??????? BMI160   
	* @note		???????? ?????????? ? ??????? ? ????????!!
	* @param  id - not used , reg_addr - address of register to write, 
						*data - pointer to data that must be written, len - No. of bytes to write
  * @retval SUCCESS = BMI160_OK
  */
int8_t SPI2_Write_BMI160 (uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	uint8_t *ptmp = data;			//get start address of data
	
	for (uint16_t i = 0; i < len; i++)
	{
	GPIO_ResetBits(SPI_GPIO_PORT,SPI_CSB_PIN);					//drop CSB pin = start conversation
	
	SPI_I2S_SendData(SPI2, reg_addr);  							// ???????? ???? ?????? ? ??????? ?? ????????
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET); // ????, ???????????? SPI_DR
	
	SPI_I2S_SendData(SPI2, *data);  							// ???????? ???? ?????? ? ??????? ?? ????????
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET); // ????, ???????????? SPI_DR
		
	GPIO_SetBits(SPI_GPIO_PORT,SPI_CSB_PIN);		//up CSB pin = stop conversation
	data++;
	}
	data = ptmp;
	return BMI160_OK;
}

/**
  * @brief   !!OPTIMIZED for hardware Read accel and gyro data from BMI160
  * @param   
  * @retval  Success = BMI160_OK
 */ 
int8_t BMI160_OptGet_data (struct bmi160_sensor_data *accel, struct bmi160_sensor_data *gyro)
{
	int8_t rslt;
	uint8_t idx = 0;
	uint8_t data_array[15] = {0};
	uint8_t lsb;
	uint8_t msb;
	int16_t msblsb;

	/* read both accel and gyro bmi160 data
	 * along with time if requested */
	SPI2_Read_BMI160(1, BMI160_GYRO_DATA_ADDR | BMI160_SPI_RD_MASK, data_array, 12);
	//rslt = bmi160_get_regs(BMI160_GYRO_DATA_ADDR, data_array, 12 + len, dev);
	
	if (rslt == BMI160_OK) 
		{
		// Gyro Data 
		lsb = data_array[idx++];
		msb = data_array[idx++];
		msblsb = (int16_t)((msb << 8) | lsb);
		gyro->x = msblsb; // gyro X axis data 

		lsb = data_array[idx++];
		msb = data_array[idx++];
		msblsb = (int16_t)((msb << 8) | lsb);
		gyro->y = msblsb; // gyro Y axis data 

		lsb = data_array[idx++];
		msb = data_array[idx++];
		msblsb = (int16_t)((msb << 8) | lsb);
		gyro->z = msblsb; // gyro Z axis data 

		// Accel Data 
		lsb = data_array[idx++];
		msb = data_array[idx++];
		msblsb = (int16_t)((msb << 8) | lsb);
		accel->x = (int16_t)msblsb; // accel X axis data 

		lsb = data_array[idx++];
		msb = data_array[idx++];
		msblsb = (int16_t)((msb << 8) | lsb);
		accel->y = (int16_t)msblsb; // accel Y axis data 

		lsb = data_array[idx++];
		msb = data_array[idx++];
		msblsb = (int16_t)((msb << 8) | lsb);
		accel->z = (int16_t)msblsb; // accel Z axis data 
		}
	
	return rslt;	
}


/**
  * @brief   Read accel and gyro from BMI160 data and send it to bluetooth module HC-05
  * @param   None
  * @retval  Sucsess = BMI160_OK
  */
int8_t BMI160_sendto_HC05 (void)
{			
	//BMI160_OptGet_data(&accel,&gyro);
	//rslt = bmi160_get_sensor_data(BMI160_BOTH_ACCEL_AND_GYRO,&accel,&gyro,&bmi160);
			
	sprintf(USART1_TxBuf, "\r\n\tGyro value: %d ", gyro.x );			
    USART2_SEND(USART1_TxBuf);			
	sprintf(USART1_TxBuf, "%d ", gyro.y );			
    USART2_SEND(USART1_TxBuf);			
	sprintf(USART1_TxBuf, "%d \r\n", gyro.z );			
    USART2_SEND(USART1_TxBuf);			
	sprintf(USART1_TxBuf, "\r\n\tAccel value: %d ", accel.x );			
    USART2_SEND(USART1_TxBuf);			
	sprintf(USART1_TxBuf, "%d ", accel.y );			
    USART2_SEND(USART1_TxBuf);			
	sprintf(USART1_TxBuf, "%d\r\n", accel.z );			
    USART2_SEND(USART1_TxBuf);			
			
	if (rslt == BMI160_OK) return (BMI160_OK);
return 0;
}

/**
  * @brief   Convert Raw valus from BMI160 for Madgwick filter
  * @param   raw value
  * @retval  cinverted value
 */ 
float convertRawAcceleration(int16_t aRaw) 
{
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int16_t gRaw) 
{
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}


/**
  * @brief   Filter data with one of incluted methodes of filtration: Madgwick \ Mahony
  * @param   *fdata - pointer to struct to fill with filtered values
  *sens1 - pointer to input bmi160's data struct (accel)
  *sens2 - pointer to input bmi160's data struct (gyro)
  * @retval  SUCCESS = SUCCESS 
 */
_Bool filter_data(filtered_data *fdata, struct bmi160_sensor_data *sens1, struct bmi160_sensor_data *sens2)
{
		fdata->ax = convertRawAcceleration(sens1->x);
    fdata->ay = convertRawAcceleration(sens1->y);
    fdata->az = convertRawAcceleration(sens1->z);
    fdata->gx = convertRawGyro(sens2->x);
    fdata->gy = convertRawGyro(sens2->y);
    fdata->gz = convertRawGyro(sens2->z);
	
		#ifdef Madgwick
		MadgwickAHRSupdateIMU(fdata->gx,fdata->gy,fdata->gz,fdata->ax,fdata->ay,fdata->az);
		fdata->roll = Madgwick_getRoll();
		fdata->pitch = Madgwick_getPitch();
		fdata->heading = Madgwick_getYaw();
		#endif
	
		return SUCCESS;
}

/* Initialize the Auxiliary BMM150 following the above code 
 * until setting the power mode (Set the power mode as forced mode)
 * and preset mode */

/*
	// In BMM150 Mag data starts from register address 0x42 
	uint8_t aux_addr = 0x42;
	// Buffer to store the Mag data from 0x42 to 0x48 	
	uint8_t mag_data[8] = {0};
	
	uint8_t index;
		
	// Configure the Auxiliary sensor either in auto/manual modes and set the 
	polling frequency for the Auxiliary interface 	
	bmi160.aux_cfg.aux_odr = 8; // Represents polling rate in 100 Hz
	rslt = bmi160_config_aux_mode(&bmi160);
	
	// Set the auxiliary sensor to auto mode 
	rslt = bmi160_set_aux_auto_mode(&aux_addr, &bmi160);

	// Reading data from BMI160 data registers 
	rslt = bmi160_read_aux_data_auto_mode(mag_data, &bmi160);

	// Compensating the raw mag data available from the BMM150 API 
	rslt = bmm150_aux_mag_data(mag_data, &bmm150);
	
	USART1_SEND("\n COMPENSATED DATA ");
    
	sprintf(usart_TxBuf,"\r\n MAG DATA X : %d \r\n", bmm150.data.x);
    USART1_SEND(usart_TxBuf);
    sprintf(usart_TxBuf,"\r\n MAG DATA Y : %d \r\n", bmm150.data.y);
    USART1_SEND(usart_TxBuf);
    sprintf(usart_TxBuf,"\r\n MAG DATA Z : %d \r\n", bmm150.data.z);
    USART1_SEND(usart_TxBuf);
*/
