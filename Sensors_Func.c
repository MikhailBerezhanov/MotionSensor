/*==============================================================================
>File name:  	Sensors_func.c
>Brief:         Source code for init and work with BMI160,BMM150 in projectONE	
>Author:        LL
>Date:          07.05.2018
>Version:       1.0
==============================================================================*/

#include "Sensors_func.h"

//---------------------------= 101[V]ariables =--------------------------------
struct bmi160_dev bmi160;
struct bmi160_int_settg int_config;
struct bmi160_pmu_status pmu_status;	
struct bmi160_sensor_data accel;
struct bmi160_sensor_data gyro;	
struct Sensors_common_status sensor_status;
struct bmm150_dev bmm150;
struct bmm150_raw_mag_data raw;
int8_t rsltBMI160 = BMI160_OK;
int8_t rsltBMM150 = BMM150_OK;
filtered_data fdata;

extern struct MainMenu_Struct MainMenu;
extern char USART1_TxBuf[TX_BUF_SIZE];
extern uint8_t accel_mode;
extern uint8_t gyro_mode;
extern uint8_t magni_mode;
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
	
	if (bmi160_init(&bmi160) == BMI160_OK)
/* After the above function call, accel_cfg and gyro_cfg parameters in the device 
structure are set with default values, found in the datasheet of the bmi160. 
***Initializing accel_cfg and gyro_cfg parameters at the start of soft then	*/
	{		
	/* Fill configurations for accel */
	bmi160.accel_cfg.range = sensor_status.accel_cfg_range;					//Range settings
	bmi160.accel_cfg.bw = sensor_status.accel_cfg_bw;			//Bandwidth
	bmi160.accel_cfg.odr = sensor_status.accel_cfg_odr;					//Output data rate	
	
	/* Fill configurations for gyro */	
	bmi160.gyro_cfg.range =	sensor_status.gyro_cfg_range;
	bmi160.gyro_cfg.bw = sensor_status.gyro_cfg_bw;	
	bmi160.gyro_cfg.odr = sensor_status.gyro_cfg_odr;
    
	/* At the start BMI160 is in suspend mode */
	//bmi160.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;	
	//bmi160.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;	
		
  if (bmi160_set_sens_conf(&bmi160) == BMI160_OK) 					//Set above configurations of sensors
		if (BMI160_INT_config() == BMI160_OK) return BMI160_OK;	//Initialize INT1 dataready interrupt         
	}
return (!BMI160_OK);
}

/**
  * @brief   Set params or functionality BMI160 , power mode and enable DRDY IT in Normal mode
  * @param   *dev - pointer at struct bmi160_dev
			 accel_mode - BMI160_ACCEL_SUSPEND_MODE     gyro_mode - BMI160_GYRO_SUSPEND_MODE        
						- BMI160_ACCEL_NORMAL_MODE			  	  -	BMI160_GYRO_NORMAL_MODE          
						- BMI160_ACCEL_LOWPOWER_MODE			  -	BMI160_GYRO_FASTSTARTUP_MODE   												  												
  * @retval  SUCCESS = BMI160_OK, FAILURE = 0
  */
inline int8_t BMI160_set_Mode(struct bmi160_dev *dev, const uint8_t accel_mode, const uint8_t gyro_mode )
{
	/* Set power modes of accel and gyro */
	bmi160.accel_cfg.power = accel_mode;	
	
	bmi160.gyro_cfg.power = gyro_mode;

    if ((set_accel_pwr(&bmi160) == BMI160_OK)&&(set_gyro_pwr(&bmi160) == BMI160_OK))
     return BMI160_OK;
    
return -1;    
}

inline void BMI160_setGyro_Mode(struct bmi160_dev *dev, const uint8_t gyro_mode)
{
	bmi160.gyro_cfg.power = gyro_mode;
	set_gyro_pwr(&bmi160);
}

inline void BMI160_setAccel_Mode(struct bmi160_dev *dev, const uint8_t accel_mode)
{
	bmi160.accel_cfg.power = accel_mode;
	set_accel_pwr(&bmi160);	
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
  * @brief   Displays via USART the results of init and internal registers status BMI160
  * @note    After SelfTests SoftReset proceeded so need to recurve power and config settings
  * @param   accel_mode - 
  * @retval  SUCCESS = BMI160_OK
  */
int8_t BMI160_display_status (void)
{	
			EXTI_InitTypeDef DRDY_EnableDisable;
			/* Disable EXTI0 Interrupt (DRDY) when displaying status */
			DRDY_EnableDisable.EXTI_Line = EXTI_Line0;
			DRDY_EnableDisable.EXTI_LineCmd = DISABLE;
			DRDY_EnableDisable.EXTI_Mode = EXTI_Mode_Interrupt;
			DRDY_EnableDisable.EXTI_Trigger = EXTI_Trigger_Falling;
			EXTI_Init(&DRDY_EnableDisable);
			/*-------------------------------------------------------*/
			
			/* save current values into the status structure*/
			sensor_status.accel_pwr_mode = bmi160.accel_cfg.power;
			sensor_status.gyro_pwr_mode = bmi160.gyro_cfg.power;
            sensor_status.accel_cfg_range = bmi160.accel_cfg.range;					
            sensor_status.accel_cfg_bw = bmi160.accel_cfg.bw; 			
            sensor_status.accel_cfg_odr = bmi160.accel_cfg.odr; 						
            sensor_status.gyro_cfg_range = bmi160.gyro_cfg.range;	
            sensor_status.gyro_cfg_bw = bmi160.gyro_cfg.bw;	
            sensor_status.gyro_cfg_odr = bmi160.gyro_cfg.odr;
            /*-------------------------------------------------------*/
    
	/* Getting bmi160 ID */ 
	SPI2_Read_BMI160(1,(BMI160_CHIP_ID_ADDR | BMI160_SPI_RD_MASK),&sensor_status.id,1);
	//bmi160_get_regs(BMI160_CHIP_ID_ADDR,&sensor_status.id,1,&bmi160);
	sprintf(USART1_TxBuf, "\r\n\t> Sensor ID:\t\t0x%02X\r\n", sensor_status.id );			
    MainMenu.terminalSend(USART1_TxBuf);																		
		
	/* GYRO self test */				
	if (bmi160_perform_self_test(BMI160_GYRO_ONLY, &bmi160) != BMI160_OK) 	
	MainMenu.terminalSend("\t> GYRO SELF TEST:\tF A I L E D.\r\n");
	else MainMenu.terminalSend("\t> GYRO SELF TEST:\tSUCCESSFULL\r\n");
	//ends with soft reset so Initialization again needed 
	BMI160_init();

	/* ACCEl self test */	
	if (bmi160_perform_self_test(BMI160_ACCEL_ONLY, &bmi160) == BMI160_W_ACCEl_SELF_TEST_FAIL) 	
	MainMenu.terminalSend("\t> ACCELSELF TEST:\tF A I L E D.\r\n");
	else MainMenu.terminalSend("\t> ACCEL SELF TEST:\tSUCCESSFULL\r\n");
	//ends with soft reset so Initialization again needed			
	BMI160_init();
	BMI160_set_Mode(&bmi160,sensor_status.accel_pwr_mode,sensor_status.gyro_pwr_mode);

	/* Error status check */
	bmi160_get_regs(BMI160_ERROR_REG_ADDR, &sensor_status.err_status, 1, &bmi160);				
	sprintf(USART1_TxBuf, "\t> Error status:\t\t0x%02X", sensor_status.err_status );		
    MainMenu.terminalSend(USART1_TxBuf);			
	switch (sensor_status.err_status)
	{
		case /*0b0000*/ 0: MainMenu.terminalSend("\tNo error\r\n"); break;
		case /*0b0001*/ 1: MainMenu.terminalSend("\tError1\r\n"); break;
		case /*0b0010*/ 2: MainMenu.terminalSend("\tError2\r\n"); break;
		case /*0b0011*/ 3: MainMenu.terminalSend("\tLow power mode and INT uses pre-filtered data\r\n"); break;
		case /*0b0110*/ 6: MainMenu.terminalSend("\tODRs of enabled sensors in headerless mode don't match\r\n"); break;
		case /*0b0111*/ 7: MainMenu.terminalSend("\tPre-filtered data are used in low power mode\r\n");break;
		default: MainMenu.terminalSend("\tUNKNOWN_VAL\r\n"); break;		
	}

	/* GYRO power mode check */		
	bmi160_get_power_mode(&pmu_status,&bmi160);
	sprintf(USART1_TxBuf, "\r\n\t> Gyro PWR mode:\t0x%02X", pmu_status.gyro_pmu_status );		
    MainMenu.terminalSend(USART1_TxBuf);
	switch (pmu_status.gyro_pmu_status)
	{
		case /*0b00*/ 0: MainMenu.terminalSend("\tSUSPEND MODE\r\n"); break;
		case /*0b01*/ 1: MainMenu.terminalSend("\tNORMAL MODE\r\n"); break;
		case /*0b11*/ 3: MainMenu.terminalSend("\tFAST START-UP MODE\r\n"); break;
		default: MainMenu.terminalSend("\tUNKNOWN_VAL\r\n"); break;
	}	
	
	/*Gyro Sensor Settings */
	bmi160_get_regs(BMI160_GYRO_RANGE_ADDR,&sensor_status.gyro_range_reg,1,&bmi160);
	sprintf(USART1_TxBuf, "\t> Gyro range:\t\t0x%02X", sensor_status.gyro_range_reg );		
    MainMenu.terminalSend(USART1_TxBuf);	
	switch(sensor_status.gyro_range_reg & 0x07)
	{
		case BMI160_GYRO_RANGE_2000_DPS: MainMenu.terminalSend("\t2000_DPS\r\n"); break;
		case BMI160_GYRO_RANGE_1000_DPS: MainMenu.terminalSend("\t1000_DPS\r\n"); break;
		case BMI160_GYRO_RANGE_500_DPS: MainMenu.terminalSend("\t500_DPS\r\n"); break;
		case BMI160_GYRO_RANGE_250_DPS: MainMenu.terminalSend("\t250_DPS\r\n"); break;
		case BMI160_GYRO_RANGE_125_DPS: MainMenu.terminalSend("\t125_DPS\r\n"); break;
		default: MainMenu.terminalSend("\tUNKNOWN_VAL\r\n"); break;
	}
	
	bmi160_get_regs(BMI160_GYRO_CONFIG_ADDR,&sensor_status.gyro_config_reg,1,&bmi160);
	sprintf(USART1_TxBuf, "\t> Gyro Bandwidth:\t0x%02X", (sensor_status.gyro_config_reg >> 4));		
    MainMenu.terminalSend(USART1_TxBuf);	
	switch(sensor_status.gyro_config_reg >> 4)
	{
		case BMI160_GYRO_BW_OSR4_MODE : MainMenu.terminalSend("\tOSR4_MODE\r\n"); break;
		case BMI160_GYRO_BW_OSR2_MODE: MainMenu.terminalSend("\tOSR2_MODE\r\n"); break;
		case BMI160_GYRO_BW_NORMAL_MODE: MainMenu.terminalSend("\tNORMAL_MODE\r\n"); break;
		default: MainMenu.terminalSend("\tUNKNOWN_VAL\r\n"); break;
	}

	sprintf(USART1_TxBuf, "\t> Gyro OutDataRate:\t0x%02X", (sensor_status.gyro_config_reg & 0x0F) );		
    MainMenu.terminalSend(USART1_TxBuf);	
	switch(sensor_status.gyro_config_reg & 0x0F)
	{
		case BMI160_GYRO_ODR_25HZ : MainMenu.terminalSend("\t25HZ\r\n"); break;
		case BMI160_GYRO_ODR_50HZ: MainMenu.terminalSend("\t50HZ\r\n"); break;
		case BMI160_GYRO_ODR_100HZ: MainMenu.terminalSend("\t100HZ\r\n"); break;
		case BMI160_GYRO_ODR_200HZ: MainMenu.terminalSend("\t200HZ\r\n"); break;
		case BMI160_GYRO_ODR_400HZ: MainMenu.terminalSend("\t400HZ\r\n"); break;
		case BMI160_GYRO_ODR_800HZ: MainMenu.terminalSend("\t800HZ\r\n"); break;
		case BMI160_GYRO_ODR_1600HZ : MainMenu.terminalSend("\t1600HZ\r\n"); break;
		case BMI160_GYRO_ODR_3200HZ: MainMenu.terminalSend("\t3200HZ\r\n"); break;
		default: MainMenu.terminalSend("\tUNKNOWN_VAL\r\n"); break;
	}		
	
	sprintf(USART1_TxBuf, "\r\n\t> Accel PWR mode:\t0x%02X", pmu_status.accel_pmu_status );		
    MainMenu.terminalSend(USART1_TxBuf);
	switch (pmu_status.accel_pmu_status)
	{
		case /*0b00*/ 0: MainMenu.terminalSend("\tSUSPEND MODE\r\n"); break;
		case /*0b01*/ 1: MainMenu.terminalSend("\tNORMAL MODE\r\n"); break;
		case /*0b10*/ 2: MainMenu.terminalSend("\tLOW POWER MODE\r\n"); break;
		default: MainMenu.terminalSend("\tUNKNOWN_VAL\r\n"); break;
	}

	
	/* Accel sensor settings */
	bmi160_get_regs(BMI160_ACCEL_RANGE_ADDR,&sensor_status.accel_range_reg,1,&bmi160);
	sprintf(USART1_TxBuf, "\t> Accel range:\t\t0x%02X", sensor_status.accel_range_reg & 0x0F );		
    MainMenu.terminalSend(USART1_TxBuf);	
	switch(sensor_status.accel_range_reg & 0x0F)
	{
		case BMI160_ACCEL_RANGE_2G: MainMenu.terminalSend("\t2G\r\n"); break;
		case BMI160_ACCEL_RANGE_4G: MainMenu.terminalSend("\t4G\r\n"); break;
		case BMI160_ACCEL_RANGE_8G: MainMenu.terminalSend("\t8G\r\n"); break;
		case BMI160_ACCEL_RANGE_16G: MainMenu.terminalSend("\t16G\r\n"); break;
		default: MainMenu.terminalSend("\tDEFAULT 2G\r\n"); break;
	}
	
	bmi160_get_regs(BMI160_ACCEL_CONFIG_ADDR,&sensor_status.accel_config_reg,1,&bmi160);
	sprintf(USART1_TxBuf, "\t> Accel Bandwidth:\t0x%02X",(sensor_status.accel_config_reg >> 4));		
    MainMenu.terminalSend(USART1_TxBuf);
	switch (sensor_status.accel_config_reg >> 4)
	{
		case BMI160_ACCEL_BW_OSR4_AVG1: MainMenu.terminalSend("\tOSR4_AVG1\r\n"); break;
		case BMI160_ACCEL_BW_OSR2_AVG2: MainMenu.terminalSend("\tOSR2_AVG2\r\n"); break;
		case BMI160_ACCEL_BW_NORMAL_AVG4: MainMenu.terminalSend("\tNORMAL_AVG4\r\n"); break;
		case BMI160_ACCEL_BW_RES_AVG8: MainMenu.terminalSend("\tRES_AVG8\r\n"); break;
		case BMI160_ACCEL_BW_RES_AVG16: MainMenu.terminalSend("\tRES_AVG16\r\n"); break;
		case BMI160_ACCEL_BW_RES_AVG32: MainMenu.terminalSend("\tRES_AVG32\r\n"); break;
		case BMI160_ACCEL_BW_RES_AVG64: MainMenu.terminalSend("\tRES_AVG64\r\n"); break; 
		case BMI160_ACCEL_BW_RES_AVG128: MainMenu.terminalSend("\tRES_AVG128\r\n"); break;
		default: MainMenu.terminalSend("\tUNKNOWN_VAL\r\n"); break;
	}
	
	sprintf(USART1_TxBuf, "\t> Accel OutDataRate:\t0x%02X",(sensor_status.accel_config_reg & 0x0F));		
    MainMenu.terminalSend(USART1_TxBuf);
	switch(sensor_status.accel_config_reg & 0x0F)
	{
		case BMI160_ACCEL_ODR_0_78HZ: MainMenu.terminalSend("\t0_78HZ\r\n"); break;
		case BMI160_ACCEL_ODR_1_56HZ: MainMenu.terminalSend("\t1_56HZ\r\n"); break;
		case BMI160_ACCEL_ODR_3_12HZ: MainMenu.terminalSend("\t3_12HZ\r\n"); break;
		case BMI160_ACCEL_ODR_6_25HZ: MainMenu.terminalSend("\t6_25HZ\r\n"); break;
		case BMI160_ACCEL_ODR_12_5HZ: MainMenu.terminalSend("\t12_5HZ\r\n"); break;
		case BMI160_ACCEL_ODR_25HZ: MainMenu.terminalSend("\t25HZ\r\n"); break;
		case BMI160_ACCEL_ODR_50HZ: MainMenu.terminalSend("\t50HZ\r\n"); break; 
		case BMI160_ACCEL_ODR_100HZ: MainMenu.terminalSend("\t100HZ\r\n"); break;
		case BMI160_ACCEL_ODR_200HZ: MainMenu.terminalSend("\t200HZ\r\n"); break;
		case BMI160_ACCEL_ODR_400HZ: MainMenu.terminalSend("\t400HZ\r\n"); break;
		case BMI160_ACCEL_ODR_800HZ : MainMenu.terminalSend("\t800HZ\r\n"); break;
		case BMI160_ACCEL_ODR_1600HZ: MainMenu.terminalSend("\t1600HZ\r\n"); break; 
		default: MainMenu.terminalSend("\tUNKNOWN_VAL\r\n"); break;
	}
	
	/* Interrupt status */
	
	bmi160_get_regs(BMI160_INT_ENABLE_0_ADDR,&sensor_status.INT_status,1,&bmi160);
	sprintf(USART1_TxBuf, "\r\n\t> Interrupt[0] status:\t0x%02X",sensor_status.INT_status);
	MainMenu.terminalSend(USART1_TxBuf);
	if (sensor_status.INT_status & 0x80) MainMenu.terminalSend("\tFlat En");
	if (sensor_status.INT_status & 0x40) MainMenu.terminalSend("\tOrientation En");
	if (sensor_status.INT_status & 0x20) MainMenu.terminalSend("\tSingle tap full En");
	if (sensor_status.INT_status & 0x10) MainMenu.terminalSend("\tDouble tap En");
	if (sensor_status.INT_status & 0x04) MainMenu.terminalSend("\tAny motion z En");
	if (sensor_status.INT_status & 0x02) MainMenu.terminalSend("\tAny motion y En");
	if (sensor_status.INT_status & 0x01) MainMenu.terminalSend("\tAny motion x En");
	MainMenu.terminalSend("\r\n");
	
	bmi160_get_regs(BMI160_INT_ENABLE_1_ADDR,&sensor_status.INT_status,1,&bmi160);
	sprintf(USART1_TxBuf, "\t> Interrupt[1] status:\t0x%02X",sensor_status.INT_status);
	MainMenu.terminalSend(USART1_TxBuf);
	if (sensor_status.INT_status & 0x40) MainMenu.terminalSend("\tFIFO watermark En");
	if (sensor_status.INT_status & 0x20) MainMenu.terminalSend("\tFIFO full En");
	if (sensor_status.INT_status & 0x10) MainMenu.terminalSend("\tDRDY En");
	if (sensor_status.INT_status & 0x08) MainMenu.terminalSend("\tLow g En");
	if (sensor_status.INT_status & 0x04) MainMenu.terminalSend("\tHigh g z En");
	if (sensor_status.INT_status & 0x02) MainMenu.terminalSend("\tHigh g y En");
	if (sensor_status.INT_status & 0x01) MainMenu.terminalSend("\tHigh g x En");
	MainMenu.terminalSend("\r\n");
	
	bmi160_get_regs(BMI160_INT_ENABLE_2_ADDR,&sensor_status.INT_status,1,&bmi160);
	sprintf(USART1_TxBuf, "\t> Interrupt[2] status:\t0x%02X",sensor_status.INT_status);
	MainMenu.terminalSend(USART1_TxBuf);
	if (sensor_status.INT_status & 0x08) MainMenu.terminalSend("\tStep detector En");
	if (sensor_status.INT_status & 0x04) MainMenu.terminalSend("\tNo/slow motion z En");
	if (sensor_status.INT_status & 0x02) MainMenu.terminalSend("\tNo/slow motion En");
	if (sensor_status.INT_status & 0x01) MainMenu.terminalSend("\tNo/slow motion En");
	MainMenu.terminalSend("\r\n");
	
	/* Power mode status check */
	bmi160_get_regs(BMI160_PMU_STATUS_ADDR, &sensor_status.pwr_status, 1, &bmi160);				
	sprintf(USART1_TxBuf, "\t> PWR status:\t\t0x%02X\r\n", sensor_status.pwr_status );		
    MainMenu.terminalSend(USART1_TxBuf);			

	/* Common bmi160 status check */				
	bmi160_get_regs(BMI160_STATUS_ADDR,&sensor_status.comm_status,1,&bmi160);
	sprintf(USART1_TxBuf, "\t> Common Status:\t0x%02X\r\n", sensor_status.comm_status );		
    MainMenu.terminalSend(USART1_TxBuf);					

	if (pmu_status.gyro_pmu_status == 1)
	{
	/* Temperature1 status check */
	bmi160_get_regs(0x20, &sensor_status.temperature1, 1, &bmi160);				
	sprintf(USART1_TxBuf, "\t> Temperature1:\t\t0x%02X\r\n", sensor_status.temperature1 );		
    MainMenu.terminalSend(USART1_TxBuf);
	
	/* Temperature2 status check */
	bmi160_get_regs(0x21, &sensor_status.temperature2, 1, &bmi160);				
	sprintf(USART1_TxBuf, "\t> Temperature2:\t\t0x%02X", sensor_status.temperature2 );		
    MainMenu.terminalSend(USART1_TxBuf);

	uint16_t Tempr = 0;
	Tempr = sensor_status.temperature2 << 8;
	Tempr += sensor_status.temperature1;
	sprintf(USART1_TxBuf, "\tTotal:%#X\r\n", Tempr );		
    MainMenu.terminalSend(USART1_TxBuf);		
	}
	
	/* Getting data values from sensors */
	bmi160_get_sensor_data(BMI160_BOTH_ACCEL_AND_GYRO,&accel,&gyro,&bmi160);
	sprintf(USART1_TxBuf, "\r\n\tX_Gyro value:\t\t%d\r\n", gyro.x );			
    MainMenu.terminalSend(USART1_TxBuf);				
	sprintf(USART1_TxBuf, "\tY_Gyro value:\t\t%d\r\n", gyro.y );			
    MainMenu.terminalSend(USART1_TxBuf);			
	sprintf(USART1_TxBuf, "\tZ_Gyro value:\t\t%d\r\n", gyro.z );			
    MainMenu.terminalSend(USART1_TxBuf);			
	sprintf(USART1_TxBuf, "\r\n\tX_Accel value:\t\t%d\r\n", accel.x );			
    MainMenu.terminalSend(USART1_TxBuf);			
	sprintf(USART1_TxBuf, "\tY_Accel value:\t\t%d\r\n", accel.y );			
    MainMenu.terminalSend(USART1_TxBuf);			
	sprintf(USART1_TxBuf, "\tZ_Accel value:\t\t%d\r\n", accel.z );			
    MainMenu.terminalSend(USART1_TxBuf);	
	
            /* ENABLE EXTI0 Interrupt (DRDY) after displaying status */
            DRDY_EnableDisable.EXTI_Line = EXTI_Line0;
            DRDY_EnableDisable.EXTI_LineCmd = ENABLE;
            DRDY_EnableDisable.EXTI_Mode = EXTI_Mode_Interrupt;
            DRDY_EnableDisable.EXTI_Trigger = EXTI_Trigger_Falling;
            EXTI_Init(&DRDY_EnableDisable);
            /*-------------------------------------------------------*/
return (BMI160_OK);
}


/**
  * @brief   Read accel and gyro from BMI160 data and display it via terminal
  * @param   None
  * @retval  Sucsess = BMI160_OK
  */
inline void BMI160_display_data (void)
{			
	//BMI160_OptGet_data(&accel,&gyro);
	//rsltBMI160  = bmi160_get_sensor_data(BMI160_BOTH_ACCEL_AND_GYRO,&accel,&gyro,&bmi160);
    
sprintf(USART1_TxBuf, "\r G_value: %d %d %d\t\tA_value: %d %d %d\r",
        gyro.x,gyro.y,gyro.z,accel.x,accel.y,accel.z);
    
MainMenu.terminalSend(USART1_TxBuf);	
}

/**
  * @brief	Reading from BMI160 registers via SPI2
  *	@note		Can be faster. Watch notes in func code.
  * @param  id - not used , reg_addr - address of register to read, 
						*data - pointer of data that has been read, len - No. of bytes to read
  * @retval SUCCESS = BMI160_OK
  */
static inline int8_t SPI2_Read_BMI160(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{ 
	/******** Optimized realisation *****||********** Analog with functions calls **********/	
	
	SPI2->DR;		//dunno why it's here but it doesnt read MOSI answer byte properly without it
	//drop CSB pin = start conversation
	GPIOB->BRR = BMI160_CSB_PIN;								//GPIO_ResetBits(SPI_GPIO_PORT,SPI_CSB_PIN);	
	
	SPI2->DR = reg_addr;										//SPI_I2S_SendData(SPI2,reg_addr);
	
	while ((SPI2->SR & SPI_I2S_FLAG_TXE) == (uint16_t)RESET){}	//while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	
	/* NOTE! Skipping this waiting of flag and reading from SPI_RX buffer will make SPI2_Read_BMM150 function 2times faster 
	but it will read false response (first MOSI byte instead of 2nd). May be there's some way to fix it => make func faster. 	
	-------------------------------------------------------------------------------------------------------------------------*/
  while ((SPI2->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);	//while (SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_RXNE) == RESET){}	
	//clear Rx buffer after sending BMI160 reg_adr byte
	SPI2->DR;																	//SPI_I2S_ReceiveData(SPI2);
	/*-----------------------------------------------------------------------------------------------------------------------*/	
	for (register uint16_t i=0; i < len; i++)
	{
	//send CLK for MISO data
	SPI2->DR = 0x00;											//SPI_I2S_SendData(SPI2,0x00);	
	while ((SPI2->SR & SPI_I2S_FLAG_TXE) == (uint16_t)RESET){}	//while (SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE) == RESET){}
 		
	while ((SPI2->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);	//while (SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_RXNE) == RESET){}
		
	//get data from BMI160 reg
	*data = (uint8_t)SPI2->DR;								//*data = (uint8_t)SPI_I2S_ReceiveData(SPI2);
	data++;
	}
	
	//Stop conversation
	GPIOB->BSRR = BMI160_CSB_PIN;							//GPIO_SetBits(SPI_GPIO_PORT,SPI_CSB_PIN);
	return BMI160_OK; 
}

/**
    * @brief	write to BMI160 reg via SPI2  
	* @note		
	* @param  id - not used , reg_addr - address of register to write, 
			  *data - pointer to data that must be written, len - No. of bytes to write
  * @retval SUCCESS = BMI160_OK
  */
static inline int8_t SPI2_Write_BMI160 (uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	uint8_t *ptmp = data;			//get start address of data
	
	for (uint16_t i = 0; i < len; i++)
	{
	GPIO_ResetBits(SPI_GPIO_PORT,BMI160_CSB_PIN);					//drop CSB pin = start conversation
	
	SPI_I2S_SendData(SPI2, reg_addr);  							// send reg addr
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET); // wait till SPI_DR become empty
	
	SPI_I2S_SendData(SPI2, *data);  							// send data
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET); //  wait till SPI_DR become empty
		
	GPIO_SetBits(SPI_GPIO_PORT,BMI160_CSB_PIN);		//up CSB pin = stop conversation
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
inline void BMI160_OptGet_data (struct bmi160_sensor_data *accel, struct bmi160_sensor_data *gyro)
{
	int8_t rsltBMI160 ;
	uint8_t idx = 0;
	uint8_t data_array[15] = {0};
	uint8_t lsb;
	uint8_t msb;
	int16_t msblsb;

	/* read both accel and gyro bmi160 data
	 * along with time if requested */
	rsltBMI160 = SPI2_Read_BMI160(1, BMI160_GYRO_DATA_ADDR | BMI160_SPI_RD_MASK, data_array, 12);
	//rsltBMI160  = bmi160_get_regs(BMI160_GYRO_DATA_ADDR, data_array, 12 + len, dev);
	
	if (rsltBMI160  == BMI160_OK) 
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

}

/**
  * @brief   !!OPTIMIZED for hardware Read accel and gyro data from BMI160
  * @param   
  * @retval  Success = BMI160_OK
 */ 
inline void BMI160_OptGet_data2 (char *buf)
{
	int8_t rsltBMI160 ;
	uint8_t idx = 0;
	uint8_t data_array[15] = {0};
	uint8_t lsb;
	uint8_t msb;
	int16_t msblsb;
    uint16_t temp;
	/* read both accel and gyro bmi160 data
	 * along with time if requested */
	rsltBMI160 = SPI2_Read_BMI160(1, BMI160_GYRO_DATA_ADDR | BMI160_SPI_RD_MASK, data_array, 12);
	//rsltBMI160  = bmi160_get_regs(BMI160_GYRO_DATA_ADDR, data_array, 12 + len, dev);
	
	if (rsltBMI160  == BMI160_OK) 
		{
		// Gyro Data 
		lsb = data_array[idx++];
		msb = data_array[idx++];
		msblsb = (int16_t)((msb << 8) | lsb);
		//gyro->x = msblsb; // gyro X axis data
            *buf++ = '\r';
            *buf++ = '\n';
                if (msblsb <0) *buf++ = '-';// sign 
                else *buf++ = ' ';
            msblsb = abs(msblsb);       // take module of value
            temp = msblsb/10000;    // 1st number of value
            *buf++ = (char)temp + 48;     // transform to ASCII and write to buffer
            temp = msblsb/1000;
            temp %= 10;
            *buf++ = (char)temp + 48;     // 2nd num
            temp = msblsb/100;
            temp %= 10;
            *buf++ = (char)temp + 48;     // 3d num
            temp = msblsb/10;
            temp %= 10;
            *buf++ = (char)temp + 48;     // 4th num
            temp = msblsb%10;
            *buf++ = (char)temp + 48;     // 5th num
            *buf++ = '\t';
                
		lsb = data_array[idx++];
		msb = data_array[idx++];
		msblsb = (int16_t)((msb << 8) | lsb);
		//gyro->y = msblsb; // gyro Y axis data 
                if (msblsb <0) *buf++ = '-';// sign 
                else *buf++ = ' ';
            msblsb = abs(msblsb);       // take module of value
            temp = msblsb/10000;    // 1st number of value
            *buf++ = (char)temp + 48;     // transform to ASCII and write to buffer
            temp = msblsb/1000;
            temp %= 10;
            *buf++ = (char)temp + 48;     // 2nd num
            temp = msblsb/100;
            temp %= 10;
            *buf++ = (char)temp + 48;     // 3d num
            temp = msblsb/10;
            temp %= 10;
            *buf++ = (char)temp + 48;     // 4th num
            temp = msblsb%10;
            *buf++ = (char)temp + 48;     // 5th num
            *buf++ = '\t';
            
		lsb = data_array[idx++];
		msb = data_array[idx++];
		msblsb = (int16_t)((msb << 8) | lsb);
		//gyro->z = msblsb; // gyro Z axis data 
                if (msblsb <0) *buf++ = '-';// sign 
                else *buf++ = ' ';
            msblsb = abs(msblsb);       // take module of value
            temp = msblsb/10000;    // 1st number of value
            *buf++ = (char)temp + 48;     // transform to ASCII and write to buffer
            temp = msblsb/1000;
            temp %= 10;
            *buf++ = (char)temp + 48;     // 2nd num
            temp = msblsb/100;
            temp %= 10;
            *buf++ = (char)temp + 48;     // 3d num
            temp = msblsb/10;
            temp %= 10;
            *buf++ = (char)temp + 48;     // 4th num
            temp = msblsb%10;
            *buf++ = (char)temp + 48;     // 5th num
            *buf++ = '\t';
            
		// Accel Data 
		lsb = data_array[idx++];
		msb = data_array[idx++];
		msblsb = (int16_t)((msb << 8) | lsb);
		//accel->x = (int16_t)msblsb; // accel X axis data 
                if (msblsb <0) *buf++ = '-';// sign 
                else *buf++ = ' ';
            msblsb = abs(msblsb);       // take module of value
            temp = msblsb/10000;    // 1st number of value
            *buf++ = (char)temp + 48;     // transform to ASCII and write to buffer
            temp = msblsb/1000;
            temp %= 10;
            *buf++ = (char)temp + 48;     // 2nd num
            temp = msblsb/100;
            temp %= 10;
            *buf++ = (char)temp + 48;     // 3d num
            temp = msblsb/10;
            temp %= 10;
            *buf++ = (char)temp + 48;     // 4th num
            temp = msblsb%10;
            *buf++ = (char)temp + 48;     // 5th num
            *buf++ = '\t';
            
		lsb = data_array[idx++];
		msb = data_array[idx++];
		msblsb = (int16_t)((msb << 8) | lsb);
		//accel->y = (int16_t)msblsb; // accel Y axis data 
                if (msblsb <0) *buf++ = '-';// sign 
                else *buf++ = ' ';
            msblsb = abs(msblsb);       // take module of value
            temp = msblsb/10000;    // 1st number of value
            *buf++ = (char)temp + 48;     // transform to ASCII and write to buffer
            temp = msblsb/1000;
            temp %= 10;
            *buf++ = (char)temp + 48;     // 2nd num
            temp = msblsb/100;
            temp %= 10;
            *buf++ = (char)temp + 48;     // 3d num
            temp = msblsb/10;
            temp %= 10;
            *buf++ = (char)temp + 48;     // 4th num
            temp = msblsb%10;
            *buf++ = (char)temp + 48;     // 5th num
            *buf++ = '\t';
		lsb = data_array[idx++];
		msb = data_array[idx++];
		msblsb = (int16_t)((msb << 8) | lsb);
		//accel->z = (int16_t)msblsb; // accel Z axis data
                if (msblsb <0) *buf++ = '-';// sign 
                else *buf++ = ' ';
            msblsb = abs(msblsb);       // take module of value
            temp = msblsb/10000;    // 1st number of value
            *buf++ = (char)temp + 48;     // transform to ASCII and write to buffer
            temp = msblsb/1000;
            temp %= 10;
            *buf++ = (char)temp + 48;     // 2nd num
            temp = msblsb/100;
            temp %= 10;
            *buf++ = (char)temp + 48;     // 3d num
            temp = msblsb/10;
            temp %= 10;
            *buf++ = (char)temp + 48;     // 4th num
            temp = msblsb%10;
            *buf++ = (char)temp + 48;     // 5th num
            *buf = '\0';
		}

}

/**
  * @brief   !Read data from BMI160 and filter it with Madgwick filter algorythm
  * @param   
  * @retval  Success = BMI160_OK
 */ 
inline void BMI160_OptGet_FilteredData (filtered_data *fdata)
{
	uint8_t idx = 0;
	uint8_t data_array[15] = {0};
	uint8_t lsb;
	uint8_t msb;
	int16_t msblsb;
    float f_Gx = 0;
    float f_Gy = 0;
    float f_Gz = 0;
    float f_Ax = 0;
    float f_Ay = 0;
    float f_Az = 0;
    
	/* read both accel and gyro bmi160 data
	 * along with time if requested */
	SPI2_Read_BMI160(1, BMI160_GYRO_DATA_ADDR | BMI160_SPI_RD_MASK, data_array, 12);
	//rsltBMI160  = bmi160_get_regs(BMI160_GYRO_DATA_ADDR, data_array, 12 + len, dev);
	
    // Gyro Data 
    lsb = data_array[idx++];
    msb = data_array[idx++];
    msblsb = (int16_t)((msb << 8) | lsb);   //raw value
    /*           convertRawGyro         
    since we are using 250 degrees/seconds range
    -250 maps to a raw value of -32768
    +250 maps to a raw value of 32767  */  
    f_Gx = (msblsb * 250.0) / 32768.0;    

    lsb = data_array[idx++];
    msb = data_array[idx++];
    msblsb = (int16_t)((msb << 8) | lsb);
    f_Gy = (msblsb * 250.0) / 32768.0;  

    lsb = data_array[idx++];
    msb = data_array[idx++];
    msblsb = (int16_t)((msb << 8) | lsb);
    f_Gz = (msblsb * 250.0) / 32768.0;  

    // Accel Data 
    lsb = data_array[idx++];
    msb = data_array[idx++];
    msblsb = (int16_t)((msb << 8) | lsb);
    /* convertRawAcceleration
    // since we are using 2G range
    // -2g maps to a raw value of -32768
    // +2g maps to a raw value of 32767 */
    f_Ax = (msblsb * 2.0) / 32768.0;
    
    lsb = data_array[idx++];
    msb = data_array[idx++];
    msblsb = (int16_t)((msb << 8) | lsb);
    f_Ay = (msblsb * 2.0) / 32768.0;

    lsb = data_array[idx++];
    msb = data_array[idx++];
    msblsb = (int16_t)((msb << 8) | lsb);
    f_Az = (msblsb * 2.0) / 32768.0; 
    
    MadgwickAHRSupdateIMU(f_Gx,f_Gy,f_Gz,f_Ax,f_Ay,f_Az);
    fdata->roll = Madgwick_getRoll();
    fdata->pitch = Madgwick_getPitch();
    fdata->heading = Madgwick_getYaw();
		  
}



/*======================================================================================*/
/*----------------------------------- BMI150 functions ---------------------------------*/
/*======================================================================================*/


/**
  * @brief	BMM150 init SPI-4 wire
  *	@note	if start was with suspend mode 
            after bmi150_init will be sleep mode 
            otherwise mode will stay as at start	
  * @param  
  * @retval rsltBMM150
  */
int8_t BMM150_init (void)
{
/* Sensor interface over SPI with native chip select line */
bmm150.dev_id = 0;
bmm150.intf = BMM150_SPI_INTF;
bmm150.read = SPI2_Read_BMM150;
bmm150.write = SPI2_Write_BMM150;
bmm150.delay_ms = _delay_ms;   

/* startup with suspend mode as default => after bmm150_init must be sleep mode */  
rsltBMM150 = bmm150_init(&bmm150);
return 	rsltBMM150;
}


/**
  * @brief	set PWR mode, DATARATE and REPITITIONS params of sensor if Active mode
  *	@note	watch bmm150_set_presetmode func for more info and other settings
  * @param  *dev = pointer at bmm150 structure
			mode = 	BMM150_NORMAL_MODE		 
					BMM150_FORCED_MODE					
					BMM150_SLEEP_MODE					
					BMM150_SUSPEND_MODE					
  * @retval SUCCESS = BMM150_OK
  */
inline int8_t BMM150_set_Mode(struct bmm150_dev *dev, const uint8_t mode)
{	
	/* Setting the power mode */
	dev->settings.pwr_mode = mode;
return	(bmm150_set_op_mode(dev));	
}

/**
  * @brief	set PWR mode, DATARATE and REPITITIONS params of sensor if Active mode
  *	@note	watch bmm150_set_presetmode func for more info and other settings
  * @param  *dev = pointer at bmm150 structure
			preset  - 	BMM150_PRESETMODE_LOWPOWER 
						BMM150_PRESETMODE_REGULAR
						BMM150_PRESETMODE_HIGHACCURACY
						BMM150_PRESETMODE_ENHANCED
  * @retval SUCCESS = BMM150_OK , -1 if impossible to set preset
  */
inline int8_t BMM150_set_Preset(struct bmm150_dev *dev, const uint8_t preset)
{  
 	/* If Active mode */
	if ((dev->settings.pwr_mode == BMM150_NORMAL_MODE)||
        (dev->settings.pwr_mode == BMM150_FORCED_MODE))
	{
	/* Setting the preset mode */
	dev->settings.preset_mode = preset;
	return(bmm150_set_presetmode(dev));
	} 
return -1;    
}

/**
  * @brief	Enables DATAREADY (DRDY) interrupt 
  *	@note		
  * @param  *dev = pointer at bmm150 structure
  * @retval SUCCESS = BMM150_OK
  */
int8_t BMM150_DRDY_config(struct bmm150_dev *dev)
{
	int8_t rslt;
	uint16_t desired_settings;

	/* Set the macros to enable DRDY pin */
	desired_settings = BMM150_DRDY_PIN_EN_SEL | BMM150_DRDY_POLARITY_SEL;
	/* Set the drdy_pin_en to enable the drdy interrupt  */
	dev->settings.int_settings.drdy_pin_en = BMM150_INT_ENABLE;
	/* Enable data overrun  */
	//dev->settings.int_settings.data_overrun_en = BMM150_INT_ENABLE;
	/* Set the polarity as active high on the DRDY pin */
	dev->settings.int_settings.drdy_polarity = BMM150_ACTIVE_LOW_POLARITY; 
	
	/* Set the configurations in the sensor */
	rslt = bmm150_set_sensor_settings(desired_settings, dev);
	
	return rslt;
}

/**
  * @brief	Displays status of BMM150 registers   
  * @note		
  * @param  
  * @retval SUCCESS = BMM150_OK
  */
inline void BMM150_OptGet_data(struct bmm150_dev *dev,struct bmm150_raw_mag_data *raw_mag_data)
{
	int16_t msb_data;
	uint8_t reg_data[BMM150_XYZR_DATA_LEN] = {0};
    
    /* activate FORCED_MODE at first to read up at >300 Hz  */
    dev->settings.pwr_mode = BMM150_FORCED_MODE;         
    bmm150_set_op_mode(dev);
    
    /* Get Magni data */
    
    /*Read the mag data registers */
    //bmm150_get_regs(BMM150_DATA_X_LSB, reg_data, BMM150_XYZR_DATA_LEN, dev);
    SPI2_Read_BMM150(1,BMM150_DATA_X_LSB | 0x80, reg_data,BMM150_XYZR_DATA_LEN);
    
    /* Mag X axis data */
    reg_data[0] = BMM150_GET_BITS(reg_data[0], BMM150_DATA_X);
    /* Shift the MSB data to left by 5 bits */
    /* Multiply by 32 to get the shift left by 5 value */
    msb_data = ((int16_t)((int8_t)reg_data[1]))<< 5;  // * 32;
    /* Raw mag X axis data */
    raw_mag_data->raw_datax = (int16_t)(msb_data | reg_data[0]);
    
    /* Mag Y axis data */
    reg_data[2] = BMM150_GET_BITS(reg_data[2], BMM150_DATA_Y);
    /* Shift the MSB data to left by 5 bits */
    /* Multiply by 32 to get the shift left by 5 value */
    msb_data = ((int16_t)((int8_t)reg_data[3]))<< 5; //* 32;
    /* Raw mag Y axis data */
    raw_mag_data->raw_datay = (int16_t)(msb_data | reg_data[2]);
    
    /* Mag Z axis data */
    reg_data[4] = BMM150_GET_BITS(reg_data[4], BMM150_DATA_Z);
    /* Shift the MSB data to left by 7 bits */
    /* Multiply by 128 to get the shift left by 7 value */
    msb_data = ((int16_t)((int8_t)reg_data[5]))<< 7;// * 128;
    /* Raw mag Z axis data */
    raw_mag_data->raw_dataz = (int16_t)(msb_data | reg_data[4]);
    
    /* Mag R-HALL data */
    //reg_data[6] = BMM150_GET_BITS(reg_data[6], BMM150_DATA_RHALL);
    //raw_mag_data->raw_data_r = (uint16_t)(((uint16_t)reg_data[7] << 6) | reg_data[6]);
    
    /* Compensated Mag X data in int16_t format */
    //dev->data.x = compensate_x(raw_mag_data.raw_datax, raw_mag_data.raw_data_r, dev);
    /* Compensated Mag Y data in int16_t format */
    //dev->data.y = compensate_y(raw_mag_data.raw_datay, raw_mag_data.raw_data_r, dev);
    /* Compensated Mag Z data in int16_t format */
    //dev->data.z = compensate_z(raw_mag_data.raw_dataz, raw_mag_data.raw_data_r, dev);

    /* ends up with SLEEP_MODE */
}

/**
  * @brief	Displays status of BMM150 registers   
  * @note		
  * @param  
  * @retval SUCCESS = BMM150_OK
  */
int8_t BMM150_display_status (void)
{
            EXTI_InitTypeDef DRDY_EnableDisable;
            /* Disable EXTI0 Interrupt (DRDY) when displaying status */
            DRDY_EnableDisable.EXTI_Line = EXTI_Line0;
            DRDY_EnableDisable.EXTI_LineCmd = DISABLE;
            DRDY_EnableDisable.EXTI_Mode = EXTI_Mode_Interrupt;
            DRDY_EnableDisable.EXTI_Trigger = EXTI_Trigger_Falling;
            EXTI_Init(&DRDY_EnableDisable);
            /*-------------------------------------------------------*/
            /* save current sensor's values */ 
            bmm150_get_op_mode(&sensor_status.bmm150_pwr_mode,&bmm150);    
            //bmm150_get_sensor_settings(&bmm150);
            sensor_status.bmm150_data_rate = bmm150.settings.data_rate;
            sensor_status.bmm150_xy_rep = bmm150.settings.xy_rep;
            sensor_status.bmm150_z_rep = bmm150.settings.z_rep;
            sensor_status.bmm150_preset_mode = bmm150.settings.preset_mode;
            /*-------------------------------------------------------*/   
	//int8_t rsltAdvSelfTest = -1;
	int32_t adv_self_test_Value = 0;
	
	/* Getting bmm150 ID */ 
	SPI2_Read_BMM150(1,(BMM150_CHIP_ID_ADDR | 0x80),&sensor_status.id,1);
	//bmm150_get_regs(BMM150_CHIP_ID_ADDR, &sensor_status.id, 1, &bmm150);
	sprintf(USART1_TxBuf, "\r\n\t> Sensor ID:\t\t0x%02X\r\n", sensor_status.id );			
	MainMenu.terminalSend(USART1_TxBuf);
    
    /* BMM150 ADVANCED self test 	
	rsltAdvSelfTest = bmm150_perform_self_test(BMM150_ADVANCED_SELF_TEST, &bmm150, &adv_self_test_Value);
	if (rsltAdvSelfTest == BMM150_OK) 	
	MainMenu.terminalSend("\t> ADVANCED SELF TEST:\tSUCCESSFULL\r\n");
	else MainMenu.terminalSend("\t> ADVANCED SELF TEST:\tFAILED");
	sprintf(USART1_TxBuf, "\terrCode: %d Val: %d\r\n", rsltAdvSelfTest,adv_self_test_Value );			
	MainMenu.terminalSend(USART1_TxBuf);
	//ends with soft reset so Initialization again needed
	BMM150_init();*/
    
    /* BMM150 NORMAL self test */				
	if (bmm150_perform_self_test(BMM150_NORMAL_SELF_TEST, &bmm150, &adv_self_test_Value) == BMM150_OK) 	
	MainMenu.terminalSend("\t> NORMAL SELF TEST:\tSUCCESSFULL\r\n");
	else MainMenu.terminalSend("\t> NORMAL SELF TEST:\tF A I L E D.\r\n");
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
    
	/* Power mode status check */
	bmm150_get_op_mode(&sensor_status.bmm150_pwr_mode,&bmm150);				
	sprintf(USART1_TxBuf, "\t> OP Mode status:\t0x%02X", sensor_status.bmm150_pwr_mode );		
	MainMenu.terminalSend(USART1_TxBuf);
	if (sensor_status.bmm150_pwr_mode == 0) MainMenu.terminalSend("\tNormal Mode\r\n");
	else if (sensor_status.bmm150_pwr_mode == 1) MainMenu.terminalSend("\tForced Mode\r\n");
	else if (sensor_status.bmm150_pwr_mode == 3) MainMenu.terminalSend("\tSleep Mode\r\n");
	else MainMenu.terminalSend("\tSuspend Mode\r\n");
		
	/* bmm150 preset mode status */
	sprintf(USART1_TxBuf, "\t> Preset Mode Settings:\t0x%02X", bmm150.settings.preset_mode );		
	MainMenu.terminalSend(USART1_TxBuf);
	if (bmm150.settings.preset_mode == BMM150_PRESETMODE_LOWPOWER) MainMenu.terminalSend("\tLowpower\r\n");
	else if (bmm150.settings.preset_mode == BMM150_PRESETMODE_REGULAR) MainMenu.terminalSend("\tRegular\r\n");
	else if (bmm150.settings.preset_mode == BMM150_PRESETMODE_HIGHACCURACY) MainMenu.terminalSend("\tHighaccuracy\r\n");
	else if (bmm150.settings.preset_mode == BMM150_PRESETMODE_ENHANCED) MainMenu.terminalSend("\tEnhanced\r\n");
	else MainMenu.terminalSend("\tNo preset\r\n");
	
	/* bmm150 settings */		
	bmm150_get_sensor_settings(&bmm150);
	sprintf(USART1_TxBuf, "\t> Z_Repetitions num:\t0x%02X", bmm150.settings.z_rep );		
	MainMenu.terminalSend(USART1_TxBuf);
	sprintf(USART1_TxBuf, "\t%d\r\n", (bmm150.settings.z_rep+1));		
	MainMenu.terminalSend(USART1_TxBuf);
	sprintf(USART1_TxBuf, "\t> XY_Repetitions num:\t0x%02X", bmm150.settings.xy_rep );		
	MainMenu.terminalSend(USART1_TxBuf);
	sprintf(USART1_TxBuf, "\t%d\r\n", (bmm150.settings.xy_rep*2+1));		
	MainMenu.terminalSend(USART1_TxBuf);
	sprintf(USART1_TxBuf, "\t> XYZ Measurement:\t0x%02X", bmm150.settings.xyz_axes_control );		
    MainMenu.terminalSend(USART1_TxBuf);
	if (bmm150.settings.xyz_axes_control == 0) MainMenu.terminalSend("\tEnabled\r\n");
	else MainMenu.terminalSend("\tDisabled\r\n");
	sprintf(USART1_TxBuf, "\t> Data Rate Settings:\t0x%02X", bmm150.settings.data_rate );
	MainMenu.terminalSend(USART1_TxBuf);
    switch (bmm150.settings.data_rate)
    {
        case 0:	MainMenu.terminalSend("\t10 Hz\r\n"); break;
        case 1:	MainMenu.terminalSend("\t2 Hz\r\n");  break;
        case 2:	MainMenu.terminalSend("\t6 Hz\r\n");  break;
        case 3:	MainMenu.terminalSend("\t8 Hz\r\n");  break;
        case 4:	MainMenu.terminalSend("\t15 Hz\r\n"); break;
        case 5: MainMenu.terminalSend("\t20 Hz\r\n"); break;
        case 6:	MainMenu.terminalSend("\t25 Hz\r\n"); break;
        case 7:	MainMenu.terminalSend("\t30 Hz\r\n"); break;
        default: MainMenu.terminalSend("\tUNKNOWN VAL\r\n");break;
    }
	sprintf(USART1_TxBuf, "\t> PWR Mode Settings:\t0x%02X\r\n", bmm150.settings.pwr_mode );		
	MainMenu.terminalSend(USART1_TxBuf);
	sprintf(USART1_TxBuf, "\t> PWR Cntrl bit:\t0x%02X", bmm150.settings.pwr_cntrl_bit );		
	MainMenu.terminalSend(USART1_TxBuf);
	if (bmm150.settings.pwr_cntrl_bit) MainMenu.terminalSend("\tNonsuspend Mode\r\n");
	else MainMenu.terminalSend("\tSuspend Mode\r\n");
	sprintf(USART1_TxBuf, "\t> Data Overrun:\t\t0x%02X", bmm150.settings.int_settings.data_overrun_en );	
	MainMenu.terminalSend(USART1_TxBuf);
	if (bmm150.settings.int_settings.data_overrun_en) MainMenu.terminalSend("\tEnabled\r\n");
	else MainMenu.terminalSend("\tDisabled\r\n");	
	sprintf(USART1_TxBuf, "\t> DRDY Interrupt EN:\t0x%02X", bmm150.settings.int_settings.drdy_pin_en );	
	MainMenu.terminalSend(USART1_TxBuf);
	if (bmm150.settings.int_settings.drdy_pin_en) MainMenu.terminalSend("\tEnabled\r\n");
	else MainMenu.terminalSend("\tDisabled\r\n");
	sprintf(USART1_TxBuf, "\t> DRDY Interrupt Pol:\t0x%02X", bmm150.settings.int_settings.drdy_polarity );	
	MainMenu.terminalSend(USART1_TxBuf);
	if (bmm150.settings.int_settings.drdy_polarity) MainMenu.terminalSend("\tActive High\r\n"); 
	else MainMenu.terminalSend("\tActive Low\r\n"); 
	
    /* Getting data values from sensors */
	bmm150_read_mag_data(&bmm150,&raw);
    
    sprintf(USART1_TxBuf, "\r\n\tTRIM_Z1 value:\t\t%d\r\n", bmm150.trim_data.dig_z1 );
    MainMenu.terminalSend(USART1_TxBuf);
    sprintf(USART1_TxBuf, "\tTRIM_Z2:\t\t%d\r\n", bmm150.trim_data.dig_z2 );
    MainMenu.terminalSend(USART1_TxBuf);
    sprintf(USART1_TxBuf, "\tTRIM_XYZ1:\t\t%d\r\n", bmm150.trim_data.dig_xyz1 );
    MainMenu.terminalSend(USART1_TxBuf);
    
    sprintf(USART1_TxBuf, "\r\n\tR_RAW value:\t\t%d\r\n\n\r", raw.raw_data_r );
    MainMenu.terminalSend(USART1_TxBuf);                                                            
    sprintf(USART1_TxBuf, "\tX_RAW value:\t\t%d\r\n", raw.raw_datax );
    MainMenu.terminalSend(USART1_TxBuf);
    sprintf(USART1_TxBuf, "\tY_RAW value:\t\t%d\r\n", raw.raw_datay );
    MainMenu.terminalSend(USART1_TxBuf);
    sprintf(USART1_TxBuf, "\tZ_RAW value:\t\t%d\r\n", raw.raw_dataz );
    MainMenu.terminalSend(USART1_TxBuf);
    
    if (bmm150.data.x != BMM150_OVERFLOW_OUTPUT)
    {
	sprintf(USART1_TxBuf, "\r\n\tX_Magn value:\t\t%d\r\n", bmm150.data.x );
    MainMenu.terminalSend(USART1_TxBuf);
    }
    else MainMenu.terminalSend("\r\n\tX_Magn value:\t\tOVERFLOW\r\n");
    if (bmm150.data.y != BMM150_OVERFLOW_OUTPUT)
    {
	sprintf(USART1_TxBuf, "\tY_Magn value:\t\t%d\r\n", bmm150.data.y );			
    MainMenu.terminalSend(USART1_TxBuf);	
    }
    else MainMenu.terminalSend("\tY_Magn value:\t\tOVERFLOW\r\n");
    if (bmm150.data.z != BMM150_OVERFLOW_OUTPUT)
    {
	sprintf(USART1_TxBuf, "\tZ_Magn value:\t\t%d\r\n", bmm150.data.z );			
    MainMenu.terminalSend(USART1_TxBuf);	
    }
    else MainMenu.terminalSend("\tZ_Magn value:\t\tOVERFLOW\r\n");
	
            /* ENABLE EXTI0 Interrupt (DRDY) after displaying status */
            DRDY_EnableDisable.EXTI_Line = EXTI_Line0;
            DRDY_EnableDisable.EXTI_LineCmd = ENABLE;
            DRDY_EnableDisable.EXTI_Mode = EXTI_Mode_Interrupt;
            DRDY_EnableDisable.EXTI_Trigger = EXTI_Trigger_Falling;
            EXTI_Init(&DRDY_EnableDisable);
            /*-------------------------------------------------------*/
return BMM150_OK;
}

/**
  * @brief	non-optimized BMM150 data display  
  * @note		
  * @param  id - not used , reg_addr - address of register to write, 
			*data - pointer to data that must be written, len - No. of bytes to write
  * @retval SUCCESS = BMM150_OK
  */
int8_t BMM150_display_data (void)
{
	uint8_t rslt;
	rslt = bmm150_read_mag_data(&bmm150,&raw);
	sprintf(USART1_TxBuf, "\r\n\tX_Magn value:\t\t%d\r\n", bmm150.data.x );			
    MainMenu.terminalSend(USART1_TxBuf);				
	sprintf(USART1_TxBuf, "\tY_Magn value:\t\t%d\r\n", bmm150.data.y );			
    MainMenu.terminalSend(USART1_TxBuf);			
	sprintf(USART1_TxBuf, "\tZ_Magn value:\t\t%d\r\n", bmm150.data.z );			
    MainMenu.terminalSend(USART1_TxBuf);
	return rslt;
}
/**
  * @brief	Reading from BMM150 registers
  *	@note		
  * @param  id - not used , reg_addr - address of register to read, 
			*data - pointer of data that has been read, len - No. of bytes to read
  * @retval SUCCESS = BMM150_OK
  */
static inline int8_t SPI2_Read_BMM150(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{ 
	/******** Optimized realisation *****||********** Analog with functions calls **********/	
	
	SPI2->DR;		//dunno why it's here but it doesnt read MOSI answer byte properly without it
	//drop CSB pin = start conversation
	BMM150_CSB_PORT->BRR = BMM150_CSB_PIN;						//GPIO_ResetBits(SPI_GPIO_PORT,SPI_CSB_PIN);	
	
	SPI2->DR = reg_addr;										//SPI_I2S_SendData(SPI2,reg_addr);
	
	while ((SPI2->SR & SPI_I2S_FLAG_TXE) == (uint16_t)RESET){}	//while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	
	/* NOTE! Skipping this waiting of flag and reading from SPI_RX buffer will make SPI2_Read_BMM150 function 2times faster 
	but it will read false response (first MOSI byte instead of 2nd). May be there's some way to fix it => make func faster. 	
	-------------------------------------------------------------------------------------------------------------------------*/
  while ((SPI2->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);	//while (SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_RXNE) == RESET){}	
	//clear Rx buffer after sending BMI160 reg_adr byte
	SPI2->DR;																	//SPI_I2S_ReceiveData(SPI2);
	/*-----------------------------------------------------------------------------------------------------------------------*/	
	for (register uint16_t i=0; i < len; i++)
	{
	//send CLK for MISO data
	SPI2->DR = 0x00;											//SPI_I2S_SendData(SPI2,0x00);	
	while ((SPI2->SR & SPI_I2S_FLAG_TXE) == (uint16_t)RESET){}	//while (SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE) == RESET){}
 		
	while ((SPI2->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);	//while (SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_RXNE) == RESET){}
		
	//get data from BMM150 reg
	*data = (uint8_t)SPI2->DR;				//*data = (uint8_t)SPI_I2S_ReceiveData(SPI2);
	data++;
	}
	
	//Stop conversation
	BMM150_CSB_PORT->BSRR = BMM150_CSB_PIN;	//GPIO_SetBits(SPI_GPIO_PORT,SPI_CSB_PIN);
	return BMM150_OK; 
}

/**
  * @brief	write to BMM150 register   
  * @note		
  * @param  id - not used , reg_addr - address of register to write, 
			*data - pointer to data that must be written, len - No. of bytes to write
  * @retval SUCCESS = BMM150_OK
  */
static inline int8_t SPI2_Write_BMM150 (uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	uint8_t *ptmp = data;			//get start address of data
	
	for (uint16_t i = 0; i < len; i++)
	{
	GPIO_ResetBits(GPIOA,GPIO_Pin_1);					//drop CSB pin = start conversation
	
	SPI_I2S_SendData(SPI2, reg_addr);  				//send reg address 
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET); // wait TX flag when SPI_DR free
	
	SPI_I2S_SendData(SPI2, *data);  					// send data
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET); // wait TX flag when SPI_DR free
		
	GPIO_SetBits(GPIOA,GPIO_Pin_1);						//up CSB pin = stop conversation
	data++;
	}
	data = ptmp;
	return BMM150_OK;
}


/*========================================================================================
--------------------------------- Sensor common fuctions ---------------------------------
========================================================================================*/


/**
  * @brief   Convert Raw valus from BMI160 for Madgwick filter
  * @param   raw value
  * @retval  cinverted value
 */ 
static inline float convertRawAcceleration(int16_t aRaw) 
{
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

static inline float convertRawGyro(int16_t gRaw) 
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


/**
  * @brief   Dislpays gyro, accel, magni data via choosen interface
  * @param   
  * @retval  None
 */ 
inline void Sensor_display_data (void)
{ 
    sprintf(USART1_TxBuf,"\r%d   \t%d   \t%d \t%d   \t%d   \t%d   \t%d   \t%d   \t%d   ",
        accel.x,accel.y,accel.z,
        gyro.x,gyro.y,gyro.z,raw.raw_datax,
        raw.raw_datay,raw.raw_dataz );
    
    #ifdef DMA_USART1_EN
    USART1_SEND(USART1_TxBuf);
    #else
    USART_SENDoptimized(USART1,USART1_TxBuf);
    #endif
    
    /*
    if (!rsltBMI160)
    {
     //The most non-optimizated realisation 
    			
	MainMenu.terminalSend(USART1_TxBuf);			
	sprintf(USART1_TxBuf, "%d ", gyro.y );			
	MainMenu.terminalSend(USART1_TxBuf);			
	sprintf(USART1_TxBuf, "%d \r\n", gyro.z );
    
	MainMenu.terminalSend(USART1_TxBuf);			
	sprintf(USART1_TxBuf, "\tAccel value: %d ", accel.x );			
	MainMenu.terminalSend(USART1_TxBuf);			
	sprintf(USART1_TxBuf, "%d ", accel.y );			
	MainMenu.terminalSend(USART1_TxBuf);			
	sprintf(USART1_TxBuf, "%d\r\n", accel.z );			
	MainMenu.terminalSend(USART1_TxBuf);
    }
    
    if (!rsltBMM150)
    {
    sprintf(USART1_TxBuf, "\tMegni value: %d ", raw.raw_datax );			
	MainMenu.terminalSend(USART1_TxBuf);			
	sprintf(USART1_TxBuf, "%d ", raw.raw_datay );			
	MainMenu.terminalSend(USART1_TxBuf);			
	sprintf(USART1_TxBuf, "%d\r\n", raw.raw_dataz );			
	MainMenu.terminalSend(USART1_TxBuf);
    }
	*/
}

/**
  * @brief  create buffer with sensor data to send via USART
  * @param  *pbuf - pointer at 2xDimention buf to store strings
  * @retval None
  */
void createSensorDataBuffer ( char **pbuf[][9],
                    struct bmi160_sensor_data const *accel_data,
                    struct bmi160_sensor_data const *gyro_data, 
                    struct bmm150_raw_mag_data const *mag_data )
{
    *pbuf[0][0] = "Ax ";
    *pbuf[0][1] = "Ay ";
    *pbuf[0][2] = "Az ";
    *pbuf[0][3] = "Ax ";
    *pbuf[0][4] = "Ay ";
    *pbuf[0][5] = "Az ";
    *pbuf[0][6] = "Ax ";
    *pbuf[0][7] = "Ay ";
    *pbuf[0][8] = "Az\n\r"; 
}

/**
  * @brief  common function to read All sensor data and send to display
  * @param  
  * @retval None
  */
inline void Read_Display_SensorData (USART_TypeDef* USARTx, struct bmm150_dev *dev)
{
    /* BMI160 local variables */
	uint8_t idx = 0;
	uint8_t data_array[15] = {0};
	uint8_t lsb;
	uint8_t msb;
	int16_t msblsb;
    int16_t Gx;
    int16_t Gy;
    int16_t Gz;
    int16_t Ax;
    int16_t Ay;
    int16_t Az;     
    /* BMM150 local variables */
    int16_t msb_data;
	uint8_t reg_data[BMM150_XYZR_DATA_LEN] = {0};
    int16_t Mraw_x;
    int16_t Mraw_y;
    int16_t Mraw_z;   
    
	/* reading both accel and gyro bmi160 data */
    SPI2_Read_BMI160(1, BMI160_GYRO_DATA_ADDR | BMI160_SPI_RD_MASK, data_array, 12);
	//rsltBMI160  = bmi160_get_regs(BMI160_GYRO_DATA_ADDR, data_array, 12 + len, dev);

    // Gyro Data 
    lsb = data_array[idx++];
    msb = data_array[idx++];
    msblsb = (int16_t)((msb << 8) | lsb);
    Gx = msblsb; // gyro X axis data 

    lsb = data_array[idx++];
    msb = data_array[idx++];
    msblsb = (int16_t)((msb << 8) | lsb);
    Gy = msblsb; // gyro Y axis data 

    lsb = data_array[idx++];
    msb = data_array[idx++];
    msblsb = (int16_t)((msb << 8) | lsb);
    Gz = msblsb; // gyro Z axis data 

    // Accel Data 
    lsb = data_array[idx++];
    msb = data_array[idx++];
    msblsb = (int16_t)((msb << 8) | lsb);
    Ax = (int16_t)msblsb; // accel X axis data 

    lsb = data_array[idx++];
    msb = data_array[idx++];
    msblsb = (int16_t)((msb << 8) | lsb);
    Ay = (int16_t)msblsb; // accel Y axis data 

    lsb = data_array[idx++];
    msb = data_array[idx++];
    msblsb = (int16_t)((msb << 8) | lsb);
    Az = (int16_t)msblsb; // accel Z axis data    
    
    
    /* -=== Reading BMM150 data ===- */

    /* activate forced mode ar first to read up at >300 Hz  */
    dev->settings.pwr_mode = BMM150_FORCED_MODE;
    bmm150_set_op_mode(dev); 
                       
    /* Get Magni data */
    /*Read the mag data registers */
    //bmm150_get_regs(BMM150_DATA_X_LSB, reg_data, BMM150_XYZR_DATA_LEN, dev);
    SPI2_Read_BMM150(1,BMM150_DATA_X_LSB | 0x80, reg_data,BMM150_XYZR_DATA_LEN);
    
    /* Mag X axis data */
    reg_data[0] = BMM150_GET_BITS(reg_data[0], BMM150_DATA_X);
    /* Shift the MSB data to left by 5 bits */
    /* Multiply by 32 to get the shift left by 5 value */
    msb_data = ((int16_t)((int8_t)reg_data[1]))<< 5;  // * 32;  
    Mraw_x = (int16_t)(msb_data | reg_data[0]);  // Raw mag X axis data
    
    /* Mag Y axis data */
    reg_data[2] = BMM150_GET_BITS(reg_data[2], BMM150_DATA_Y);
    /* Shift the MSB data to left by 5 bits */
    /* Multiply by 32 to get the shift left by 5 value */
    msb_data = ((int16_t)((int8_t)reg_data[3]))<< 5; //* 32; 
    Mraw_y = (int16_t)(msb_data | reg_data[2]); // Raw mag Y axis data 
    
    /* Mag Z axis data */
    reg_data[4] = BMM150_GET_BITS(reg_data[4], BMM150_DATA_Z);
    /* Shift the MSB data to left by 7 bits */
    /* Multiply by 128 to get the shift left by 7 value */
    msb_data = ((int16_t)((int8_t)reg_data[5]))<< 7;// * 128;   
    Mraw_z = (int16_t)(msb_data | reg_data[4]); // Raw mag Z axis data
    
            #ifdef COMPENSATE_MAGNI
            int16_t Mraw_R;
            /* Mag R-HALL data */
            reg_data[6] = BMM150_GET_BITS(reg_data[6], BMM150_DATA_RHALL);
            Mraw_R = (uint16_t)(((uint16_t)reg_data[7] << 6) | reg_data[6]);
            /* Compensated Mag X data in int16_t format */
            Mraw_x = compensate_x(Mraw_x, Mraw_R, dev);
            /* Compensated Mag Y data in int16_t format */
            Mraw_y = compensate_y(Mraw_y, Mraw_R, dev);
            /* Compensated Mag Z data in int16_t format */
            Mraw_z = compensate_z(Mraw_z, Mraw_R, dev);
            #endif
    
    /* Send obtained data via USART */
    sprintf(USART1_TxBuf,"\r\n%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d",
        Gx,Gy,Gz,               //gyro data
        Ax,Ay,Az,               //accel data
        Mraw_x,Mraw_y,Mraw_z ); //magni raw data
 
    /* Choosing stream and way of sending data */   
    if (USARTx == USART1)
    {    
        #ifdef DMA_USART1_EN
        USART1_SEND_DMA(USART1_TxBuf);
        #else
        USART_SENDoptimized(USARTx,USART1_TxBuf);
        #endif   
    }
    else    //USARTx == USART2
    {
        #ifdef DMA_USART2_EN
        USART2_SEND_DMA(USART1_TxBuf);
        #else
        USART_SENDoptimized(USARTx,USART1_TxBuf);
        #endif
    }
    
}




