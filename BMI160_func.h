/*==============================================================================
>File name:  	BMI160_func.h
>Brief:         Header for BMI160,BMM150 and Filters API	
>Author:        LL
>Date:          07.05.2018
>Version:       1.0
==============================================================================*/

#ifndef BMI160_FUNC_H_
#define BMI160_FUNC_H_

#include "Main.h"

#include "stm32f10x_Inits.h"
#include "Menu.h"

#include "bmm150.h"						//BMM150 sensor driver
#include "bmi160.h"             		//BMI160 sensor driver
#include "stm32_delay_asm.h"


#ifdef Madgwick
	#include "MadgwickAHRS.h"
#else 
	#include "MahonyAHRS.h"
#endif

//---------------------------= 101[V]ariables =---------------------------------
struct bmi160_common_status
{
uint8_t err_status;
uint8_t pwr_status;	
uint8_t comm_status; 
uint8_t temperature1; 			
uint8_t temperature2; 	
uint8_t id;	
};

typedef struct 
{
float ax;
float ay;
float az;
float gx;
float gy;
float gz;
float mx; 
float my; 
float mz;
float roll;	
float pitch;
float heading;	
}filtered_data;

//----------------------------= [P]rototypes ; =--------------------------------
int8_t BMI160_init (void);
int8_t BMI160_display_data (void);
int8_t BMI160_display_status (void);
int8_t BMI160_INT_config (void);
int8_t BMI160_OptGet_data (struct bmi160_sensor_data *accel, struct bmi160_sensor_data *gyro);
int8_t BMI160_sendto_HC05 (void);

float convertRawAcceleration(int16_t aRaw);
float convertRawGyro(int16_t gRaw);
_Bool filter_data(filtered_data *fdata, struct bmi160_sensor_data *sens1, struct bmi160_sensor_data *sens2);

int8_t user_aux_read(uint8_t id, uint8_t reg_addr, uint8_t *aux_data, uint16_t len);
int8_t user_aux_write(uint8_t id, uint8_t reg_addr, uint8_t *aux_data, uint16_t len);

int8_t SPI2_Read_BMM150(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t SPI2_Write_BMM150 (uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t BMM150_init (void);

#endif /* BMI160_FUNC_H_ */
