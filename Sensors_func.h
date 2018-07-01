/*==============================================================================
>File name:  	Sensors_func.h
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
struct Sensors_common_status
{
uint8_t err_status;
uint8_t gyro_config_reg;
uint8_t gyro_range_reg;
uint8_t accel_config_reg;
uint8_t accel_range_reg;	
uint8_t pwr_status;	
uint8_t comm_status; 
uint8_t temperature1; 			
uint8_t temperature2; 	
uint8_t id;	
uint8_t	INT_status;
uint8_t accel_pwr_mode;
uint8_t gyro_pwr_mode;
uint8_t bmm150_pwr_mode;
uint8_t accel_cfg_range;
uint8_t accel_cfg_bw;
uint8_t accel_cfg_odr;
uint8_t gyro_cfg_range;
uint8_t gyro_cfg_bw;
uint8_t gyro_cfg_odr;
uint8_t bmm150_data_rate;
uint8_t bmm150_xy_rep;
uint8_t bmm150_z_rep;
uint8_t bmm150_preset_mode;
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
extern inline void BMI160_display_data (void);
int8_t BMI160_display_status (void);
int8_t BMI160_INT_config (void);
extern inline void BMI160_OptGet_data (struct bmi160_sensor_data *accel, struct bmi160_sensor_data *gyro);
extern inline void BMI160_OptGet_data2 (char *buf);
extern inline void BMI160_OptGet_FilteredData (filtered_data *fdata);
extern inline int8_t BMI160_set_Mode(struct bmi160_dev *dev, const uint8_t accel_mode, const uint8_t gyro_mode );
extern inline void BMI160_setGyro_Mode(struct bmi160_dev *dev, const uint8_t gyro_mode);
extern inline void BMI160_setAccel_Mode(struct bmi160_dev *dev, const uint8_t accel_mode);

int8_t BMM150_init (void);
int8_t BMM150_display_data (void);
int8_t BMM150_display_status (void);
extern inline void BMM150_OptGet_data(struct bmm150_dev *dev,struct bmm150_raw_mag_data *raw_mag_data);
extern inline int8_t BMM150_set_Mode(struct bmm150_dev *dev, const uint8_t mode);
extern inline int8_t BMM150_set_Preset(struct bmm150_dev *dev, const uint8_t preset);
int8_t BMM150_DRDY_config(struct bmm150_dev *dev);

extern inline void Read_Display_SensorData (USART_TypeDef* USARTx, struct bmm150_dev *dev);
extern inline void Sensor_display_data (void);
float convertRawAcceleration(int16_t aRaw);
float convertRawGyro(int16_t gRaw);
_Bool filter_data(filtered_data *fdata, struct bmi160_sensor_data *sens1, struct bmi160_sensor_data *sens2);

static inline int8_t SPI2_Read_BMI160(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);
static inline int8_t SPI2_Write_BMI160 (uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);
static inline int8_t SPI2_Read_BMM150(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);
static inline int8_t SPI2_Write_BMM150 (uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);


#endif /* BMI160_FUNC_H_ */
