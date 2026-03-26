/*
 ****************************************************************************************
 * @file			sht31.h
 * @author			Gradiren_5
 * @brief			STM32 HAL driver header for the SHT3x temperature and humidity sensor.
 *****************************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef SHT31_H
#define SHT31_H


#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"





/* SHT3x structure definition -----------------------------------------------*/

typedef struct {

	I2C_HandleTypeDef* i2c;
	uint8_t Dev_Addr;

	uint16_t StatusReg;

	uint16_t Temp_Raw;						/*Raw Temperature data from sensor*/
	uint16_t Humid_Raw;						/*Raw Humidity data from sensor*/

	float Temp;								/*Calculated final Temperature value from sensor in deg Celsius */
	float Humid;							/*Calculated final Humidity value from sensor in %RH */

	uint8_t crc_Temp;
	uint8_t crc_Humid;
	uint8_t Temp_valid;
	uint8_t Humid_valid;

	struct {

		uint16_t high_set_raw;
		uint16_t high_clear_raw;
		uint16_t low_clear_raw;
		uint16_t low_set_raw;


		float Temp_High_Set;
		float Temp_High_Clear;
		float Temp_Low_Clear;
		float Temp_Low_Set;


		float Hum_High_Set;
		float Hum_High_Clear;
		float Hum_Low_Clear;
		float Hum_Low_Set;



	}Alert;


	} SHT3x_HandleTypeDef;

/* Enums for measurement settings -------------------------------------------- */


	typedef enum {

		SHT3x_ADDR_DEF = 	0x44,			/*Device address when ADDR (Pin 2) connected to logic Low*/
		SHT3x_ADDR_ALT = 	0x45			/*Device address when ADDR (Pin 2) connected to logic High*/

	} SHT3x_DevAddr_t;


	typedef enum {

		SHT3x_CLK_STR_DISABLE = 0, 			/*Clock stretching disabled (single-shot mode only) */
	    SHT3x_CLK_STR_ENABLE				/*Clock stretching disabled (single-shot mode only) */

	} SHT3x_ClockStr_t;


	typedef enum {

		SHT3x_REPEAT_LOW = 0,				/*Low repeatability measurement */
	    SHT3x_REPEAT_MEDIUM,				/*Medium repeatability measurement*/
	    SHT3x_REPEAT_HIGH					/*High repeatability measurement */

	} SHT3x_Repeat_t;

	typedef enum {

		SHT3x_RATE_0_5_HZ = 0,				/*Periodic mode: 0.5 measurements per second */
	    SHT3x_RATE_1_HZ,					/*Periodic mode: 1 measurement per second */
	    SHT3x_RATE_2_HZ,					/*Periodic mode: 2 measurements per second */
	    SHT3x_RATE_4_HZ,					/*Periodic mode: 4 measurements per second */
	    SHT3x_RATE_10_HZ					/*Periodic mode: 10 measurements per second */

	} SHT3x_PerRate_t;


/* Initialization and measurement API ---------------------------------------- */

uint8_t SHT3x_Init (SHT3x_HandleTypeDef* sht, I2C_HandleTypeDef* i2c, SHT3x_DevAddr_t dev_addr); 				/*Initializes the sensor handle, resets the sensor, and clears the status register */

uint8_t SHT3x_Meas_Single_Shot (SHT3x_HandleTypeDef* sht, SHT3x_Repeat_t rep, SHT3x_ClockStr_t cl_str);			/*Performs a single-shot measurement with selected repeatability and clock stretching */

uint8_t SHT3x_Meas_Periodic_Start (SHT3x_HandleTypeDef* sht, SHT3x_Repeat_t rep, SHT3x_PerRate_t rate);			/*Starts periodic measurements with selected repeatability and measurement rate */

uint8_t SHT3x_Meas_Periodic_Fetch (SHT3x_HandleTypeDef* sht);													/*Reads the latest temperature and humidity data in periodic mode; returns 0 if CRC check fails */

uint8_t SHT3x_Meas_Periodic_Break	(SHT3x_HandleTypeDef* sht);													/*Stops periodic measurements and returns the sensor to idle mode */




/* Device control and status API --------------------------------------------- */

uint8_t SHT3x_Reset(SHT3x_HandleTypeDef* sht);					/*Sensor software reset*/

uint8_t SHT3x_Read_Status (SHT3x_HandleTypeDef* sht);			/*Reads the status register and updates the handle field */

uint8_t SHT3x_Clear_Status (SHT3x_HandleTypeDef* sht);			/*Clears the status register */

uint8_t SHT3x_Heater_On (SHT3x_HandleTypeDef* sht);				/*Enables the internal heater */

uint8_t SHT3x_Heater_Off (SHT3x_HandleTypeDef* sht);			/*Disables the internal heater */


/* Alert managment API - periodic mode only ------------------------------------------------------------------------------------*/

uint8_t SHT3x_Alrt_ReadSettings (SHT3x_HandleTypeDef* sht);																					/* Reads all current alert limit settings from the sensor */

uint8_t SHT3x_Alrt_SetTemp (SHT3x_HandleTypeDef* sht, float T_High_Set, float T_High_Clear, float T_Low_Clear, float T_Low_Set);			/* Updates temperature alert limits, keeps current humidity alert limits */

uint8_t SHT3x_Alrt_SetHumid (SHT3x_HandleTypeDef* sht, float H_High_Set, float H_High_Clear, float H_Low_Clear, float H_Low_Set);			/* Updates humidity alert limits, keeps current humidity alert limits */



#ifdef __cplusplus
}
#endif


#endif /* SHT31_H */
