/*
 ****************************************************************************************
 * @file			sht31.c
 * @author			Gradiren_5
 * @brief			STM32 HAL driver source for the SHT3x temperature and humidity sensor.
 *****************************************************************************************/


#include "sht31.h"

/* Private defines---------------------------------------------------------------------------------------------------*/

#define S31_CMD_READ_STATUS				0xF32D				/* Read the 16 bit Status Register*/
#define S31_CMD_RESET					0x30A2				/* Software reset */
#define S31_CMD_CLEAR_STATUS			0x3041				/* Clears all Status Register bits  */

#define S31_CMD_FETCH_DATA				0xE000				/* Read Temperature and Humidity during Periodic measurment*/

#define S31_CMD_BREAK					0x3093				/* Stops the Periodic measurment. Sensor returns to SS mode*/

#define S31_CMD_HEATER_ON				0x306D				/* Turns heater ON (if iits not on already)*/
#define S31_CMD_HEATER_OFF				0x3066				/* Turns heater OFF (if iits not off already)*/


#define S31_CMD_SS_MES_NOSTR_HIGH		0x2400				/* Single Shot measurement in NO CLOCK STREACH mode, high repeatablity*/
#define S31_CMD_SS_MES_NOSTR_MED		0x240B				/* Single Shot measurement in NO CLOCK STREACH mode, medium repeatablity*/
#define S31_CMD_SS_MES_NOSTR_LOW		0x2416				/* Single Shot measurement in NO CLOCK STREACH mode, low repeatablity*/
#define S31_CMD_SS_MES_STR_HIGH			0x2C06				/* Single Shot measurement in CLOCK STREACH mode, high repeatablity*/
#define S31_CMD_SS_MES_STR_MED			0x2C0D				/* Single Shot measurement in CLOCK STREACH mode, medium repeatablity*/
#define S31_CMD_SS_MES_STR_LOW			0x2C10				/* Single Shot measurement in CLOCK STREACH mode, low repeatablity*/

#define S31_CMD_PER_MES_05_HIGH			0x2032				/* Start periodic measurement, 0.5 mps, high repeatablity*/
#define S31_CMD_PER_MES_05_MED			0x2024				/* Start periodic measurement, 0.5 mps, medium repeatablity*/
#define S31_CMD_PER_MES_05_LOW			0x202F				/* Start periodic measurement, 0.5 mps, low repeatablity*/
#define S31_CMD_PER_MES_1_HIGH			0x2130				/* Start periodic measurement, 1 mps, high repeatablity*/
#define S31_CMD_PER_MES_1_MED			0x2126				/* Start periodic measurement, 1 mps, medium repeatablity*/
#define S31_CMD_PER_MES_1_LOW			0x212D				/* Start periodic measurement, 1 mps, low repeatablity*/
#define S31_CMD_PER_MES_2_HIGH			0x2236				/* Start periodic measurement, 2 mps, high repeatablity*/
#define S31_CMD_PER_MES_2_MED			0x2220				/* Start periodic measurement, 2 mps, medium repeatablity*/
#define S31_CMD_PER_MES_2_LOW			0x222B				/* Start periodic measurement, 2 mps, low repeatablity*/
#define S31_CMD_PER_MES_4_HIGH			0x2334				/* Start periodic measurement, 4 mps, high repeatablity*/
#define S31_CMD_PER_MES_4_MED			0x2322				/* Start periodic measurement, 4 mps, medium repeatablity*/
#define S31_CMD_PER_MES_4_LOW			0x2329				/* Start periodic measurement, 4 mps, low repeatablity*/
#define S31_CMD_PER_MES_10_HIGH			0x2737				/* Start periodic measurement, 10 mps, high repeatablity*/
#define S31_CMD_PER_MES_10_MED			0x2721				/* Start periodic measurement, 10 mps, medium repeatablity*/
#define S31_CMD_PER_MES_10_LOW			0x272A				/* Start periodic measurement, 10 mps, low repeatablity*/


#define S31_ALRT_READ_HIGH_SET			0xE11F
#define S31_ALRT_READ_HIGH_CLEAR		0xE114
#define S31_ALRT_READ_LOW_SET			0xE109
#define S31_ALRT_READ_LOW_CLEAR			0xE102

#define S31_ALRT_WRITE_HIGH_SET			0x611D
#define S31_ALRT_WRITE_HIGH_CLEAR		0x6116
#define S31_ALRT_WRITE_LOW_SET			0x610B
#define S31_ALRT_WRITE_LOW_CLEAR		0x6100

/*Private functions prototypes---------------------------------------------------------------------------------------*/

static uint16_t SHT3x_Get_SS_Command(SHT3x_Repeat_t rep, SHT3x_ClockStr_t cl_str);

static uint16_t SHT3x_Get_Periodic_Command (SHT3x_Repeat_t rep, SHT3x_PerRate_t rate);

static void SHT3x_Cmd_Devide(uint16_t cmd, uint8_t* cmd_arr);

static void SHT3x_Calc_Temp_Hum (SHT3x_HandleTypeDef* sht, uint8_t* data_arr);

static uint8_t SHT3x_Temp_CRC (SHT3x_HandleTypeDef* sht);

static uint8_t SHT3x_Humid_CRC (SHT3x_HandleTypeDef* sht);

static uint8_t SHT3x_Status_CRC (uint8_t* reg_data);

static uint8_t SHT3x_CRCCalc (uint8_t* data);


/********************************************************************************************************/
/*  Public functions------------------------------------------------------------------------------------*/
/********************************************************************************************************/



uint8_t SHT3x_Init (SHT3x_HandleTypeDef* sht, I2C_HandleTypeDef* i2c, SHT3x_DevAddr_t dev_addr) {

	sht->i2c = i2c;
	sht->Dev_Addr = dev_addr;

	if (SHT3x_Reset(sht) == 0) return 0;

	if (SHT3x_Clear_Status(sht) == 0) return 0;

	HAL_Delay(1);

	return 1;

}


uint8_t SHT3x_Meas_Single_Shot (SHT3x_HandleTypeDef* sht, SHT3x_Repeat_t rep, SHT3x_ClockStr_t cl_str) {

		uint16_t cmd = 0;

		uint8_t cmd_arr[2] = {0};
		uint8_t data_arr[6] = {0};

		cmd = SHT3x_Get_SS_Command (rep, cl_str);

		if (cmd == 0) return 0;

		SHT3x_Cmd_Devide(cmd, cmd_arr);

		if (HAL_I2C_Master_Transmit(sht->i2c, (sht->Dev_Addr << 1), cmd_arr, 2, 100) != HAL_OK) return 0;

		if (cmd == S31_CMD_SS_MES_NOSTR_HIGH) HAL_Delay(15);
		else if (cmd == S31_CMD_SS_MES_NOSTR_MED) HAL_Delay(6);
		else if (cmd == S31_CMD_SS_MES_NOSTR_LOW) HAL_Delay(4);
		else if (cmd == S31_CMD_SS_MES_STR_HIGH ||
			cmd == S31_CMD_SS_MES_STR_MED ||
			cmd == S31_CMD_SS_MES_STR_LOW) HAL_Delay(1);
		else return 0;

		if (HAL_I2C_Master_Receive(sht->i2c, (sht->Dev_Addr << 1), data_arr, 6, HAL_MAX_DELAY) != HAL_OK) return 0;

		SHT3x_Calc_Temp_Hum (sht, data_arr);

		sht->Temp_valid = SHT3x_Temp_CRC(sht);
		sht->Humid_valid = SHT3x_Humid_CRC(sht);

		if ((sht->Temp_valid == 0) || (sht->Humid_valid == 0)) return 0;

		return 1;

}


uint8_t SHT3x_Meas_Periodic_Start (SHT3x_HandleTypeDef* sht, SHT3x_Repeat_t rep, SHT3x_PerRate_t rate) {


		uint16_t cmd = 0;
		uint8_t cmd_arr[2] = {0};

		cmd = SHT3x_Get_Periodic_Command (rep, rate);

		if (cmd == 0) return 0;

		SHT3x_Cmd_Devide(cmd, cmd_arr);

		if (HAL_I2C_Master_Transmit(sht->i2c, (sht->Dev_Addr << 1), cmd_arr, 2, HAL_MAX_DELAY) != HAL_OK) return 0;

		HAL_Delay(1);

		return 1;

}

uint8_t SHT3x_Meas_Periodic_Fetch (SHT3x_HandleTypeDef* sht) {

		uint8_t cmd_arr[2] = {0};
		uint8_t data_arr[6] = {0};


		SHT3x_Cmd_Devide(S31_CMD_FETCH_DATA, cmd_arr);

		if (HAL_I2C_Master_Transmit(sht->i2c, (sht->Dev_Addr << 1), cmd_arr, 2, 100) != HAL_OK) return 0;
		if (HAL_I2C_Master_Receive(sht->i2c, (sht->Dev_Addr << 1), data_arr, 6, 100) != HAL_OK) return 0;

		SHT3x_Calc_Temp_Hum (sht, data_arr);

		sht->Temp_valid = SHT3x_Temp_CRC(sht);
		sht->Humid_valid = SHT3x_Humid_CRC(sht);

		if ((sht->Temp_valid == 0) || (sht->Humid_valid == 0)) return 0;

		return 1;
}

uint8_t SHT3x_Meas_Periodic_Break	(SHT3x_HandleTypeDef* sht) {

		uint8_t cmd_arr[2] = {0};

		SHT3x_Cmd_Devide(S31_CMD_BREAK, cmd_arr);

		if (HAL_I2C_Master_Transmit(sht->i2c, (sht->Dev_Addr << 1), cmd_arr, 2, HAL_MAX_DELAY) != HAL_OK) return 0;

		HAL_Delay(1);

		return 1;

}

uint8_t SHT3x_Reset (SHT3x_HandleTypeDef* sht) {

	uint8_t cmd_arr[2] = {0};

	SHT3x_Cmd_Devide(S31_CMD_RESET, cmd_arr);

	if (HAL_I2C_Master_Transmit(sht->i2c, (sht->Dev_Addr << 1), cmd_arr, 2, HAL_MAX_DELAY) != HAL_OK) return 0;

	HAL_Delay(2);

	return 1;

};




uint8_t SHT3x_Read_Status (SHT3x_HandleTypeDef* sht) {

	uint8_t cmd_arr[2] = {0};
	uint8_t reg_data[3] = {0};

	SHT3x_Cmd_Devide(S31_CMD_READ_STATUS, cmd_arr);

    if (HAL_I2C_Master_Transmit(sht->i2c, (sht->Dev_Addr << 1), cmd_arr, 2, HAL_MAX_DELAY) != HAL_OK) return 0;
  	if (HAL_I2C_Master_Receive(sht->i2c, (sht->Dev_Addr << 1), reg_data, 3, HAL_MAX_DELAY) != HAL_OK) return 0;

  	if (SHT3x_Status_CRC(reg_data) == 1) sht->StatusReg = reg_data[0]<<8 | reg_data[1];
  	else return 0;

  	HAL_Delay(1);

  	return 1;
}



uint8_t SHT3x_Clear_Status (SHT3x_HandleTypeDef* sht) {

	uint8_t cmd_arr[2] = {0};

	SHT3x_Cmd_Devide(S31_CMD_CLEAR_STATUS, cmd_arr);

	if (HAL_I2C_Master_Transmit(sht->i2c, (sht->Dev_Addr << 1), cmd_arr, 2, HAL_MAX_DELAY) != HAL_OK) return 0;

	HAL_Delay(1);

	return 1;

}

uint8_t SHT3x_Heater_On (SHT3x_HandleTypeDef* sht) {

		uint8_t cmd_arr[2] = {0};

		SHT3x_Read_Status (sht);

		if ((sht->StatusReg & 0x2000) == 0x2000) return 0;

		SHT3x_Cmd_Devide(S31_CMD_HEATER_ON, cmd_arr);
		if (HAL_I2C_Master_Transmit(sht->i2c, (sht->Dev_Addr << 1), cmd_arr, 2, HAL_MAX_DELAY) != HAL_OK) return 0;

		HAL_Delay(1);

		return 1;
}


uint8_t SHT3x_Heater_Off (SHT3x_HandleTypeDef* sht) {

		uint8_t cmd_arr[2] = {0};

		SHT3x_Read_Status (sht);

		if ((sht->StatusReg & 0x2000) == 0x0) return 0;

		SHT3x_Cmd_Devide(S31_CMD_HEATER_OFF, cmd_arr);
		if (HAL_I2C_Master_Transmit(sht->i2c, (sht->Dev_Addr << 1), cmd_arr, 2, HAL_MAX_DELAY)!= HAL_OK) return 0;

		HAL_Delay(1);

		return 1;
}



uint8_t SHT3x_Alrt_ReadSettings (SHT3x_HandleTypeDef* sht) {

	uint8_t cmd_arr[2] = {0};
	uint8_t data_arr[3] = {0};

	uint16_t temp_T = 0, temp_H = 0;

	/*Read and unpack High-Set values*/

	SHT3x_Cmd_Devide(S31_ALRT_READ_HIGH_SET, cmd_arr);
	if (HAL_I2C_Master_Transmit(sht->i2c, (sht->Dev_Addr << 1), cmd_arr, 2, HAL_MAX_DELAY)!= HAL_OK) return 0;
  	if (HAL_I2C_Master_Receive(sht->i2c, (sht->Dev_Addr << 1), data_arr, 3, HAL_MAX_DELAY)!= HAL_OK) return 0;

  	if (SHT3x_Status_CRC(data_arr) == 1) sht->Alert.high_set_raw = (data_arr[0] << 8) | data_arr[1];
  	else return 0;

  	temp_H = ((uint16_t)data_arr[0] << 8) & 0b1111111000000000;
  	temp_T = (((uint16_t)data_arr[0] << 15) | ((uint16_t)data_arr[1] << 7)) & 0b1111111110000000;

  	sht->Alert.Hum_High_Set = 100.0f * (float)temp_H / 65535.0f;
	sht->Alert.Temp_High_Set = -45.0f + 175.0f * (float)temp_T / 65535.0f;


	/*Read and unpack High-Clear values*/

	SHT3x_Cmd_Devide(S31_ALRT_READ_HIGH_CLEAR, cmd_arr);
	if (HAL_I2C_Master_Transmit(sht->i2c, (sht->Dev_Addr << 1), cmd_arr, 2, HAL_MAX_DELAY)!= HAL_OK) return 0;
  	if (HAL_I2C_Master_Receive(sht->i2c, (sht->Dev_Addr << 1), data_arr, 3, HAL_MAX_DELAY)!= HAL_OK) return 0;

  	if (SHT3x_Status_CRC(data_arr) == 1) sht->Alert.high_clear_raw = (data_arr[0] << 8) | data_arr[1];
  	else return 0;

  	temp_H = ((uint16_t)data_arr[0] << 8) & 0b1111111000000000;
  	temp_T = (((uint16_t)data_arr[0] << 15) | ((uint16_t)data_arr[1] << 7)) & 0b1111111110000000;

  	sht->Alert.Hum_High_Clear = 100.0f * (float)temp_H / 65535.0f;
	sht->Alert.Temp_High_Clear = -45.0f + 175.0f * (float)temp_T / 65535.0f;


	/*Read and unpack Low-Set values*/

	SHT3x_Cmd_Devide(S31_ALRT_READ_LOW_SET, cmd_arr);
	if (HAL_I2C_Master_Transmit(sht->i2c, (sht->Dev_Addr << 1), cmd_arr, 2, HAL_MAX_DELAY)!= HAL_OK) return 0;
  	if (HAL_I2C_Master_Receive(sht->i2c, (sht->Dev_Addr << 1), data_arr, 3, HAL_MAX_DELAY)!= HAL_OK) return 0;

  	if (SHT3x_Status_CRC(data_arr) == 1) sht->Alert.low_set_raw = (data_arr[0] << 8) | data_arr[1];
  	else return 0;

  	temp_H = ((uint16_t)data_arr[0] << 8) & 0b1111111000000000;
  	temp_T = (((uint16_t)data_arr[0] << 15) | ((uint16_t)data_arr[1] << 7)) & 0b1111111110000000;

  	sht->Alert.Hum_Low_Set = 100.0f * (float)temp_H / 65535.0f;
	sht->Alert.Temp_Low_Set = -45.0f + 175.0f * (float)temp_T / 65535.0f;


	/*Read and unpack Low-Clear values*/

	SHT3x_Cmd_Devide(S31_ALRT_READ_LOW_CLEAR, cmd_arr);
	if (HAL_I2C_Master_Transmit(sht->i2c, (sht->Dev_Addr << 1), cmd_arr, 2, HAL_MAX_DELAY)!= HAL_OK) return 0;
  	if (HAL_I2C_Master_Receive(sht->i2c, (sht->Dev_Addr << 1), data_arr, 3, HAL_MAX_DELAY)!= HAL_OK) return 0;

  	if (SHT3x_Status_CRC(data_arr) == 1) sht->Alert.low_clear_raw = (data_arr[0] << 8) | data_arr[1];
  	else return 0;

  	temp_H = ((uint16_t)data_arr[0] << 8) & 0b1111111000000000;
  	temp_T = (((uint16_t)data_arr[0] << 15) | ((uint16_t)data_arr[1] << 7)) & 0b1111111110000000;

  	sht->Alert.Hum_Low_Clear = 100.0f * (float)temp_H / 65535.0f;
	sht->Alert.Temp_Low_Clear = -45.0f + 175.0f * (float)temp_T / 65535.0f;

  	return 1;
}


uint8_t SHT3x_Alrt_SetTemp (SHT3x_HandleTypeDef* sht, float T_High_Set, float T_High_Clear, float T_Low_Clear, float T_Low_Set) {

	uint8_t cmd_arr[2] = {0};
	uint8_t data_arr[3] = {0};
	uint8_t out_arr[5] = {0};

	uint16_t temp_T = 0;
	uint16_t temp_Setting = 0;

	if ((T_High_Set > 130) || (T_High_Clear > T_High_Set) || (T_Low_Clear > T_High_Clear) || (T_Low_Set > T_Low_Clear) || (T_Low_Set < -50)) return 0;

	if (SHT3x_Alrt_ReadSettings (sht) == 0) return 0;


	/* Calculate and pack High-Set limit: update temperature, keep previous humidity */

	temp_Setting = sht->Alert.high_set_raw;

	temp_T = (uint16_t)(((T_High_Set + 45.0f) / 175.0f * 65535.0f) + 64.0f);      	// Convert temperature from °C to 16-bit raw format. Add 64 to round before dropping 7 LSBs (64 = half of 128).
	temp_T = (temp_T & 0b1111111110000000) >> 7;

	temp_Setting = temp_Setting & 0b1111111000000000;							 	// Clear previous temperature bits, keep humidity bits

	sht->Alert.high_set_raw = temp_T | temp_Setting;								// Build new packed alert limit value

	data_arr[0] = (uint8_t)(sht->Alert.high_set_raw >> 8);
	data_arr[1] = (uint8_t)(sht->Alert.high_set_raw & 0x00FF);
	data_arr[2] = SHT3x_CRCCalc (data_arr);

	SHT3x_Cmd_Devide(S31_ALRT_WRITE_HIGH_SET, cmd_arr);

	out_arr[0] = cmd_arr[0];
	out_arr[1] = cmd_arr[1];
	out_arr[2] = data_arr[0];
	out_arr[3] = data_arr[1];
	out_arr[4] = data_arr[2];

	if (HAL_I2C_Master_Transmit(sht->i2c, (sht->Dev_Addr << 1), out_arr, 5, HAL_MAX_DELAY)!= HAL_OK) return 0;


	/* Calculate and pack High-Clear limit: update temperature, keep previous humidity */

	temp_Setting = sht->Alert.high_clear_raw;

	temp_T = (uint16_t)(((T_High_Clear + 45.0f) / 175.0f * 65535.0f) + 64.0f);    	// Convert temperature from °C to 16-bit raw format. Add 64 to round before dropping 7 LSBs (64 = half of 128).
	temp_T = (temp_T & 0b1111111110000000) >> 7;

	temp_Setting = temp_Setting & 0b1111111000000000;								// Clear previous temperature bits, keep humidity bits

	sht->Alert.high_clear_raw = temp_T | temp_Setting;								// Build new packed alert limit value

	data_arr[0] = (uint8_t)(sht->Alert.high_clear_raw >> 8);
	data_arr[1] = (uint8_t)(sht->Alert.high_clear_raw & 0x00FF);
	data_arr[2] = SHT3x_CRCCalc (data_arr);

	SHT3x_Cmd_Devide(S31_ALRT_WRITE_HIGH_CLEAR, cmd_arr);

	out_arr[0] = cmd_arr[0];
	out_arr[1] = cmd_arr[1];
	out_arr[2] = data_arr[0];
	out_arr[3] = data_arr[1];
	out_arr[4] = data_arr[2];

	if (HAL_I2C_Master_Transmit(sht->i2c, (sht->Dev_Addr << 1), out_arr, 5, HAL_MAX_DELAY)!= HAL_OK) return 0;


	/* Calculate and pack Low-Clear limit: update temperature, keep previous humidity */

	temp_Setting = sht->Alert.low_clear_raw;

	temp_T = (uint16_t)(((T_Low_Clear + 45.0f) / 175.0f * 65535.0f) + 64.0f);    	// Convert temperature from °C to 16-bit raw format. Add 64 to round before dropping 7 LSBs (64 = half of 128).
	temp_T = (temp_T & 0b1111111110000000) >> 7;

	temp_Setting = temp_Setting & 0b1111111000000000;								// Clear previous temperature bits, keep humidity bits

	sht->Alert.low_clear_raw = temp_T | temp_Setting;								// Build new packed alert limit value

	data_arr[0] = (uint8_t)(sht->Alert.low_clear_raw >> 8);
	data_arr[1] = (uint8_t)(sht->Alert.low_clear_raw & 0x00FF);
	data_arr[2] = SHT3x_CRCCalc (data_arr);

	SHT3x_Cmd_Devide(S31_ALRT_WRITE_LOW_CLEAR, cmd_arr);

	out_arr[0] = cmd_arr[0];
	out_arr[1] = cmd_arr[1];
	out_arr[2] = data_arr[0];
	out_arr[3] = data_arr[1];
	out_arr[4] = data_arr[2];

	if (HAL_I2C_Master_Transmit(sht->i2c, (sht->Dev_Addr << 1), out_arr, 5, HAL_MAX_DELAY)!= HAL_OK) return 0;


	/* Calculate and pack Low-Set limit: update temperature, keep previous humidity */

	temp_Setting = sht->Alert.low_set_raw;

	temp_T = (uint16_t)(((T_Low_Set + 45.0f) / 175.0f * 65535.0f) + 64.0f);    	// Convert temperature from °C to 16-bit raw format. Add 64 to round before dropping 7 LSBs (64 = half of 128).
	temp_T = (temp_T & 0b1111111110000000) >> 7;

	temp_Setting = temp_Setting & 0b1111111000000000;							// Clear previous temperature bits, keep humidity bits

	sht->Alert.low_set_raw = temp_T | temp_Setting;								// Build new packed alert limit value

	data_arr[0] = (uint8_t)(sht->Alert.low_set_raw >> 8);
	data_arr[1] = (uint8_t)(sht->Alert.low_set_raw & 0x00FF);
	data_arr[2] = SHT3x_CRCCalc (data_arr);

	SHT3x_Cmd_Devide(S31_ALRT_WRITE_LOW_SET, cmd_arr);

	out_arr[0] = cmd_arr[0];
	out_arr[1] = cmd_arr[1];
	out_arr[2] = data_arr[0];
	out_arr[3] = data_arr[1];
	out_arr[4] = data_arr[2];

	if (HAL_I2C_Master_Transmit(sht->i2c, (sht->Dev_Addr << 1), out_arr, 5, HAL_MAX_DELAY)!= HAL_OK) return 0;


	return 1;

}


uint8_t SHT3x_Alrt_SetHumid (SHT3x_HandleTypeDef* sht, float H_High_Set, float H_High_Clear, float H_Low_Clear, float H_Low_Set) {

	uint8_t cmd_arr[2] = {0};
	uint8_t data_arr[3] = {0};
	uint8_t out_arr[5] = {0};

	uint16_t temp_H = 0;
	uint16_t temp_Setting = 0;

	if ((H_High_Set > 100) || (H_High_Clear > H_High_Set) || (H_Low_Clear > H_High_Clear) || (H_Low_Set > H_Low_Clear) || (H_Low_Set < 0)) return 0;


	if (SHT3x_Alrt_ReadSettings (sht) == 0) return 0;


	/* Calculate and pack High-Set limit: update humidity, keep previous temperature */

	temp_Setting = sht->Alert.high_set_raw;

	temp_H = (uint16_t)(65535.0f * H_High_Set / 100.0f + 256.0f);				// Add 256 to round the 16-bit raw RH value before keeping only the top 7 bits.
	temp_H = temp_H & 0b1111111000000000;

	temp_Setting = temp_Setting & 0b0000000111111111;

	sht->Alert.high_set_raw = temp_H | temp_Setting;

	data_arr[0] = (uint8_t)(sht->Alert.high_set_raw >> 8);
	data_arr[1] = (uint8_t)(sht->Alert.high_set_raw & 0x00FF);
	data_arr[2] = SHT3x_CRCCalc (data_arr);

	SHT3x_Cmd_Devide(S31_ALRT_WRITE_HIGH_SET, cmd_arr);

	out_arr[0] = cmd_arr[0];
	out_arr[1] = cmd_arr[1];
	out_arr[2] = data_arr[0];
	out_arr[3] = data_arr[1];
	out_arr[4] = data_arr[2];

	if (HAL_I2C_Master_Transmit(sht->i2c, (sht->Dev_Addr << 1), out_arr, 5, HAL_MAX_DELAY)!= HAL_OK) return 0;


	/* Calculate and pack High-Clear limit: update humidity, keep previous temperature */

	temp_Setting = sht->Alert.high_clear_raw;

	temp_H = (uint16_t)(65535.0f * H_High_Clear / 100.0f + 256.0f);  			// Add 256 to round the 16-bit raw RH value before keeping only the top 7 bits.
	temp_H = temp_H & 0b1111111000000000;

	temp_Setting = temp_Setting & 0b0000000111111111;

	sht->Alert.high_clear_raw = temp_H | temp_Setting;

	data_arr[0] = (uint8_t)(sht->Alert.high_clear_raw >> 8);
	data_arr[1] = (uint8_t)(sht->Alert.high_clear_raw & 0x00FF);
	data_arr[2] = SHT3x_CRCCalc (data_arr);

	SHT3x_Cmd_Devide(S31_ALRT_WRITE_HIGH_CLEAR, cmd_arr);

	out_arr[0] = cmd_arr[0];
	out_arr[1] = cmd_arr[1];
	out_arr[2] = data_arr[0];
	out_arr[3] = data_arr[1];
	out_arr[4] = data_arr[2];

	if (HAL_I2C_Master_Transmit(sht->i2c, (sht->Dev_Addr << 1), out_arr, 5, HAL_MAX_DELAY)!= HAL_OK) return 0;


	/* Calculate and pack Low-Clear limit: update humidity, keep previous temperature */

	temp_Setting = sht->Alert.low_clear_raw;

	temp_H = (uint16_t)(65535.0f * H_Low_Clear / 100.0f + 256.0f); 				// Add 256 to round the 16-bit raw RH value before keeping only the top 7 bits.
	temp_H = temp_H & 0b1111111000000000;

	temp_Setting = temp_Setting & 0b0000000111111111;

	sht->Alert.low_clear_raw = temp_H | temp_Setting;

	data_arr[0] = (uint8_t)(sht->Alert.low_clear_raw >> 8);
	data_arr[1] = (uint8_t)(sht->Alert.low_clear_raw & 0x00FF);
	data_arr[2] = SHT3x_CRCCalc (data_arr);

	SHT3x_Cmd_Devide(S31_ALRT_WRITE_LOW_CLEAR, cmd_arr);

	out_arr[0] = cmd_arr[0];
	out_arr[1] = cmd_arr[1];
	out_arr[2] = data_arr[0];
	out_arr[3] = data_arr[1];
	out_arr[4] = data_arr[2];

	if (HAL_I2C_Master_Transmit(sht->i2c, (sht->Dev_Addr << 1), out_arr, 5, HAL_MAX_DELAY)!= HAL_OK) return 0;


	/* Calculate and pack Low-Set limit: update humidity, keep previous temperature */

	temp_Setting = sht->Alert.low_set_raw;

	temp_H = (uint16_t)(65535.0f * H_Low_Set / 100.0f + 256.0f);				// Add 256 to round the 16-bit raw RH value before keeping only the top 7 bits.
	temp_H = temp_H & 0b1111111000000000;

	temp_Setting = temp_Setting & 0b0000000111111111;

	sht->Alert.low_set_raw = temp_H | temp_Setting;

	data_arr[0] = (uint8_t)(sht->Alert.low_set_raw >> 8);
	data_arr[1] = (uint8_t)(sht->Alert.low_set_raw & 0x00FF);
	data_arr[2] = SHT3x_CRCCalc (data_arr);

	SHT3x_Cmd_Devide(S31_ALRT_WRITE_LOW_SET, cmd_arr);

	out_arr[0] = cmd_arr[0];
	out_arr[1] = cmd_arr[1];
	out_arr[2] = data_arr[0];
	out_arr[3] = data_arr[1];
	out_arr[4] = data_arr[2];

	if (HAL_I2C_Master_Transmit(sht->i2c, (sht->Dev_Addr << 1), out_arr, 5, HAL_MAX_DELAY)!= HAL_OK) return 0;

	return 1;
}

/*Private functions-------------------------------------------------------------------------------------------------*/

static uint16_t SHT3x_Get_SS_Command(SHT3x_Repeat_t rep, SHT3x_ClockStr_t cl_str) {

	if (cl_str == SHT3x_CLK_STR_DISABLE) {

		switch (rep) {

			case SHT3x_REPEAT_LOW:

				return S31_CMD_SS_MES_NOSTR_LOW;


			case SHT3x_REPEAT_MEDIUM:

				return S31_CMD_SS_MES_NOSTR_MED;


			case SHT3x_REPEAT_HIGH:

				return S31_CMD_SS_MES_NOSTR_HIGH;

			default:

				return 0;


		}

	} else if (cl_str == SHT3x_CLK_STR_ENABLE) {

		switch (rep)  {

			case SHT3x_REPEAT_LOW:

				return S31_CMD_SS_MES_STR_LOW;


			case SHT3x_REPEAT_MEDIUM:

				return S31_CMD_SS_MES_STR_MED;


			case SHT3x_REPEAT_HIGH:

				return S31_CMD_SS_MES_STR_HIGH;

			default:

				return 0;

		}

	}

	return 0;

}

static uint16_t SHT3x_Get_Periodic_Command (SHT3x_Repeat_t rep, SHT3x_PerRate_t rate) {

	if (rep == SHT3x_REPEAT_LOW) {

		switch (rate) {

			case SHT3x_RATE_0_5_HZ :

				return S31_CMD_PER_MES_05_LOW;

			case SHT3x_RATE_1_HZ :

				return S31_CMD_PER_MES_1_LOW;

			case SHT3x_RATE_2_HZ :

				return S31_CMD_PER_MES_2_LOW;

			case SHT3x_RATE_4_HZ :

				return S31_CMD_PER_MES_4_LOW;

			case SHT3x_RATE_10_HZ :

				return S31_CMD_PER_MES_10_LOW;

			default :

				return 0;

		}

	} else if (rep == SHT3x_REPEAT_MEDIUM) {

		switch (rate) {

			case SHT3x_RATE_0_5_HZ :

				return S31_CMD_PER_MES_05_MED;

			case SHT3x_RATE_1_HZ :

				return S31_CMD_PER_MES_1_MED;

			case SHT3x_RATE_2_HZ :

				return S31_CMD_PER_MES_2_MED;

			case SHT3x_RATE_4_HZ :

				return S31_CMD_PER_MES_4_MED;

			case SHT3x_RATE_10_HZ :

				return S31_CMD_PER_MES_10_MED;

			default :

				return 0;

		}

	} else if (rep == SHT3x_REPEAT_HIGH) {

		switch (rate) {

			case SHT3x_RATE_0_5_HZ :

				return S31_CMD_PER_MES_05_HIGH;

			case SHT3x_RATE_1_HZ :

				return S31_CMD_PER_MES_1_HIGH;

			case SHT3x_RATE_2_HZ :

				return S31_CMD_PER_MES_2_HIGH;

			case SHT3x_RATE_4_HZ :

				return S31_CMD_PER_MES_4_HIGH;

			case SHT3x_RATE_10_HZ :

				return S31_CMD_PER_MES_10_HIGH;

			default :

				return 0;

		}

	}

	return 0;

}

static void SHT3x_Cmd_Devide (uint16_t cmd, uint8_t *cmd_arr) {

	cmd_arr[0] = cmd >> 8;
	cmd_arr[1] = cmd & 0x00FF;

}

static void SHT3x_Calc_Temp_Hum (SHT3x_HandleTypeDef* sht, uint8_t* data_arr) {

	sht->Temp_Raw = (data_arr[0] << 8)+data_arr[1];
	sht->Temp = -45.0f + 175.0f * (float)sht->Temp_Raw / 65535.0f;

	sht->Humid_Raw = (data_arr[3] << 8)+data_arr[4];
	sht->Humid = 100.0f * (float)sht->Humid_Raw / 65535.0f;

	sht->crc_Temp = data_arr[2];
	sht->crc_Humid = data_arr[5];

}

static uint8_t SHT3x_Temp_CRC (SHT3x_HandleTypeDef* sht) {

		uint8_t data_arr[2] = {0};

		data_arr[0] = sht->Temp_Raw >> 8;
		data_arr[1] = sht->Temp_Raw & 0x00FF;

		if (SHT3x_CRCCalc (data_arr) == sht->crc_Temp) {
			return 1;
		} else {
			return 0;
		}


}


static uint8_t SHT3x_Humid_CRC (SHT3x_HandleTypeDef* sht) {

		uint8_t data_arr[2] = {0};

		data_arr[0] = sht->Humid_Raw >> 8;
		data_arr[1] = sht->Humid_Raw & 0x00FF;


		if (SHT3x_CRCCalc (data_arr) == sht->crc_Humid) {
			return 1;
		} else {
			return 0;
		}

}

static uint8_t SHT3x_Status_CRC (uint8_t* reg_data) {


		if (SHT3x_CRCCalc (reg_data) == reg_data[2]) {
			return 1;
		} else {
			return 0;
		}

}


static uint8_t SHT3x_CRCCalc (uint8_t* data_arr) {

	uint8_t crc = 0xFF;

	    for (uint8_t data_i = 0; data_i < 2; data_i++) {
	        crc ^= data_arr[data_i];
	        for (uint8_t bit_i = 0; bit_i < 8; bit_i++) {
	            if (crc & 0x80) {
	                crc = (crc << 1) ^ 0x31;
	            } else {
	                crc = (crc << 1);
	            }
	        }
	    }
	    return crc;


}


