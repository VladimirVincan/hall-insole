/*
 * c3dhall11.c
 *
 *  Created on: Jun 23, 2024
 *      Author: bici
 */
#include "c3dhall11.h"

uint8_t C3DHALL11_DATA_BUF[12] = {0};
uint8_t C3DHALL11_SENSOR_CONFIG[2] = {0};
uint8_t C3DHALL11_TEMP_CONFIG = 0;

HAL_StatusTypeDef c3dhall11_generic_read(UART_HandleTypeDef huart2, I2C_HandleTypeDef hi2c1, const uint8_t i2c_addr, const uint8_t reg_addr, uint8_t *reg, const int len)
{
	uint8_t transmit[32];
	HAL_StatusTypeDef ret;

	const uint8_t i2c_rd_addr = (i2c_addr << 1) | 0x1;
	const uint8_t i2c_wr_addr = (i2c_addr << 1) | 0x0;

	reg[0] = reg_addr | 0x80;
	ret = HAL_I2C_Master_Transmit(&hi2c1, i2c_wr_addr, reg, 1, HAL_MAX_DELAY);
	HAL_Delay(100);
	if (ret != HAL_OK) {
		strcpy((char*)transmit, "TxEr\r\n");
		HAL_UART_Transmit(&huart2, transmit, strlen((char*)transmit), HAL_MAX_DELAY);
		return ret;
	}
	else {
		ret = HAL_I2C_Master_Receive(&hi2c1, i2c_rd_addr, reg, len, HAL_MAX_DELAY);
		HAL_Delay(100);
		if (ret != HAL_OK) {
			strcpy((char*)transmit, "RxEr\r\n");
			HAL_UART_Transmit(&huart2, transmit, strlen((char*)transmit), HAL_MAX_DELAY);
			return ret;
		}
	}
	HAL_Delay(100);
	return ret;
}

HAL_StatusTypeDef c3dhall11_generic_write(UART_HandleTypeDef huart2, I2C_HandleTypeDef hi2c1, const uint8_t i2c_addr, const uint8_t reg_addr, uint8_t *reg, const int len)
{
	HAL_StatusTypeDef ret;
	const uint8_t i2c_wr_addr = (i2c_addr << 1) | 0x0;

	uint8_t data_buf[ 16 ] = { 0 };
	data_buf[ 0 ] = reg_addr; // | 0x80;
	for ( uint8_t cnt = 0; cnt < len; cnt++ )
	{
		data_buf[ cnt + 1 ] = reg[ cnt ];
	}

	ret = HAL_I2C_Master_Transmit(&hi2c1, i2c_wr_addr, data_buf, len + 1, HAL_MAX_DELAY);
	HAL_Delay(100);
	if (ret != HAL_OK) {
		strcpy((char*)data_buf, "Wr TxEr\r\n");
		HAL_UART_Transmit(&huart2, data_buf, strlen((char*)data_buf), HAL_MAX_DELAY);
		return ret;
	}
	return ret;
}

HAL_StatusTypeDef c3dhall11_read_register(UART_HandleTypeDef huart2, I2C_HandleTypeDef hi2c1, const uint8_t i2c_addr, const uint8_t reg_addr, uint8_t *reg)
{
	return c3dhall11_generic_read(huart2, hi2c1, i2c_addr, reg_addr, reg, 1);
}

HAL_StatusTypeDef c3dhall11_write_register(UART_HandleTypeDef huart2, I2C_HandleTypeDef hi2c1, const uint8_t i2c_addr, const uint8_t reg_addr, uint8_t reg)
{
	return c3dhall11_generic_write(huart2, hi2c1, i2c_addr, reg_addr, &reg, 1);
}

HAL_StatusTypeDef c3dhall11_init(UART_HandleTypeDef huart2, I2C_HandleTypeDef hi2c1, const uint8_t i2c_addr)
{
	c3dhall11_write_register(huart2, hi2c1, i2c_addr, C3DHALL11_REG_DEVICE_CONFIG_1,
			C3DHALL11_CONV_AVG_32X |
			C3DHALL11_I2C_RD_STANDARD);
	c3dhall11_write_register(huart2, hi2c1, i2c_addr, C3DHALL11_REG_SENSOR_CONFIG_1,
			C3DHALL11_MAG_CH_EN_ENABLE_XYZ);
	c3dhall11_write_register(huart2, hi2c1, i2c_addr, C3DHALL11_REG_SENSOR_CONFIG_2,
			C3DHALL11_ANGLE_EN_XY_ANGLE |
			C3DHALL11_X_Y_RANGE_40mT |
			C3DHALL11_Z_RANGE_40mT);
	c3dhall11_write_register(huart2, hi2c1, i2c_addr, C3DHALL11_REG_T_CONFIG,
			C3DHALL11_T_CH_EN_ENABLE);
	c3dhall11_write_register(huart2, hi2c1, i2c_addr, C3DHALL11_REG_INT_CONFIG_1,
			C3DHALL11_MASK_INTB_DISABLE);
	c3dhall11_write_register(huart2, hi2c1, i2c_addr, C3DHALL11_REG_DEVICE_CONFIG_2,
			C3DHALL11_OPERATING_MODE_CONTINUOUS);
}

HAL_StatusTypeDef c3dhall11_read_data(UART_HandleTypeDef huart2, I2C_HandleTypeDef hi2c1, const uint8_t i2c_addr)
{
	c3dhall11_generic_read(huart2, hi2c1, i2c_addr, C3DHALL11_REG_T_MSB_RESULT, &C3DHALL11_DATA_BUF, 6);
	c3dhall11_generic_read(huart2, hi2c1, i2c_addr, C3DHALL11_REG_SENSOR_CONFIG_1, &C3DHALL11_SENSOR_CONFIG, 2);
	c3dhall11_read_register(huart2, hi2c1, i2c_addr, C3DHALL11_REG_T_CONFIG, &C3DHALL11_TEMP_CONFIG);
}

float c3dhall11_get_x()
{
	int16_t val = (C3DHALL11_DATA_BUF[2] << 8) | C3DHALL11_DATA_BUF[3];
	float ret = 0;
	if (C3DHALL11_SENSOR_CONFIG[1] & C3DHALL11_X_Y_RANGE_80mT)
		ret = (float)val / C3DHALL11_XYZ_SENSITIVITY_80mT;
	else
		ret = (float)val / C3DHALL11_XYZ_SENSITIVITY_40mT;
	return ret;
}

float c3dhall11_get_y()
{
	int16_t val = (C3DHALL11_DATA_BUF[4] << 8) | C3DHALL11_DATA_BUF[5];
	float ret = 0;
	if (C3DHALL11_SENSOR_CONFIG[1] & C3DHALL11_X_Y_RANGE_80mT)
		ret = (float)val / C3DHALL11_XYZ_SENSITIVITY_80mT;
	else
		ret = (float)val / C3DHALL11_XYZ_SENSITIVITY_40mT;
	return ret;
}

float c3dhall11_get_z()
{
	int16_t val = (C3DHALL11_DATA_BUF[6] << 8) | C3DHALL11_DATA_BUF[7];
	float ret = 0;
	if (C3DHALL11_SENSOR_CONFIG[1] & C3DHALL11_Z_RANGE_80mT)
		ret = (float)val / C3DHALL11_XYZ_SENSITIVITY_80mT;
	else
		ret = (float)val / C3DHALL11_XYZ_SENSITIVITY_40mT;
	return ret;
}

float c3dhall11_get_temp()
{
	uint8_t temp_config = 0;
	int16_t val = 0;
	if (C3DHALL11_TEMP_CONFIG & C3DHALL11_T_CH_EN_ENABLE)
		val = (int16_t)(((uint16_t)C3DHALL11_DATA_BUF[0] << 8) | C3DHALL11_DATA_BUF[1]) - C3DHALL11_TEMP_ADC_T0;
	float ret = C3DHALL11_TEMP_SENS_T0 + (float)val/C3DHALL11_TEMP_ADC_RESOLUTION;
	return ret;
}

int8_t c3dhall11_get_magnitude()
{
	int8_t val = 0;
	if (C3DHALL11_SENSOR_CONFIG[0] & C3DHALL11_MAG_CH_EN_BIT_MASK)
		val = C3DHALL11_DATA_BUF[11];
	return val;
}

float c3dhall11_get_angle()
{
	uint8_t val = ((uint16_t)C3DHALL11_DATA_BUF[9] << 8) | C3DHALL11_DATA_BUF[10];
	float ret = 0;
	if (C3DHALL11_SENSOR_CONFIG[1] & C3DHALL11_ANGLE_EN_BIT_MASK)
		ret = (float)val / C3DHALL11_ANGLE_RESOLUTION;
	return ret;
}

uint16_t c3dhall11_get_sensor_config_1()
{
	uint8_t config = 0x00;
	config = (uint8_t)C3DHALL11_SENSOR_CONFIG[0];
	return config;
}

uint16_t c3dhall11_get_sensor_config_2()
{
	uint8_t config = 0x00;
	config = (uint8_t)C3DHALL11_SENSOR_CONFIG[1];
	return config;
}

uint8_t c3dhall11_get_data(int index)
{
	return C3DHALL11_DATA_BUF[index];
}





