/*
 * c3dhall9.c
 *
 *  Created on: Jun 23, 2024
 *      Author: bici
 */
#include "c3dhall9.h"

uint32_t C3DHALL9_REGISTER_MSB = 0x00;
uint32_t C3DHALL9_REGISTER_LSB = 0x00;
uint32_t C3DHALL9_REGISTER_TEST = 0x00;

void c3dhall9_read_register(I2C_HandleTypeDef hi2c1, const uint8_t i2c_addr, const uint8_t reg_addr, uint32_t *reg)
{
	uint8_t buf[4];
	uint8_t transmit[32];
	HAL_StatusTypeDef ret;

	const uint8_t i2c_rd_addr = (i2c_addr << 1) | 0x1;
	const uint8_t i2c_wr_addr = (i2c_addr << 1) | 0x0;

	buf[0] = reg_addr;
	ret = HAL_I2C_Master_Transmit(&hi2c1, i2c_wr_addr, buf, 1, HAL_MAX_DELAY);
	HAL_Delay(100);
	if (ret != HAL_OK) {
		// strcpy((char*)transmit, "TxEr\r\n");
		// HAL_UART_Transmit(&huart2, transmit, strlen((char*)transmit), HAL_MAX_DELAY);
	}
	else {
		ret = HAL_I2C_Master_Receive(&hi2c1, i2c_rd_addr, buf, 4, HAL_MAX_DELAY);
		HAL_Delay(100);
		if (ret != HAL_OK) {
			// strcpy((char*)transmit, "RxEr\r\n");
			// HAL_UART_Transmit(&huart2, transmit, strlen((char*)transmit), HAL_MAX_DELAY);
		}
	}

	(*reg) = buf[0];
	(*reg) <<= 8;
	(*reg) |= buf[1];
	(*reg) <<= 8;
	(*reg) |= buf[2];
	(*reg) <<= 8;
	(*reg) |= buf[3];
}

void c3dhall9_write_register(I2C_HandleTypeDef hi2c1, const uint8_t i2c_addr, const uint8_t reg_addr, const uint32_t reg)
{
	HAL_StatusTypeDef ret;
	uint8_t buf[5];
	uint8_t transmit[32];

	const uint8_t i2c_wr_addr = (i2c_addr << 1) | 0x0;

	buf[0] = reg_addr;
	buf[1] = (uint8_t)((reg >> 24) & 0xFF);
	buf[2] = (uint8_t)((reg >> 16) & 0xFF);
	buf[3] = (uint8_t)((reg >> 8) & 0xFF);
	buf[4] = (uint8_t)(reg & 0xFF);

	ret = HAL_I2C_Master_Transmit(&hi2c1, i2c_wr_addr, buf, 5, HAL_MAX_DELAY);
	if (ret != HAL_OK) {
		// strcpy((char*)transmit, "WrEr\r\n");
	}
}

void c3dhall9_read_data(I2C_HandleTypeDef hi2c1)
{
	c3dhall9_read_register(hi2c1, C3DHALL9_I2C_DEFAULT_ADDR, C3DHALL9_REGISTER_MSB_ADDR, &C3DHALL9_REGISTER_MSB);
    c3dhall9_read_register(hi2c1, C3DHALL9_I2C_DEFAULT_ADDR, C3DHALL9_REGISTER_LSB_ADDR, &C3DHALL9_REGISTER_LSB);
}

void c3dhall9_init(I2C_HandleTypeDef hi2c1, const uint8_t i2c_addr)
{
	c3dhall9_write_register(hi2c1, i2c_addr, C3DHALL9_REG_EEPROM_02,
			C3DHALL9_EEPROM_02_ENABLE_X |
			C3DHALL9_EEPROM_02_ENABLE_Y |
			C3DHALL9_EEPROM_02_ENABLE_Z |
			C3DHALL9_EEPROM_02_I2C_THRESHOLD_18V |
			C3DHALL9_EEPROM_02_DISABLE_SLAVE_ADC |
			C3DHALL9_EEPROM_02_I2C_ADDR_MIN);
	c3dhall9_write_register(hi2c1, i2c_addr, C3DHALL9_REG_CUSTOMER_ACCESS, C3DHALL9_REG_CUSTOMER_ACCESS_CODE);
}

float c3dhall9_get_x()
{
	int16_t val = ((C3DHALL9_REGISTER_MSB >> 20) & 0x0FF0) | ((C3DHALL9_REGISTER_LSB >> 16) & 0x0F);
	val = (val ^ C3DHALL9_SIGN_BIT) - C3DHALL9_SIGN_BIT;
	float ret = (float)val / C3DHALL9_GAUSS_RESOLUTION;
	return ret;
}

float c3dhall9_get_y()
{
	int16_t val = ((C3DHALL9_REGISTER_MSB >> 12) & 0x0FF0) | ((C3DHALL9_REGISTER_LSB >> 12) & 0x0F);
	val = (val ^ C3DHALL9_SIGN_BIT) - C3DHALL9_SIGN_BIT;
	float ret = (float)val / C3DHALL9_GAUSS_RESOLUTION;
	return ret;
}

float c3dhall9_get_z()
{
	int16_t val = ((C3DHALL9_REGISTER_MSB >> 4) & 0x0FF0) | ((C3DHALL9_REGISTER_LSB >> 8) & 0x0F);
	val = (val ^ C3DHALL9_SIGN_BIT) - C3DHALL9_SIGN_BIT;
	float ret = (float)val / C3DHALL9_GAUSS_RESOLUTION;
	return ret;
}

float c3dhall9_get_temp()
{
	int32_t val = ((C3DHALL9_REGISTER_MSB & 0x0000003F) << 6) | (C3DHALL9_REGISTER_LSB & 0x0000003F);
	float ret = C3DHALL9_TEMPERATURE_MULTIPLIER*((float)val - C3DHALL9_TEMPERATURE_SUBTRACTOR)/C3DHALL9_12BIT_RESOLUTION;
	return ret;
}

