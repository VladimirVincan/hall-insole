/*
 * c3dhall9.c
 *
 *  Created on: Jun 23, 2024
 *      Author: bici
 */


void c3dhall9_init(const uint8_t i2c_addr)
{
	write_register(i2c_addr, C3DHALL9_REG_EEPROM_02,
			C3DHALL9_EEPROM_02_ENABLE_X |
			C3DHALL9_EEPROM_02_ENABLE_Y |
			C3DHALL9_EEPROM_02_ENABLE_Z |
			C3DHALL9_EEPROM_02_I2C_THRESHOLD_18V |
			C3DHALL9_EEPROM_02_DISABLE_SLAVE_ADC |
			C3DHALL9_EEPROM_02_I2C_ADDR_MIN);
	write_register(i2c_addr, C3DHALL9_REG_CUSTOMER_ACCESS, C3DHALL9_REG_CUSTOMER_ACCESS_CODE);
}

float c3dhall9_get_x()
{
	int16_t val = ((REGISTER_MSB >> 20) & 0x0FF0) | ((REGISTER_LSB >> 16) & 0x0F);
	val = (val ^ C3DHALL9_SIGN_BIT) - C3DHALL9_SIGN_BIT;
	float ret = (float)val / C3DHALL9_GAUSS_RESOLUTION;
	return ret;
}

float c3dhall9_get_y()
{
	int16_t val = ((REGISTER_MSB >> 12) & 0x0FF0) | ((REGISTER_LSB >> 12) & 0x0F);
	val = (val ^ C3DHALL9_SIGN_BIT) - C3DHALL9_SIGN_BIT;
	float ret = (float)val / C3DHALL9_GAUSS_RESOLUTION;
	return ret;
}

float c3dhall9_get_z()
{
	int16_t val = ((REGISTER_MSB >> 4) & 0x0FF0) | ((REGISTER_LSB >> 8) & 0x0F);
	val = (val ^ C3DHALL9_SIGN_BIT) - C3DHALL9_SIGN_BIT;
	float ret = (float)val / C3DHALL9_GAUSS_RESOLUTION;
	return ret;
}

float c3dhall9_get_temp()
{
	int32_t val = ((REGISTER_MSB & 0x0000003F) << 6) | (REGISTER_LSB & 0x0000003F);
	float ret = C3DHALL9_TEMPERATURE_MULTIPLIER*((float)val - C3DHALL9_TEMPERATURE_SUBTRACTOR)/C3DHALL9_12BIT_RESOLUTION;
	return ret;
}

