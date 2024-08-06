/*
 * c3dhall9.h
 *
 *  Created on: Jun 23, 2024
 *      Author: bici
 */

#ifndef INC_C3DHALL9_H_
#define INC_C3DHALL9_H_

#include "stm32l4xx_hal.h"

/**
 * @brief 3D Hall 9 Register Map.
 * @details Specified register map of 3D Hall 9 Click driver.
 */
#define C3DHALL9_REG_EEPROM_02                  0x02
#define C3DHALL9_REG_EEPROM_03                  0x03
#define C3DHALL9_REG_EEPROM_0D                  0x0D
#define C3DHALL9_REG_EEPROM_0E                  0x0E
#define C3DHALL9_REG_EEPROM_0F                  0x0F
#define C3DHALL9_REG_VOLATILE_27                0x27
#define C3DHALL9_REG_VOLATILE_28                0x28
#define C3DHALL9_REG_VOLATILE_29                0x29
#define C3DHALL9_REG_CUSTOMER_ACCESS            0x35

/**
 * @brief 3D Hall 9 register setting.
 * @details Specified setting for registers of 3D Hall 9 Click driver.
 */
#define C3DHALL9_EEPROM_02_I2C_THRESHOLD_18V    0x00000200ul
#define C3DHALL9_EEPROM_02_DISABLE_SLAVE_ADC    0x00020000ul
#define C3DHALL9_EEPROM_02_ENABLE_Z             0x00000100ul
#define C3DHALL9_EEPROM_02_ENABLE_Y             0x00000080ul
#define C3DHALL9_EEPROM_02_ENABLE_X             0x00000040ul
#define C3DHALL9_VOLATILE_27_ACTIVE_MODE        0x00000000ul
#define C3DHALL9_VOLATILE_27_SLEEP_MODE         0x00000001ul
#define C3DHALL9_VOLATILE_27_LOW_POWER_MODE     0x00000002ul
#define C3DHALL9_VOLATILE_27_I2C_SINGLE         0x00000000ul
#define C3DHALL9_VOLATILE_27_I2C_FAST_LOOP      0x00000004ul
#define C3DHALL9_VOLATILE_27_I2C_FULL_LOOP      0x00000008ul
#define C3DHALL9_REG_CUSTOMER_ACCESS_CODE       0x2C413534ul

#define C3DHALL9_EEPROM_02_I2C_ADDR_MIN         0x00000000ul
#define C3DHALL9_EEPROM_02_I2C_ADDR_FIRST       0x00000400ul
#define C3DHALL9_EEPROM_02_I2C_ADDR_MAX         0x0001FC00ul

/**
 * @brief 3D Hall 9 measurements values.
 * @details Specified values for measurements of 3D Hall 9 Click driver.
 */
#define C3DHALL9_12BIT_RESOLUTION               0x1000
#define C3DHALL9_SIGN_BIT                       0x0800
#define C3DHALL9_DATA_READY_BIT                 0x0080
#define C3DHALL9_GAUSS_RESOLUTION               4
#define C3DHALL9_TEMPERATURE_MULTIPLIER         302
#define C3DHALL9_TEMPERATURE_SUBTRACTOR         1702
#define C3DHALL9_HALF_CICRLE_DEGREES            180.0
#define C3DHALL9_MATH_TWO_PI                    6.28318530717958

/**
 * @brief 3D Hall 9 device address setting.
 * @details Specified setting for device slave address selection of
 * 3D Hall 9 Click driver.
 */
#define C3DHALL9_SET_DEV_ADDR                   0x60


/*
 * @brief Personal definitions
 * */
#define C3DHALL9_I2C_DEFAULT_ADDR 0x60
#define C3DHALL9_I2C_DEFAULT_RD_ADDR 0xc1
#define C3DHALL9_I2C_DEFAULT_WR_ADDR 0xc0
#define C3DHALL9_I2C_NEW_ADDR 0x00

#define C3DHALL9_REGISTER_MSB_ADDR C3DHALL9_REG_VOLATILE_28
#define C3DHALL9_REGISTER_LSB_ADDR C3DHALL9_REG_VOLATILE_29
#define C3DHALL9_REGISTER_TEST_ADDR C3DHALL9_REG_EEPROM_02

extern uint32_t C3DHALL9_REGISTER_MSB;
extern uint32_t C3DHALL9_REGISTER_LSB;
extern uint32_t C3DHALL9_REGISTER_TEST;

HAL_StatusTypeDef  c3dhall9_read_register(UART_HandleTypeDef huart2, I2C_HandleTypeDef hi2c1, const uint8_t i2c_addr, const uint8_t reg_addr, uint32_t *reg);
void c3dhall9_write_register(I2C_HandleTypeDef hi2c1, const uint8_t i2c_addr, const uint8_t reg_addr, const uint32_t reg);
HAL_StatusTypeDef  c3dhall9_read_data(UART_HandleTypeDef huart2, I2C_HandleTypeDef hi2c1, const uint8_t i2c_addr);
void c3dhall9_init(I2C_HandleTypeDef hi2c1, const uint8_t i2c_addr);
float c3dhall9_get_x();
float c3dhall9_get_y();
float c3dhall9_get_z();
float c3dhall9_get_temp();


#endif /* INC_C3DHALL9_H_ */
