/*
 * c3dhall9.h
 *
 *  Created on: Jun 23, 2024
 *      Author: bici
 */

#ifndef INC_C3DHALL9_H_
#define INC_C3DHALL9_H_

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

/**
 * @brief 3D Hall 9 register setting.
 * @details Specified setting for registers of 3D Hall 9 Click driver.
 */
#define C3DHALL9_EEPROM_02_ENABLE_Z             0x00000100ul
#define C3DHALL9_EEPROM_02_ENABLE_Y             0x00000080ul
#define C3DHALL9_EEPROM_02_ENABLE_X             0x00000040ul
#define C3DHALL9_VOLATILE_27_ACTIVE_MODE        0x00000000ul
#define C3DHALL9_VOLATILE_27_SLEEP_MODE         0x00000001ul
#define C3DHALL9_VOLATILE_27_LOW_POWER_MODE     0x00000002ul
#define C3DHALL9_VOLATILE_27_I2C_SINGLE         0x00000000ul
#define C3DHALL9_VOLATILE_27_I2C_FAST_LOOP      0x00000004ul
#define C3DHALL9_VOLATILE_27_I2C_FULL_LOOP      0x00000008ul

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
#define C3DHALL9_TEMPERATURE_SUBTRACTOR         1702
#define C3DHALL9_HALF_CICRLE_DEGREES            180.0
#define C3DHALL9_MATH_TWO_PI                    6.28318530717958

/**
 * @brief 3D Hall 9 device address setting.
 * @details Specified setting for device slave address selection of
 * 3D Hall 9 Click driver.
 */
#define C3DHALL9_SET_DEV_ADDR                   0x60



#endif /* INC_C3DHALL9_H_ */
