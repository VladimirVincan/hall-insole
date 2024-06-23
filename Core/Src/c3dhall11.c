/*
 * c3dhall11.c
 *
 *  Created on: Jun 23, 2024
 *      Author: bici
 */


err_t c3dhall11_default_cfg ( c3dhall11_t *ctx )
{
    err_t error_flag = C3DHALL11_OK;
    if ( C3DHALL11_ERROR == c3dhall11_check_communication ( ctx ) )
    {
        return C3DHALL11_ERROR;
    }
    error_flag |= c3dhall11_write_register ( ctx, C3DHALL11_REG_DEVICE_CONFIG_1, C3DHALL11_CONV_AVG_32X |
                                                                                 C3DHALL11_I2C_RD_STANDARD );
    error_flag |= c3dhall11_write_register ( ctx, C3DHALL11_REG_SENSOR_CONFIG_1, C3DHALL11_MAG_CH_EN_ENABLE_XYZ );
    error_flag |= c3dhall11_write_register ( ctx, C3DHALL11_REG_SENSOR_CONFIG_2, C3DHALL11_ANGLE_EN_XY_ANGLE |
                                                                                 C3DHALL11_X_Y_RANGE_40mT |
                                                                                 C3DHALL11_Z_RANGE_40mT );
    error_flag |= c3dhall11_write_register ( ctx, C3DHALL11_REG_T_CONFIG, C3DHALL11_T_CH_EN_ENABLE );
    error_flag |= c3dhall11_write_register ( ctx, C3DHALL11_REG_INT_CONFIG_1, C3DHALL11_MASK_INTB_DISABLE );
    error_flag |= c3dhall11_write_register ( ctx, C3DHALL11_REG_DEVICE_CONFIG_2, C3DHALL11_OPERATING_MODE_CONTINUOUS );
    return error_flag;
}

err_t c3dhall11_read_data ( c3dhall11_t *ctx, c3dhall11_data_t *data_out )
{
    uint8_t data_buf[ 12 ] = { 0 };
    err_t error_flag = c3dhall11_generic_read ( ctx, C3DHALL11_REG_T_MSB_RESULT, data_buf, 12 );
    if ( ( C3DHALL11_OK == error_flag ) && ( data_buf[ 8 ] & C3DHALL11_CONV_STATUS_DATA_READY ) )
    {
        uint8_t sensor_config[ 2 ] = { 0 };
        error_flag |= c3dhall11_generic_read ( ctx, C3DHALL11_REG_SENSOR_CONFIG_1, sensor_config, 2 );
        if ( sensor_config[ 0 ] & C3DHALL11_MAG_CH_EN_BIT_MASK )
        {
            data_out->magnitude = data_buf[ 11 ];
        }
        if ( sensor_config[ 1 ] & C3DHALL11_ANGLE_EN_BIT_MASK )
        {
            data_out->angle = ( ( ( uint16_t ) data_buf[ 9 ] << 8 ) | data_buf[ 10 ] ) / C3DHALL11_ANGLE_RESOLUTION;
        }
        uint8_t temp_config = 0;
        error_flag |= c3dhall11_read_register ( ctx, C3DHALL11_REG_T_CONFIG, &temp_config );
        if ( temp_config & C3DHALL11_T_CH_EN_ENABLE )
        {
            data_out->temperature = C3DHALL11_TEMP_SENS_T0 +
                                    ( ( int16_t ) ( ( ( uint16_t ) data_buf[ 0 ] << 8 ) | data_buf[ 1 ] ) - C3DHALL11_TEMP_ADC_T0 )
                                    / C3DHALL11_TEMP_ADC_RESOLUTION;
        }
        data_out->x_axis = ( float ) ( ( ( int16_t ) data_buf[ 2 ] << 8 ) | data_buf[ 3 ] );
        data_out->y_axis = ( float ) ( ( ( int16_t ) data_buf[ 4 ] << 8 ) | data_buf[ 5 ] );
        data_out->z_axis = ( float ) ( ( ( int16_t ) data_buf[ 6 ] << 8 ) | data_buf[ 7 ] );
        if ( sensor_config[ 1 ] & C3DHALL11_X_Y_RANGE_80mT )
        {
            data_out->x_axis /= C3DHALL11_XYZ_SENSITIVITY_80mT;
            data_out->y_axis /= C3DHALL11_XYZ_SENSITIVITY_80mT;
        }
        else
        {
            data_out->x_axis /= C3DHALL11_XYZ_SENSITIVITY_40mT;
            data_out->y_axis /= C3DHALL11_XYZ_SENSITIVITY_40mT;
        }
        if ( sensor_config[ 1 ] & C3DHALL11_Z_RANGE_80mT )
        {
            data_out->z_axis /= C3DHALL11_XYZ_SENSITIVITY_80mT;
        }
        else
        {
            data_out->z_axis /= C3DHALL11_XYZ_SENSITIVITY_40mT;
        }
        return error_flag;
    }
    return C3DHALL11_ERROR;
}
