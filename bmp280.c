#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "bmp280.h"

/**
 * @brief chip register definition
 */
#define BMP280_REG_NVM_PAR_T1_L        	0x88        /**< NVM PAR T1 low register */
#define BMP280_REG_NVM_PAR_T1_H        	0x89        /**< NVM PAR T1 high register */
#define BMP280_REG_NVM_PAR_T2_L        	0x8A        /**< NVM PAR T2 low register */
#define BMP280_REG_NVM_PAR_T2_H        	0x8B        /**< NVM PAR T2 high register */
#define BMP280_REG_NVM_PAR_T3_L        	0x8C        /**< NVM PAR T3 low register */
#define BMP280_REG_NVM_PAR_T3_H        	0x8D        /**< NVM PAR T3 high register */
#define BMP280_REG_NVM_PAR_P1_L        	0x8E        /**< NVM PAR P1 low register */
#define BMP280_REG_NVM_PAR_P1_H        	0x8F        /**< NVM PAR P1 high register */
#define BMP280_REG_NVM_PAR_P2_L        	0x90        /**< NVM PAR P2 low register */
#define BMP280_REG_NVM_PAR_P2_H        	0x91        /**< NVM PAR P2 high register */
#define BMP280_REG_NVM_PAR_P3_L        	0x92        /**< NVM PAR P3 low register */
#define BMP280_REG_NVM_PAR_P3_H        	0x93        /**< NVM PAR P3 high register */
#define BMP280_REG_NVM_PAR_P4_L        	0x94        /**< NVM PAR P4 low register */
#define BMP280_REG_NVM_PAR_P4_H        	0x95        /**< NVM PAR P4 high register */
#define BMP280_REG_NVM_PAR_P5_L        	0x96        /**< NVM PAR P5 low register */
#define BMP280_REG_NVM_PAR_P5_H        	0x97        /**< NVM PAR P5 high register */
#define BMP280_REG_NVM_PAR_P6_L        	0x98        /**< NVM PAR P6 low register */
#define BMP280_REG_NVM_PAR_P6_H        	0x99        /**< NVM PAR P6 high register */
#define BMP280_REG_NVM_PAR_P7_L        	0x9A        /**< NVM PAR P7 low register */
#define BMP280_REG_NVM_PAR_P7_H        	0x9B        /**< NVM PAR P7 high register */
#define BMP280_REG_NVM_PAR_P8_L        	0x9C        /**< NVM PAR P8 low register */
#define BMP280_REG_NVM_PAR_P8_H        	0x9D        /**< NVM PAR P8 high register */
#define BMP280_REG_NVM_PAR_P9_L        	0x9E        /**< NVM PAR P9 low register */
#define BMP280_REG_NVM_PAR_P9_H        	0x9F        /**< NVM PAR P9 high register */
#define BMP280_REG_ID                  	0xD0        /**< Chip id register */
#define BMP280_REG_RESET               	0xE0        /**< Soft reset register */
#define BMP280_REG_STATUS              	0xF3        /**< Status register */
#define BMP280_REG_CTRL_MEAS           	0xF4        /**< Control measurement register */
#define BMP280_REG_CONFIG              	0xF5        /**< Configuration register */
#define BMP280_REG_PRESS_MSB           	0xF7        /**< Pressure MSB register */
#define BMP280_REG_PRESS_LSB           	0xF8        /**< Pressure LSB register */
#define BMP280_REG_PRESS_XLSB          	0xF9        /**< Pressure XLSB register */
#define BMP280_REG_TEMP_MSB            	0xFA        /**< Temperature MSB register */
#define BMP280_REG_TEMP_LSB            	0xFB        /**< Temperature LSB register */
#define BMP280_REG_TEMP_XLSB           	0xFC        /**< Temperature XLSB register */

#define BMP280_RESET_VALUE     			0xB6

#define NUM_REG_COMPENSATION  			24

#define BMP280_CS_ACTIVE   				0
#define BMP280_CS_UNACTIVE  			1

typedef struct bmp280 {
	bmp280_opr_mode_t 			opr_mode;					/*!< Operating mode */
	bmp280_filter_t  			filter;						/*!< Filter */
	bmp280_over_sampling_t  	over_sampling_pressure;		/*!< Over sampling pressure */
	bmp280_over_sampling_t  	over_sampling_temperature;	/*!< Over sampling temperature */
	bmp280_over_sampling_t  	over_sampling_humidity;		/*!< Over sampling humidity */
	bmp280_standby_time_t  		standby_time;  				/*!< Standby time */
	bmp280_comm_mode_t   		comm_mode;					/*!< Comminication mode */
	bmp280_func_i2c_send       	i2c_send;        			/*!< Function I2C send */
	bmp280_func_i2c_recv       	i2c_recv;         			/*!< Function I2C receive */
	bmp280_func_spi_send  		spi_send;					/*!< Function SPI send */
	bmp280_func_spi_recv  		spi_recv;					/*!< Function SPI receive */
	bmp280_func_set_gpio   		set_cs;  					/*!< Function set pin CS */
	bmp280_func_delay          	delay;                 		/*!< Function delay function */
	uint16_t 					dig_T1;						/*!< Temperature compensation dig_T1 */
	int16_t  					dig_T2;						/*!< Temperature compensation dig_T2 */
	int16_t  					dig_T3;						/*!< Temperature compensation dig_T3 */
	uint16_t 					dig_P1;						/*!< Pressure compensation dig_P1 */
	int16_t  					dig_P2;						/*!< Pressure compensation dig_P2 */
	int16_t  					dig_P3;						/*!< Pressure compensation dig_P3 */
	int16_t  					dig_P4;						/*!< Pressure compensation dig_P4 */
	int16_t  					dig_P5;						/*!< Pressure compensation dig_P5 */
	int16_t  					dig_P6;						/*!< Pressure compensation dig_P6 */
	int16_t  					dig_P7;						/*!< Pressure compensation dig_P7 */
	int16_t  					dig_P8;						/*!< Pressure compensation dig_P8 */
	int16_t  					dig_P9;						/*!< Pressure compensation dig_P9 */
} bmp280_t;

static err_code_t bmp280_send(bmp280_handle_t handle, uint8_t reg_addr, uint8_t *buf_send, uint16_t len)
{
	if (handle->comm_mode == BMP280_COMM_MODE_I2C)
	{
		handle->i2c_send(reg_addr, buf_send, len);
	}
	else
	{
		if (handle->set_cs != NULL)
		{
			handle->set_cs(BMP280_CS_ACTIVE);
		}

		uint8_t buf[len + 1];
		buf[0] = reg_addr;
		memcpy(&buf[1], buf_send, len);

		handle->spi_send(buf, len + 1);

		if (handle->set_cs != NULL)
		{
			handle->set_cs(BMP280_CS_UNACTIVE);
		}
	}

	return ERR_CODE_SUCCESS;
}

static err_code_t bmp280_recv(bmp280_handle_t handle, uint8_t reg_addr, uint8_t *buf_recv, uint16_t len)
{
	if (handle->comm_mode == BMP280_COMM_MODE_I2C)
	{
		handle->i2c_send(reg_addr, buf_recv, len);
	}
	else
	{
		if (handle->set_cs != NULL)
		{
			handle->set_cs(BMP280_CS_ACTIVE);
		}

		uint8_t buf = reg_addr | 0x80;

		handle->spi_send(&buf, 1);
		handle->spi_recv(buf_recv, len);

		if (handle->set_cs != NULL)
		{
			handle->set_cs(BMP280_CS_UNACTIVE);
		}
	}

	return ERR_CODE_SUCCESS;
}

static err_code_t bmp280_compensate_temperature(bmp280_handle_t handle, int32_t adc_temp, float *temp, int32_t *fine_temp) {
	int32_t var1, var2;

	var1 = ((((adc_temp >> 3) - ((int32_t) handle->dig_T1 << 1))) * (int32_t) handle->dig_T2) >> 11;
	var2 = (((((adc_temp >> 4) - (int32_t) handle->dig_T1) * ((adc_temp >> 4) - (int32_t) handle->dig_T1)) >> 12) * (int32_t) handle->dig_T3) >> 14;

	*fine_temp = var1 + var2;
	*temp = ((*fine_temp * 5 + 128) >> 8) / 100;

	return ERR_CODE_SUCCESS;
}

static err_code_t bmp280_compensate_pressure(bmp280_handle_t handle, int32_t adc_press, int32_t fine_temp, float *pressure)
{
	int64_t var1, var2, p;

	var1 = (int64_t) fine_temp - 128000;
	var2 = var1 * var1 * (int64_t) handle->dig_P6;
	var2 = var2 + ((var1 * (int64_t) handle->dig_P5) << 17);
	var2 = var2 + (((int64_t) handle->dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t) handle->dig_P3) >> 8) + ((var1 * (int64_t) handle->dig_P2) << 12);
	var1 = (((int64_t) 1 << 47) + var1) * ((int64_t) handle->dig_P1) >> 33;

	/* Avoid exception caused by division by zero */
	if (var1 == 0)
	{
		return ERR_CODE_FAIL;
	}

	p = 1048576 - adc_press;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = ((int64_t) handle->dig_P9 * (p >> 13) * (p >> 13)) >> 25;
	var2 = ((int64_t) handle->dig_P8 * p) >> 19;

	*pressure = (((p + var1 + var2) >> 8) + ((int64_t) handle->dig_P7 << 4)) / 256;

	return ERR_CODE_SUCCESS;
}

bmp280_handle_t bmp280_init(void)
{
	bmp280_handle_t handle = calloc(1, sizeof(bmp280_t));
	if (handle == NULL)
	{
		return NULL;
	}

	return handle;
}

err_code_t bmp280_set_config(bmp280_handle_t handle, bmp280_cfg_t config)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	handle->opr_mode 					= config.opr_mode;
	handle->filter 						= config.filter;
	handle->over_sampling_pressure 		= config.over_sampling_pressure;
	handle->over_sampling_temperature 	= config.over_sampling_temperature;
	handle->over_sampling_humidity 		= config.over_sampling_humidity;
	handle->standby_time 				= config.standby_time;
	handle->comm_mode 					= config.comm_mode;
	handle->i2c_send 					= config.i2c_send;
	handle->i2c_recv 					= config.i2c_recv;
	handle->spi_send 					= config.spi_send;
	handle->spi_recv 					= config.spi_recv;
	handle->set_cs  					= config.set_cs;
	handle->delay 						= config.delay;

	return ERR_CODE_SUCCESS;
}

err_code_t bmp280_config(bmp280_handle_t handle)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	uint8_t cmd_data = 0;
	uint32_t timeout_ms = 500, delay_step_ms = 50;
	uint16_t dig_T1, dig_P1;
	int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

	/* Soft reset */
	cmd_data = BMP280_RESET_VALUE;
	bmp280_send(handle, BMP280_REG_RESET, &cmd_data, 1);

	/* Wait until finished copying over the NVP data */
	while (1)
	{
		uint8_t status;
		bmp280_recv(handle, BMP280_REG_STATUS, &status, 1);

		if ((status & 0x01) == 0)
		{
			break;
		}

		timeout_ms -= delay_step_ms;
		if (timeout_ms == 0)
		{
			return ERR_CODE_FAIL;
		}

		handle->delay(delay_step_ms);
	}

	/* Read temperature compenstation */
	uint8_t reg_comp[NUM_REG_COMPENSATION] = {0};
	bmp280_recv(handle, BMP280_REG_NVM_PAR_T1_L, reg_comp, NUM_REG_COMPENSATION);
	dig_T1 = (uint16_t)((reg_comp[1]  << 8) | reg_comp[0]);
	dig_T2 =  (int16_t)((reg_comp[3]  << 8) | reg_comp[2]);
	dig_T3 =  (int16_t)((reg_comp[5]  << 8) | reg_comp[4]);
	dig_P1 = (uint16_t)((reg_comp[7]  << 8) | reg_comp[6]);
	dig_P2 =  (int16_t)((reg_comp[9]  << 8) | reg_comp[8]);
	dig_P3 =  (int16_t)((reg_comp[11] << 8) | reg_comp[10]);
	dig_P4 =  (int16_t)((reg_comp[13] << 8) | reg_comp[12]);
	dig_P5 =  (int16_t)((reg_comp[15] << 8) | reg_comp[14]);
	dig_P6 =  (int16_t)((reg_comp[17] << 8) | reg_comp[16]);
	dig_P7 =  (int16_t)((reg_comp[19] << 8) | reg_comp[18]);
	dig_P8 =  (int16_t)((reg_comp[21] << 8) | reg_comp[20]);
	dig_P9 =  (int16_t)((reg_comp[23] << 8) | reg_comp[22]);

	/* Configure standby time and filter */
	cmd_data = (handle->standby_time << 5) | (handle->filter << 2);
	bmp280_send(handle, BMP280_REG_CONFIG, &cmd_data, 1);

	if (handle->opr_mode == BMP280_OPR_MODE_FORCED)
	{
		/* Initial mode for forced is sleep */
		handle->opr_mode = BMP280_OPR_MODE_SLEEP;
	}

	cmd_data = (handle->over_sampling_temperature << 5) | (handle->over_sampling_pressure << 2) | (handle->opr_mode);
	bmp280_send(handle, BMP280_REG_CTRL_MEAS, &cmd_data, 1);

	/* Update handle structure */
	handle->dig_T1 = dig_T1;
	handle->dig_T2 = dig_T2;
	handle->dig_T3 = dig_T3;
	handle->dig_P1 = dig_P1;
	handle->dig_P2 = dig_P2;
	handle->dig_P3 = dig_P3;
	handle->dig_P4 = dig_P4;
	handle->dig_P5 = dig_P5;
	handle->dig_P6 = dig_P6;
	handle->dig_P7 = dig_P7;
	handle->dig_P8 = dig_P8;
	handle->dig_P9 = dig_P9;

	return ERR_CODE_SUCCESS;
}

err_code_t bmp280_get_temperature(bmp280_handle_t handle, float *temperature)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	int32_t adc_temp, fine_temp;
	uint8_t reg_data[3];

	bmp280_recv(handle, BMP280_REG_TEMP_MSB, reg_data, 3);

	adc_temp = (reg_data[0] << 12) | (reg_data[1] << 4) | (reg_data[2] >> 4);
	bmp280_compensate_temperature(handle, adc_temp, temperature, &fine_temp);

	return ERR_CODE_SUCCESS;
}

err_code_t bmp280_get_pressure(bmp280_handle_t handle, float *pressure)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	int32_t adc_temp, fine_temp, adc_pressure;
	float temperature;
	uint8_t reg_data[6];

	bmp280_recv(handle, BMP280_REG_PRESS_MSB, reg_data, 6);

	adc_pressure = (reg_data[0] << 12) | (reg_data[1] << 4) | (reg_data[2] >> 4);
	adc_temp 	 = (reg_data[3] << 12) | (reg_data[4] << 4) | (reg_data[5] >> 4);

	bmp280_compensate_temperature(handle, adc_temp, &temperature, &fine_temp);
	bmp280_compensate_pressure(handle, adc_pressure, fine_temp, pressure);

	return ERR_CODE_SUCCESS;
}