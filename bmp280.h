// MIT License

// Copyright (c) 2024 phonght32

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef __BMP280_H__
#define __BMP280_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "err_code.h"

#define BMP280_I2C_ADDR_0			0x76
#define BMP280_I2C_ADDR_1			0x77

typedef err_code_t (*bmp280_func_i2c_send)(uint8_t reg_addr, uint8_t *buf_send, uint16_t len);
typedef err_code_t (*bmp280_func_i2c_recv)(uint8_t reg_addr, uint8_t *buf_recv, uint16_t len);
typedef err_code_t (*bmp280_func_spi_send)(uint8_t *buf_send, uint16_t len);
typedef err_code_t (*bmp280_func_spi_recv)(uint8_t *buf_recv, uint16_t len);
typedef err_code_t (*bmp280_func_set_gpio)(uint8_t level);
typedef void (*bmp280_func_delay)(uint32_t ms);

/**
 * @brief   Handle structure.
 */
typedef struct bmp280 *bmp280_handle_t;

/**
 * @brief   Operatting mode.
 */
typedef enum {
	BMP280_OPR_MODE_SLEEP = 0,						/*!< Sleep mode */
	BMP280_OPR_MODE_FORCED = 1,						/*!< Measurement is initiated by user */
	BMP280_OPR_MODE_NORMAL = 3						/*!< Continoues measurement */
} bmp280_opr_mode_t;

/**
 * @brief   Filter.
 */
typedef enum {
	BMP280_FILTER_OFF = 0,							/*!< Filter off */
	BMP280_FILTER_2,								/*!< Filter 2 */
	BMP280_FILTER_4,								/*!< Filter 4 */
	BMP280_FILTER_8,								/*!< Filter 8 */
	BMP280_FILTER_16								/*!< Filter 16 */
} bmp280_filter_t;

/**
 * @brief   Over sampling.
 */
typedef enum {
	BMP280_OVER_SAMPLING_SKIPPED = 0,				/*!< No measurement  */
	BMP280_OVER_SAMPLING_ULTRA_LOW_POWER,			/*!< Oversampling x1 */
	BMP280_OVER_SAMPLING_LOW_POWER,					/*!< Oversampling x2 */
	BMP280_OVER_SAMPLING_STANDARD,					/*!< Oversampling x4 */
	BMP280_OVER_SAMPLING_HIGH_RES,					/*!< Oversampling x8 */
	BMP280_OVER_SAMPLING_ULTRA_HIGH_RES,			/*!< Oversampling x16 */
} bmp280_over_sampling_t;

/**
 * @brief   Standby time.
 */
typedef enum {
	BMP280_STANDBY_TIME_0_5MS = 0,					/*!< Stand by time 0.5ms */
	BMP280_STANDBY_TIME_62_5MS,						/*!< Stand by time 62.5ms */
	BMP280_STANDBY_TIME_125_0MS,					/*!< Stand by time 125ms */
	BMP280_STANDBY_TIME_250_0MS,					/*!< Stand by time 250ms */
	BMP280_STANDBY_TIME_500_0MS,					/*!< Stand by time 500ms */
	BMP280_STANDBY_TIME_1000_0MS,					/*!< Stand by time 1s */
	BMP280_STANDBY_TIME_2000_0MS,					/*!< Stand by time 2s */
	BMP280_STANDBY_TIME_4000_0MS,					/*!< Stand by time 4s */
} bmp280_standby_time_t;

/**
 * @brief   Communication mode.
 */
typedef enum {
	BMP280_COMM_MODE_I2C = 0,
	BMP280_COMM_MODE_SPI
} bmp280_comm_mode_t;

/**
 * @brief   Configuration structure.
 */
typedef struct {
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
} bmp280_cfg_t;

/*
 * @brief   Initialize BMP280 with default parameters.
 *
 * @note    This function must be called first.
 *
 * @param   None.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
bmp280_handle_t bmp280_init(void);

/*
 * @brief   Set configuration parameters.
 *
 * @param 	handle Handle structure.
 * @param   config Configuration structure.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t bmp280_set_config(bmp280_handle_t handle, bmp280_cfg_t config);

/*
 * @brief   Configure BMP280 to run.
 *
 * @param 	handle Handle structure.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t bmp280_config(bmp280_handle_t handle);

/*
 * @brief   Get pressure in Pascal.
 *
 * @param 	handle Handle structure.
 * @param 	pressure Pressure.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t bmp280_get_pressure(bmp280_handle_t handle, float *pressure);

/*
 * @brief   Get altitude in cm.
 *
 * @param 	handle Handle structure.
 * @param 	pressure Pressure.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t bmp280_get_altitude(bmp280_handle_t handle, float *altitude);


#ifdef __cplusplus
}
#endif

#endif /* __BMP280_H__ */