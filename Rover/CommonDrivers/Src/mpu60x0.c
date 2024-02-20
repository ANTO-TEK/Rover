/* 
 * Course: IoT and embedded system design 2023/2024
 * 
 * Lecturer: Francesco Moscato	fmoscato@unisa.it
 *
 * Group:
 * Adinolfi Teodoro    0622701902	   t.adinolfi2@studenti.unisa.it
 * Amato Emilio        0622701903      e.amato16@studenti.unisa.it             
 * Bove Antonio        0622701898      a.bove57@studenti.unisa.it 
 * Guarini Alessio     0622702042	   g.guarini7@studenti.unisa.it
 *
 * Copyright (C) 2024 - All Rights Reserved
 *
 * This file is part of CommonDrivers.
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of 
 * the GNU General Public License as published by the Free Software Foundation, either version 
 * 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ContestOMP. 
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "mpu60x0.h"
#include "stm32f4xx_hal.h"
/*
 * PRIVATE FUNCTIONS SECTION
 */
static inline MPU60X0_StatusTypeDef __read_register(MPU60X0_t *mpu60x0, uint8_t mem_address, uint8_t *result, uint8_t size, uint32_t Timeout){
	MPU60X0_StatusTypeDef status = MPU60X0_ERROR;
	if( HAL_I2C_Mem_Read(mpu60x0->config->hi2c, MPU60X0_ADDRESS, mem_address, 1, result, size, Timeout) == HAL_OK ){
		status = MPU60X0_OK;
	}
	return status;
}

static inline MPU60X0_StatusTypeDef __write_register(MPU60X0_t *mpu60x0, uint8_t mem_address, uint8_t *data, uint8_t size, uint32_t Timeout){
	MPU60X0_StatusTypeDef status = MPU60X0_ERROR;
	if( HAL_I2C_Mem_Write(mpu60x0->config->hi2c, MPU60X0_ADDRESS, mem_address, 1, data, size, Timeout) == HAL_OK ){
		status = MPU60X0_OK;
	}
	return status;
}

static inline MPU60X0_StatusTypeDef __reset_device(MPU60X0_t *mpu60x0, uint32_t Timeout){
	MPU60X0_StatusTypeDef status = MPU60X0_ERROR;

	static uint8_t reset_command = DEVICE_RESET;
	static const uint8_t pwr_mgmt_1_default_value = 0x40;
	uint8_t tries = 255;
	uint8_t result = 0xFF;
	uint8_t wait = 0;

	if(__write_register(mpu60x0, PWR_MGMT_1, &reset_command, 1, Timeout) == MPU60X0_OK){

		do {
			if(	__read_register(mpu60x0, PWR_MGMT_1, &result, 1, Timeout) == MPU60X0_OK &&
				result == pwr_mgmt_1_default_value){
				HAL_Delay(wait++);
				status = MPU60X0_OK;
			}

		} while(result != pwr_mgmt_1_default_value && --tries);
	}
	return status;
}

/*
 *  Wake up the device and set the internal clock source.
 *  @WARNING: This function does active waiting
 */
typedef enum {
    CLKSEL_0 = 0b00000000,  // 0U
    CLKSEL_1 = 0b00000001,  // 1U
    CLKSEL_2 = 0b00000010,  // 2U
    CLKSEL_3 = 0b00000011,  // 3U
    CLKSEL_4 = 0b00000100,  // 4U
    CLKSEL_5 = 0b00000101,  // 5U
    CLKSEL_6 = 0b00000110,  // 6U (Reserved, ma definito per completezza)
    CLKSEL_7 = 0b00000111   // 7U
} CLKSEL_value_t;

static inline MPU60X0_StatusTypeDef __set_internal_clock_source(MPU60X0_t *mpu60x0, uint32_t Timeout){
	MPU60X0_StatusTypeDef status = MPU60X0_ERROR;
    //possibilities: CLKSEL_0, CLKSEL_1, CLKSEL_2, CLKSEL_3, CLKSEL_4, CLKSEL_5, CLKSEL_7. No, there isn't CLKSEL_6.
	static uint8_t command = CLKSEL_0;
    if(__write_register(mpu60x0, PWR_MGMT_1, &command, 1, Timeout) == MPU60X0_OK){
    	HAL_Delay(1);
    	status = MPU60X0_OK;
    }
    return status;
}


typedef enum {
    DLPF_CFG_0 = 0b00000000,  // 0U
    DLPF_CFG_1 = 0b00000001,  // 1U
    DLPF_CFG_2 = 0b00000010,  // 2U
    DLPF_CFG_3 = 0b00000011,  // 3U
    DLPF_CFG_4 = 0b00000100,  // 4U
    DLPF_CFG_5 = 0b00000101,  // 5U
    DLPF_CFG_6 = 0b00000110,  // 6U
    DLPF_CFG_7 = 0b00000111   // 7U (Reserved, ma definito per completezza)
} __DLPF_value_t;


static inline MPU60X0_StatusTypeDef __set_DLPF(MPU60X0_t *mpu60x0, uint32_t Timeout){
	MPU60X0_StatusTypeDef status = MPU60X0_ERROR;
	uint8_t reg_status = 0xFF;
	if(__read_register(mpu60x0, CONFIG, &reg_status, 1, Timeout) == MPU60X0_OK){
		reg_status = (reg_status & 0b11111000) | DLPF_CFG_4;
		if(__write_register(mpu60x0, CONFIG, &reg_status, 1, Timeout) == MPU60X0_OK){
			status = MPU60X0_OK;
		}
	}
	return status;
}


typedef enum {
    FS_SEL_0 = 0b00000000,  // 0U
	FS_SEL_1 = 0b00001000,  // 1U
	FS_SEL_2 = 0b00010000,  // 2U
	FS_SEL_3 = 0b00011000,  // 3U
} __gyro_scale_value_t;

static inline MPU60X0_StatusTypeDef __set_gyro_scale(MPU60X0_t *mpu60x0, uint32_t Timeout){
	MPU60X0_StatusTypeDef status = MPU60X0_ERROR;
	uint8_t reg_status = 0xFF;

	if(__read_register(mpu60x0, GYRO_CONFIG, &reg_status, 1, Timeout) == MPU60X0_OK){
			reg_status = (reg_status & ~0b00011000) | FS_SEL_0;
			if(__write_register(mpu60x0, GYRO_CONFIG, &reg_status, 1, Timeout) == MPU60X0_OK){
				status = MPU60X0_OK;
			}
	}

	return status;
}


typedef enum {
    AFS_SEL_0 = 0b00000000,  // 0U
	AFS_SEL_1 = 0b00001000,  // 1U
	AFS_SEL_2 = 0b00010000,  // 2U
	AFS_SEL_3 = 0b00011000,  // 3U
} __accelerometer_scale_value_t;

static inline MPU60X0_StatusTypeDef __set_accelerometer_scale(MPU60X0_t *mpu60x0, uint32_t Timeout){
	MPU60X0_StatusTypeDef status = MPU60X0_ERROR;
	uint8_t reg_status = 0xFF;

	if(__read_register(mpu60x0, ACCEL_CONFIG, &reg_status, 1, Timeout) == MPU60X0_OK){
			reg_status = (reg_status & ~0b00011000) | AFS_SEL_0;
			if(__write_register(mpu60x0, ACCEL_CONFIG, &reg_status, 1,  Timeout) == MPU60X0_OK){
				status = MPU60X0_OK;
			}
	}
    return status;
}


/**
 * @brief Set the sample rate divider for the MPU-60x0.
 *
 * This method sets the sample rate divider for the MPU-60x0. The sample rate
 * is determined by dividing the gyroscope output rate by the provided
 * SMPLRT_DIV value. The gyroscope output rate is 8 kHz when the Digital Low
 * Pass Filter (DLPF) is disabled, and 1 kHz when the DLPF is enabled.
 *
 * @note The accelerometer output rate is 1 kHz. Therefore, for sample rates
 * greater than 1 kHz, the same accelerometer sample may be output multiple times.
 *
 * @param mpu60x0 Pointer to the MPU60X0_t structure that contains the configuration
 *                information for the MPU-60x0 device.
 * @param smplrt_div The divider value to set for the sample rate. It is an 8-bit
 *                   unsigned value. The actual sample rate is calculated as:
 *                   Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV).
 * @param Timeout Timeout duration for the write operation.
 *
 * @return MPU60X0_StatusTypeDef status of the operation. MPU60X0_OK is returned if
 *         the operation is successful, otherwise MPU60X0_ERROR.
 */
static inline MPU60X0_StatusTypeDef __set_sample_rate_divider(MPU60X0_t *mpu60x0, uint8_t smplrt_div, uint32_t Timeout){
    MPU60X0_StatusTypeDef status = MPU60X0_ERROR;

    if(__write_register(mpu60x0, SMPRT_DIV, &smplrt_div, 1, Timeout) == MPU60X0_OK){
        status = MPU60X0_OK;
    }
    return status;
}


typedef enum {
	FIFO_OFLOW_EN = 0b00010000,  // 0U
	I2C_MST_INT_EN  = 0b00001000,  // 1U
	DATA_RDY_EN = 0b00000001,  // 2U
} INT_ENABLE_value_t;

static inline MPU60X0_StatusTypeDef __enable_device_interrupt(MPU60X0_t *mpu60x0, INT_ENABLE_value_t interrupts,  uint32_t Timeout){
	MPU60X0_StatusTypeDef status = MPU60X0_ERROR;
	uint8_t reg_status = 0xFF;
	if(__read_register(mpu60x0, INT_ENABLE, &reg_status, 1, Timeout) == MPU60X0_OK){
			reg_status = reg_status  | interrupts;
			if(__write_register(mpu60x0, INT_ENABLE, &reg_status, 1, Timeout) == MPU60X0_OK){
				status = MPU60X0_OK;
			}
	}
	return status;
}



/*
 * PUBLIC FUNCTIONS SECTION
 */

MPU60X0_StatusTypeDef MPU60X0_init(MPU60X0_t *mpu60x0, const MPU60X0_config_t* mpu60x0_config, uint32_t Timeout){
	MPU60X0_StatusTypeDef status = MPU60X0_ERROR;
	mpu60x0->config = mpu60x0_config;
	if (MPU60X0_I2C_reconfig(mpu60x0, Timeout) == MPU60X0_OK){
		mpu60x0->accel.x = mpu60x0->accel.y = mpu60x0->accel.z = 0U;
		mpu60x0->gyro.x = mpu60x0->gyro.y = mpu60x0->gyro.z = 0U;
		status = MPU60X0_OK;
	}
	return status;
}

MPU60X0_StatusTypeDef MPU60X0_I2C_who_am_i(MPU60X0_t *mpu60x0, uint8_t *result, uint32_t Timeout) {
	MPU60X0_StatusTypeDef status = MPU60X0_ERROR;
    static const uint8_t default_who_am_i_reg_value = 0x68;
	if (__read_register(mpu60x0, WHO_AM_I, result, 1, Timeout) == MPU60X0_OK) {
        *result = (*result & 0b01111110);
        if(*result == default_who_am_i_reg_value){
        	status = MPU60X0_OK;
        }
    }
    return status;
}

MPU60X0_StatusTypeDef MPU60X0_I2C_reconfig(MPU60X0_t *mpu60x0, uint32_t Timeout) {
	MPU60X0_StatusTypeDef status = MPU60X0_ERROR;

	if( __reset_device(mpu60x0,Timeout) == MPU60X0_OK &&
		__set_internal_clock_source(mpu60x0,Timeout) == MPU60X0_OK &&
		__set_DLPF(mpu60x0, Timeout) == MPU60X0_OK &&
		__set_sample_rate_divider(mpu60x0, DEFAULT_SMPLRT_DIV, Timeout) == MPU60X0_OK &&
		__set_gyro_scale(mpu60x0, Timeout) == MPU60X0_OK &&
		__set_accelerometer_scale(mpu60x0, Timeout) == MPU60X0_OK &&
		__enable_device_interrupt(mpu60x0, DATA_RDY_EN | I2C_MST_INT_EN | FIFO_OFLOW_EN, Timeout) == MPU60X0_OK){

		status = MPU60X0_OK;

	}
    return status;
}


MPU60X0_StatusTypeDef MPU60X0_get_gyro_value(MPU60X0_t *mpu60x0, Cartesian3D *result){
	MPU60X0_StatusTypeDef status = MPU60X0_ERROR;
	uint8_t rec_data[6]; // Secondo Alessio e Teo sembra idoneo riciclare un'area di memoria già allocata per un maggior determinismo.
	int16_t gyro_x_raw = 0,
			gyro_y_raw = 0,
			gyro_z_raw = 0;
	//COMUNICATION TIME: 270us
	if(__read_register(mpu60x0, GYRO_XOUT_H, rec_data, 6, HAL_MAX_DELAY) == MPU60X0_OK){
		//FLOATING CONVETSION TIME: 38us
		gyro_x_raw = (int16_t)(rec_data[0] << 8 | rec_data[1]);
		gyro_y_raw = (int16_t)(rec_data[2] << 8 | rec_data[3]);
		gyro_z_raw = (int16_t)(rec_data[4] << 8 | rec_data[5]);
		result->x = gyro_x_raw/131.0;
		result->y = gyro_y_raw/131.0;
		result->z = gyro_z_raw/131.0;
		//
		status = MPU60X0_OK;
	}

	return status;
}

MPU60X0_StatusTypeDef MPU60X0_get_accel_value(MPU60X0_t *mpu60x0, Cartesian3D *result){
	MPU60X0_StatusTypeDef status = MPU60X0_ERROR;

	uint8_t rec_data[6]; // Secondo Alessio e Teo sembra idoneo riciclare un'area di memoria già allocata per un maggior determinismo.
	int16_t accel_x_raw = 0,
			accel_y_raw = 0,
			accel_z_raw = 0;

	if(__read_register(mpu60x0, ACCEL_XOUT_H, rec_data, 6, HAL_MAX_DELAY) == MPU60X0_OK){

		accel_x_raw = (int16_t)(rec_data[0] << 8 | rec_data[1]);
		accel_y_raw = (int16_t)(rec_data[2] << 8 | rec_data[3]);
		accel_z_raw = (int16_t)(rec_data[4] << 8 | rec_data[5]);

		result->x = accel_x_raw/16384.0;
		result->y = accel_y_raw/16384.0;
		result->z = accel_z_raw/16384.0;

		status = MPU60X0_OK;

	}

	return status;
}

