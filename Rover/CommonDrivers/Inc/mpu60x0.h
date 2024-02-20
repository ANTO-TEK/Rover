/**
 * @file mpu60x0.h
 * @author Adinolfi Teodoro, Amato Emilio, Bove Antonio, Guarini Alessio
 * @brief  Driver for the MPU60X0 sensor.
 * @version 1.0
 * @date 2024-02-15
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef INC_MPU60X0_H_
#define INC_MPU60X0_H_


#include "i2c.h"

#include "geometry.h"

/**
 * @brief Structure to hold the needed configuration for interfacing the MPU60X0 sensor
 *        with the STM32 HAL library
 */
typedef struct {
	I2C_HandleTypeDef *hi2c;
} MPU60X0_config_t;

/**
 * @brief This structure holds the configuration and state of the MPU60X0 sensor
 */
typedef struct {
	MPU60X0_config_t *config;
	Cartesian3D gyro, accel;
} MPU60X0_t;

#include <mpu60x0_constants.h>


MPU60X0_StatusTypeDef MPU60X0_init(MPU60X0_t *mpu60x0, const MPU60X0_config_t* mpu60x0_config, uint32_t Timeout);

/**
  * @brief  Read the WHO_AM_I register of the MPU60X0 sensor
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  result Pointer to the buffer to store the result
  * @param  readFunc Function pointer specifying the I2C read method to use (Polling, DMA, IT)
  * @param  timeout Timeout for the operation. Ignored if DMA or IT is used.
  * @retval HAL status of the operation
  */
MPU60X0_StatusTypeDef MPU60X0_I2C_who_am_i(MPU60X0_t *mpu60x0, uint8_t *result, uint32_t Timeout);


/**
 * @brief Configure the MPU60X0 sensor used during the initialization
 * 
 * @param mpu60x0 structure containing the configuration and state of the MPU60X0 sensor
 * @param Timeout maximum time to wait for the operation to complete
 * @return MPU60X0_StatusTypeDef MPU60X0_OK if the operation is successful, MPU60X0_ERROR otherwise
 */
MPU60X0_StatusTypeDef MPU60X0_I2C_reconfig(MPU60X0_t *mpu60x0, uint32_t Timeout);

/**
 * @brief Read the gyro values from the MPU60X0 sensor
 * 
 * @param mpu60x0 structure containing the configuration and state of the MPU60X0 sensor
 * @param result a pointer to a Cartesian3D structure to store the result
 * @return MPU60X0_StatusTypeDef MPU60X0_OK if the operation is successful, MPU60X0_ERROR otherwise
 */
MPU60X0_StatusTypeDef MPU60X0_get_gyro_value(MPU60X0_t *mpu60x0, Cartesian3D *result);

/**
 * @brief Read the accelerometer values from the MPU60X0 sensor
 * 
 * @param mpu60x0 structure containing the configuration and state of the MPU60X0 sensor
 * @param result a pointer to a Cartesian3D structure to store the result
 * @return MPU60X0_StatusTypeDef MPU60X0_OK if the operation is successful, MPU60X0_ERROR otherwise
 */
MPU60X0_StatusTypeDef MPU60X0_get_accel_value(MPU60X0_t *mpu60x0, Cartesian3D *result);

#endif /* INC_MPU60X0_H_ */
