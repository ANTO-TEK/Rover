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

/**
 * @file encoder_manager.h
 * @brief Header file for the encoder manager module.
 *
 * This module is designed for managing encoders in a microcontroller-based system, particularly
 * useful in applications requiring precise motion control and position tracking. It includes
 * functionality for initializing the encoder manager, updating encoder readings, and calculating RPM.
 * It supports multiple encoders and can be configured for various sampling rates.
 *
 * @date Jan 1, 2024
 * @author Adinolfi Teodoro, Amato Emilio, Bove Antonio, Guarini Alessio
 */

#ifndef INC_ENCODER_MANAGER_H_
#define INC_ENCODER_MANAGER_H_

/* Includes */
#include "stm32f4xx_hal.h" 
#include "encoder.h"

#define ENCODER_NUM (4U)   /**< Define the number of encoder */

/* Status Definition */
/**
 * @brief Encoder manager status type definition.
 *
 * This type is used to represent the status of operations within the encoder manager module,
 * indicating success or error states.
 */
typedef uint8_t EncoderManager_StatusTypeDef;

/**
 * @brief Encoder manager operation successful.
 */
#define ENCODER_MANAGER_OK ((EncoderManager_StatusTypeDef) 0U)

/**
 * @brief Encoder manager operation encountered an error.
 */
#define ENCODER_MANAGER_ERROR ((EncoderManager_StatusTypeDef) 1U)

typedef struct {
    TIM_HandleTypeDef **encoders_timer;     /**< Pointer to array of encoders.*/
    encoder_t encoders[ENCODER_NUM];        /**< Array of encoders.*/
}encoder_manager_config_t;

/* Function Prototypes */

EncoderManager_StatusTypeDef encoder_manager_init(encoder_manager_config_t *config);
/**
 * @brief Update the RPM readings of all encoders.
 *
 * Called to update the RPM (Revolutions Per Minute) readings of each encoder. This function
 * calculates the RPM based on the latest encoder values and the time elapsed since the last update.
 * It is typically invoked in the context of a timer interrupt handler.
 */
void encoder_manager_update_rpm(float *result);

#endif /* INC_ENCODER_MANAGER_H_ */
