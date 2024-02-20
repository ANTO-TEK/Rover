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
 * @file encoder.h
 * @author Adinolfi Teodoro, Amato Emilio, Bove Antonio, Guarini Alessio
 * @brief Header file for the encoder module, providing interfaces and definitions for encoder handling.
 *        This module offers functionalities for initializing and managing encoders, including reading
 *        and interpreting the encoder signals for rotational speed and direction.
 * @version 0.2
 * @date 2023-12-13
 * 
 * @note This header should be used in conjunction with STM32F4xx HAL libraries.
 * @warning Ensure that the included "stm32f4xx_hal.h" matches your specific MCU configuration.
 *
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "stm32f4xx_hal.h" // Specific MCU HAL library. Replace if using a different MCU family.

typedef uint8_t Encoder_StatusTypeDef;

/* Encoder Status Definitions */
#define ENCODER_OK    								((Encoder_StatusTypeDef) 0U) ///< Encoder operation successful.
#define ENCODER_ERROR 								((Encoder_StatusTypeDef) 1U) ///< Encoder operation encountered an error.

/* Constant Definitions */
#define GEAR_RATIO 						((float) 51.0f) ///< Gear ratio of the encoder system.
#define PPR        						((float) 12.0f) ///< Pulses Per Revolution (PPR) of the encoder.
#define CPR        						(2*PPR) 		///< Pulses Per Revolution (PPR) of the encoder.


typedef enum {
    DEFAULT_DIRECTION  = 0,
    BACKWARD_DIRECTION = 1,
    FORWARD_DIRECTION  = 2
}EncoderRotationEnum_t;

/**
 * @brief Structure representing an encoder.
 *
 * This structure holds the state and configuration of an encoder, including its associated timer,
 * channels, count, direction, and calculated RPM.
 */
typedef struct {
    TIM_HandleTypeDef *timer;      								///< Timer handle used for counting encoder pulses.
    uint8_t n_channels;            								///< Number of active channels of the encoder.
    uint32_t counter1;           								///< Encoder counter 1.
    uint32_t counter2;           								///< Encoder counter 2.
    EncoderRotationEnum_t rotation_status;               		///< Direction of rotation: 1 = forward, 0 = backward.
    float last_rpm;                								///< Last calculated RPM value.
    uint32_t apb_timer_clock;
} encoder_t;

/**
 * @brief Initializes an encoder structure.
 *
 * Sets up the specified encoder with the provided hardware configurations. Configures the timer
 * and channel for pulse counting and calculates the RPM based on the pulses.
 *
 * @param encoder Pointer to the encoder structure to be initialized.
 * @param timer Pointer to the timer handle associated with the encoder.
 * @param n_channels Number of active channels of the encoder.
 * @return Encoder_StatusTypeDef Status of the initialization (ENCODER_OK or ENCODER_ERROR).
 */
Encoder_StatusTypeDef encoder_init(encoder_t *encoder, TIM_HandleTypeDef *timer);

#endif /* INC_ENCODER_H_ */
