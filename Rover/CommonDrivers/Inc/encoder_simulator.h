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
 * @file encoder_simulator.h
 * @brief Header file for the Encoder Simulator module.
 *
 * This module simulates an encoder using a PWM signal and integrates with the HAL TIM timer.
 * It supports initialization and speed setting for the simulated encoder. Enhanced for robustness and clarity.
 *
 * @date Created on: Jan 1, 2024
 * @author Alewi
 */

#ifndef ENCODER_SIMULATOR_H_
#define ENCODER_SIMULATOR_H_

#include "tim.h"

/** @defgroup EncoderSimulatorStatus Encoder Simulator Status Definitions
 *  @{
 */
typedef uint8_t EncoderSimulator_StatusTypeDef;

#define ENCODER_SIM_OK  								((EncoderSimulator_StatusTypeDef) 0) /*!< Encoder simulator OK status */
#define ENCODER_SIM_ERROR    							((EncoderSimulator_StatusTypeDef) 1)/*!< Encoder simulator error status */

/** @} */

/**
 * @struct encoder_simulator_t
 * @brief Represents an encoder simulator.
 *
 * Holds PWM timer handle, timer channel, pulses per revolution, and gear ratio.
 */
typedef struct {
    TIM_HandleTypeDef *pwm_timer; /*!< PWM timer handle */
    uint32_t tim_channel;         /*!< Timer channel for PWM */
    uint16_t ppr;                 /*!< Pulses per revolution */
    uint8_t gear_ratio;           /*!< Gear ratio */
} encoder_simulator_t;

/**
 * @brief Initializes the encoder simulator.
 *
 * Sets up an encoder simulator using the specified PWM timer and channel.
 * Validates the inputs and initializes the encoder_simulator_t structure.
 *
 * @param encoder_simulator Pointer to the encoder_simulator_t structure.
 * @param pwm_timer Pointer to the HAL TIM timer handle for PWM.
 * @param tim_channel Timer channel for PWM.
 * @param ppr Pulses per revolution, must be non-zero.
 * @param gear_ratio Gear ratio, must be non-zero.
 * @return EncoderSimulator_StatusTypeDef Status of initialization.
 */
EncoderSimulator_StatusTypeDef encoder_simulator_init(encoder_simulator_t *encoder_simulator, TIM_HandleTypeDef *pwm_timer,
														uint32_t tim_channel, uint8_t ppr, uint8_t gear_ratio);

/**

@brief Sets the speed for the encoder simulator.
Adjusts the PWM signal to simulate the encoder's speed. The speed parameter
represents the velocity of the wheel after gear reduction.
Validates the speed to be within an acceptable range.
@param encoder_simulator Pointer to the initialized encoder_simulator_t structure.
@param speed Target speed for the wheel, must be non-negative.
@return EncoderSimulator_StatusTypeDef Status of speed setting.
*/
EncoderSimulator_StatusTypeDef encoder_simulator_set_speed(encoder_simulator_t *encoder_simulator, float speed);
#endif /* ENCODER_SIMULATOR_H_ */
