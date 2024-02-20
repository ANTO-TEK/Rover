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
 * @file smoothed_ps2x_analog.h
 * @brief Header file for the smoothed analog values module.
 *
 * This module provides functions and definitions for smoothing analog values,
 * specifically designed for use with PS2X analog inputs. It includes
 * a function that translates raw analog values into commands to be given to the motor
 * driver. Smoothed adjustments prevent sudden changes in motor speed,
 * allowing for a smoother control experience. This library is compliant with the MISRA C:2012 Guidelines,
 * ensuring safety, portability, and reliability in embedded systems. Code analysis has been
 * performed using cppcheck for static code analysis to ensure code quality.
 *
 * @version 1.2
 * @date Dec 27, 2023
 * @author Adinolfi Teodoro, Amato Emilio, Bove Antonio, Guarini Alessio
 */

#ifndef INC_SMOOTHED_PS2X_ANALOG_H_
#define INC_SMOOTHED_PS2X_ANALOG_H_

#include "geometry.h" // Include for Cartesian2D type

// Maximum increment and decrement steps for X and Y axes
#define MAX_INCREMENT_STEP_X (4U) ///< Maximum increment step for X-axis.
#define MAX_DECREMENT_STEP_X (50U) ///< Maximum decrement step for X-axis.
#define MAX_INCREMENT_STEP_Y (4U) ///< Maximum increment step for Y-axis.
#define MAX_DECREMENT_STEP_Y (50U) ///< Maximum decrement step for Y-axis.


typedef uint8_t SmoothedAnalog_StatusTypeDef;

#define SA_OK    (0U) ///< Operation successful.
#define SA_ERROR (1U) ///< Operation encountered an error.

/**
 * @brief Calculates smoothed motor drive inputs based on joystick input.
 *
 * This function processes Cartesian2D analog input values to determine appropriate
 * motor drive input for a 4-motor drive system, facilitating complex maneuvers such
 * as forward/backward movement, pivoting, and turning. The function ensures that output
 * values are safely limited to a specified range and applies smoothing to transition
 * between values gradually, enhancing control precision and reliability.
 *
 * @param[in] analog The current analog input values from the joystick.
 * @param[in] current_l_wheel The current value of the left wheel motor.
 * @param[in] current_r_wheel The current value of the right wheel motor.
 * @param[out] l_wheel Pointer to an integer to store the calculated left wheel motor value.
 * @param[out] r_wheel Pointer to an integer to store the calculated right wheel motor value.
 * @param[in] max_output_forward The maximum output value for forward motion to be given as input to the motor driver.
 * @param[in] max_output_backward The maximum output value for backward motion to be given as input to the motor driver.
 * @param[in] max_increment Maximum increment step for smoothing transitions.
 * @param[in] max_decrement Maximum decrement step for smoothing transitions.
 * @return SmoothedAnalog_StatusTypeDef Status of the smoothed analog calculation, indicating either
 *                                      success (SA_OK) or failure (SA_ERROR).
 * @note The function's implementation is optimized for responsiveness and stability, ensuring
 *       that the vehicle can execute smooth and controlled movements in response to user input.
 */
SmoothedAnalog_StatusTypeDef smoothed_4_motors_drive(	Cartesian2D analog, int8_t current_l_wheel, int8_t current_r_wheel,
														float max_output_forward, float max_output_backward,
														uint8_t max_increment, uint8_t max_decrement,
														int8_t *l_wheel, int8_t* r_wheel);

#endif /* INC_SMOOTHED_PS2X_ANALOG_H_ */
