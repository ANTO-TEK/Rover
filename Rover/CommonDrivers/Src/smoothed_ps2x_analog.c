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
 * @file smoothed_ps2x_analog.c
 * @brief Implementation file for the smoothed analog values module.
 *
 * Defines the functions declared in smoothed_ps2x_analog.h for initializing
 * and updating smoothed analog values.
 *
 * @date Dec 27, 2023
 */

#ifndef SRC_SMOOTHED_PS2X_ANALOG_C_
#define SRC_SMOOTHED_PS2X_ANALOG_C_

#include "smoothed_ps2x_analog.h"
#include <stdlib.h>
#include <math.h>
#include <stdint.h>

/**
 * @brief Clamps a value between a specified minimum and maximum range.
 *
 * @param value The value to be clamped.
 * @param min The minimum allowable value.
 * @param max The maximum allowable value.
 * @return The clamped value.
 */
static inline float __clamp(float value, float min, float max) {
    float clamped_value;

    if (value > max) {
        clamped_value = max;
    } else if (value < min) {
        clamped_value = min;
    } else {
        clamped_value = value;
    }
    return clamped_value;
}

/**
 * @brief Calculates the new smoothed value.
 *
 * This function calculates the new smoothed value based on the current smoothed value,
 * the raw input value, and the maximum increment and decrement steps.
 *
 * @param current_smoothed The current smoothed value.
 * @param current_raw The current raw input value.
 * @param max_increment The maximum increment step.
 * @param max_decrement The maximum decrement step.
 * @return int8_t The new smoothed value.
 */
static inline int8_t __calculate_new_smoothed_value(int8_t current_smoothed, int8_t current_raw, uint8_t max_increment, uint8_t max_decrement) {
	int16_t current_smoothed_16 = (int16_t)current_smoothed;
    int16_t current_raw_16 = (int16_t)current_raw;
    int16_t max_increment_16 = (int16_t)max_increment;
    int16_t max_decrement_16 = (int16_t)max_decrement;

    int16_t error = current_raw_16 - current_smoothed_16;
    int16_t new_smoothed_16 = 0;
    if (error != 0) {
        static const int16_t positive_sign = 1;
        static const int16_t negative_sign = -1;
    	int16_t error_sign = (error < 0) ? negative_sign : positive_sign;
        int16_t adjustment = 0;
        if (((current_smoothed_16 > 0) && (error < 0)) || ((current_smoothed_16 < 0) && (error > 0))) {
            adjustment = (abs(current_smoothed_16) <= max_decrement_16) ? -current_smoothed_16 : (error_sign * max_decrement_16);
        } else {
            int16_t abs_error = abs(error);
            adjustment = error_sign * ((abs_error > max_increment_16) ? max_increment_16 : abs_error);
        }
        new_smoothed_16 = current_smoothed_16 + adjustment;
        // Controllo limiti per evitare superamento di current_raw
        if (((adjustment > 0) && (new_smoothed_16 > current_raw_16)) || ((adjustment < 0) && (new_smoothed_16 < current_raw_16))) {
            new_smoothed_16 = current_raw_16;
        }
    } else {
        new_smoothed_16 = current_raw_16;
    }
    return (int8_t)new_smoothed_16;
}

/**
 * @brief Calculates motor drive outputs based on joystick inputs.
 *
 * This function translates Cartesian joystick input values into motor drive power values for a 4-motor vehicle system.
 * It calculates the required power for each wheel (left and right) based on the joystick's X (left-right) and Y (forward-backward)
 * axes inputs. The outputs are constrained to specific forward and backward power limits to ensure
 * controlled vehicle movement. The function also incorporates a turn regulation mechanism to adjust the aggressiveness
 * of turns based on the joystick's position.
 *
 * @param[in] analog A Cartesian2D structure representing the current joystick position, where `x` corresponds to
 *                   the left-right axis and `y` corresponds to the forward-backward axis.
 * @param[in] max_output_forward The maximum power output for forward movement. This value is used to scale
 *                               the joystick's forward movement into an appropriate motor power level.
 * @param[in] max_output_backward The maximum power output for backward movement. This value is used to scale
 *                                the joystick's backward movement into an appropriate motor power level.
 * @param[out] l_wheel Pointer to an integer where the calculated power level for the left wheels will be stored.
 * @param[out] r_wheel Pointer to an integer where the calculated power level for the right wheels will be stored.
 *
 * @return SmoothedAnalog_StatusTypeDef The status of the operation; SA_OK if the operation was successful,
 *                                      or SA_ERROR if an error occurred (e.g., if any of the provided pointers are NULL).
 *
 * @note This function ensures that the output power levels are clamped to the range [-max_output_backward, max_output_forward],
 *       thereby preventing any potential overflow or underflow issues. The turn regulation mechanism (max_turn_reg) allows for
 *       adjustment of the turning aggressiveness within the range [0, 2], where values closer to 2 result in more aggressive turns.
 *       This function is designed for internal use within the module and should be called by higher-level functions that
 *       manage the overall vehicle control logic.
 */
static inline SmoothedAnalog_StatusTypeDef __4_motors_drive(Cartesian2D analog,float max_output_forward, float max_output_backward, int8_t *l_wheel, int8_t* r_wheel){
	static const float tolerance = 0.1f;
	static const float max_input_forward = 127.0f;
    //static const float max_output_forward = 127.0f;
    static const float max_input_backward = 128.0f;
    //static const float max_output_backward = 127.0f;
    static const float max_turn_reg = 0.5f; // Questo valore può essere qualsiasi float nell'intervallo [0,2], estremi inclusi. Più si tende verso il 2 e più la sterzata in FORWARD_LEFT/RIGHT sarà aggressiva. Il suo valore non influenza in alcun modo la RIGHT e la LEFT.
    SmoothedAnalog_StatusTypeDef status = SA_ERROR;
    if((l_wheel != NULL) && (r_wheel != NULL)){
        const float x = (float)(analog.x);
        const float y = (float)(analog.y);
        float temp_l = 0.0f;
        float temp_r = 0.0f;

        if((y > 0.0f) && (x >= 0.0f)) {
        	temp_l = y / max_input_forward * max_output_forward;
        	temp_r = (y / max_input_forward * max_output_forward) - (x / max_input_forward * max_output_forward * max_turn_reg);
        } else if ((y > 0.0f) && (x < 0.0f)){
        	temp_l = (y / max_input_forward * max_output_forward) + (x / max_input_backward * max_output_forward * max_turn_reg);
        	temp_r = y / max_input_forward * max_output_forward;
        } else if ((y < 0.0f) && (x >= 0.0f)) {
        	temp_l = y / max_input_backward * max_output_backward;
        	temp_r = (y / max_input_backward * max_output_backward) + (x / max_input_forward * max_output_backward * max_turn_reg);
        } else if((y < 0.0f) && (x < 0.0f)) {
        	temp_l = (y / max_input_backward * max_output_backward) - (x / max_input_backward * max_output_backward * max_turn_reg);
        	temp_r = y / max_input_backward * max_output_backward;
        } else if (((fabsf(y) - tolerance) < 0.0f) && ((fabsf(x) - tolerance) > 0.0f)){
        	temp_l = x / max_input_forward * max_output_forward;
        	temp_r = -x / max_input_forward * max_output_forward;
        } else {
        	temp_l = 0.0f;
        	temp_r = 0.0f;
        }

        temp_l = __clamp(roundf(temp_l), -max_output_backward, max_output_backward);
        temp_r = __clamp(roundf(temp_r), -max_output_backward, max_output_backward);

        *l_wheel = (int8_t) temp_l;
        *r_wheel = (int8_t) temp_r;

        status = SA_OK;
    }

    return status;
}

SmoothedAnalog_StatusTypeDef smoothed_4_motors_drive(	Cartesian2D analog, int8_t current_l_wheel, int8_t current_r_wheel,
														float max_output_forward, float max_output_backward,
														uint8_t max_increment, uint8_t max_decrement,
														int8_t *l_wheel, int8_t* r_wheel){
	SmoothedAnalog_StatusTypeDef status = SA_ERROR;
	if((l_wheel != NULL)&&(r_wheel!=NULL)){
		int8_t l_wheel_tmp = 0;
		int8_t r_wheel_tmp = 0;
		if(__4_motors_drive(analog,max_output_forward, max_output_backward,&l_wheel_tmp, &r_wheel_tmp)==SA_OK){
			(*l_wheel) = __calculate_new_smoothed_value(current_l_wheel, l_wheel_tmp, max_increment, max_decrement);
			(*r_wheel) = __calculate_new_smoothed_value(current_r_wheel, r_wheel_tmp, max_increment, max_decrement);
			status = SA_OK;
		}
	}
	return status;
}


#endif /* SRC_SMOOTHED_PS2X_ANALOG_C_ */
