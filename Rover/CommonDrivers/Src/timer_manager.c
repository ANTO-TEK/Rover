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

#include "timer_manager.h"

uint32_t __get_timer_clock_frequency(TIM_HandleTypeDef *timer){
	
	uint32_t clock_hz_tmp = 0;

	if(timer != NULL){
		RCC_ClkInitTypeDef clkConfig;
		uint32_t latency=0;
		uint32_t peripheral_reg = (uint32_t) (timer->Instance);

		HAL_RCC_GetClockConfig(&clkConfig, &latency);
		//= The address of the timer register determines which bus it is located on
		if (peripheral_reg >= APB2PERIPH_BASE){
			clock_hz_tmp = HAL_RCC_GetPCLK2Freq();
			if (clkConfig.APB2CLKDivider != RCC_HCLK_DIV1){
				clock_hz_tmp *= 2;
			}
		}
		else if (peripheral_reg >= APB1PERIPH_BASE){
			clock_hz_tmp = HAL_RCC_GetPCLK1Freq();
			if (clkConfig.APB1CLKDivider != RCC_HCLK_DIV1){
				clock_hz_tmp *= 2;
			}
		}
	}
	
	return clock_hz_tmp;
}

/**
 * Calculates the prescaler and period for a given APB timer and update interval.
 *
 * This function determines the appropriate prescaler and period values for
 * an APB timer to achieve a specified sampling time. It calculates these
 * parameters based on the input APB timer clock frequency and the desired
 * sampling time. The function also computes the actual sampling time achievable
 * with the calculated prescaler and period values, providing a feedback mechanism
 * for system calibration.
 *
 * @note The function ensures that the calculated values are within valid ranges and
 *       handles boundary conditions to prevent erroneous settings.
 *
 * @param apb_timer_clock The clock frequency of the APB timer.
 * @param sampling_time The desired update period.
 * @param period Pointer to the variable where the calculated period will be stored.
 * @param prescaler Pointer to the variable where the calculated prescaler will be stored.
 * @param real_sampling_time Pointer to the variable where the actual sampling time will be stored.
 * @return EncoderManager_StatusTypeDef Status of the operation (ENCODER_MANAGER_OK or ENCODER_MANAGER_ERROR).
 */
void __get_sampling_timer_params(TIM_HandleTypeDef *timer, const float timeout, uint32_t *period, uint32_t *prescaler) {
    
    const float x = __get_timer_clock_frequency(timer);
    const float k = timeout;
    const float max_k = 2147450880.0f / x;
    float z = 0;

    if ((period != NULL) && (prescaler != NULL) && x > 0 && k > SAMPLING_TIME_MIN && k < max_k) {
        z = (k * x) / (MAX_VALUE);
        if ((z >= 0.0f) && (z < MAX_VALUE)) {
            // Ensure that z is a valid integer value
            uint32_t z_int = (uint32_t) z;
            if (z_int < MAX_VALUE) {
                *prescaler = (uint32_t) MAX_VALUE;
                *period = (uint32_t)(z + 0.5f);
            }
        }
    }

}

void start_timer(TIM_HandleTypeDef *timer, float timeout){
	
	uint32_t period, prescaler;
	
	__get_sampling_timer_params(timer, timeout, &period, &prescaler);
	
	timer->Init.Prescaler = prescaler;
	timer->Init.CounterMode = TIM_COUNTERMODE_UP;
	timer->Init.Period = period;
	timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(timer) != HAL_OK){
		Error_Handler();
	}

	HAL_TIM_Base_Start_IT(timer);
}

void stop_timer(TIM_HandleTypeDef *timer){
    HAL_TIM_Base_Stop_IT(timer);
}
