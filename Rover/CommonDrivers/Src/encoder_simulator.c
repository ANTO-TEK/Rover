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

#include "encoder_simulator.h"

static inline EncoderSimulator_StatusTypeDef __get_timer_clock_frequency(TIM_HandleTypeDef *timer, uint32_t *clock_hz){
	EncoderSimulator_StatusTypeDef status = ENCODER_SIM_ERROR;
	if(timer != NULL && clock_hz != NULL){
		RCC_ClkInitTypeDef clkConfig;
		uint32_t latency=0;
		uint32_t clock_hz_tmp=0;
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
		*clock_hz = clock_hz_tmp;
		status = ENCODER_SIM_OK;
	}
	return status;
}

EncoderSimulator_StatusTypeDef encoder_simulator_init(encoder_simulator_t *encoder_simulator, TIM_HandleTypeDef *pwm_timer,
														uint32_t tim_channel, uint8_t ppr, uint8_t gear_ratio){

	EncoderSimulator_StatusTypeDef status = ENCODER_SIM_ERROR;
	if (encoder_simulator != NULL && pwm_timer != NULL){
		encoder_simulator->gear_ratio = gear_ratio;
		encoder_simulator->ppr = ppr;
		encoder_simulator->tim_channel = tim_channel;
		encoder_simulator->pwm_timer = pwm_timer;
		status = ENCODER_SIM_OK;
	}
	return status;

}


EncoderSimulator_StatusTypeDef __pwm_init(encoder_simulator_t *encoder_simulator) {
	EncoderSimulator_StatusTypeDef status = ENCODER_SIM_ERROR;
	if (encoder_simulator != NULL && encoder_simulator->pwm_timer != NULL) {
		TIM_MasterConfigTypeDef sMasterConfig = {0};
		TIM_OC_InitTypeDef sConfigOC = {0};
		TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

		// Configurazione del timer PWM
		encoder_simulator->pwm_timer->Init.Prescaler = 0;
		encoder_simulator->pwm_timer->Init.CounterMode = TIM_COUNTERMODE_UP;
		encoder_simulator->pwm_timer->Init.Period = 1023;
		encoder_simulator->pwm_timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		encoder_simulator->pwm_timer->Init.RepetitionCounter = 0;
		encoder_simulator->pwm_timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

		if (HAL_TIM_PWM_Init(encoder_simulator->pwm_timer) != HAL_OK) {
			status = ENCODER_SIM_ERROR;
		} else {
			status = ENCODER_SIM_OK;
		}

		// Configurazione Master
		if (status == ENCODER_SIM_OK) {
			sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
			sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
			if (HAL_TIMEx_MasterConfigSynchronization(encoder_simulator->pwm_timer, &sMasterConfig) != HAL_OK) {
				status = ENCODER_SIM_ERROR;
			}
		}

		// Configurazione del canale PWM
		if (status == ENCODER_SIM_OK) {
			sConfigOC.OCMode = TIM_OCMODE_PWM1;
			sConfigOC.Pulse = 512;
			sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
			sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
			sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
			sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
			sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
			if (HAL_TIM_PWM_ConfigChannel(encoder_simulator->pwm_timer, &sConfigOC, encoder_simulator->tim_channel) != HAL_OK) {
				status = ENCODER_SIM_ERROR;
			}
		}

		// Configurazione BreakDeadTime
		if (status == ENCODER_SIM_OK) {
			sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
			sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
			sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
			sBreakDeadTimeConfig.DeadTime = 0;
			sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
			sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
			sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
			if (HAL_TIMEx_ConfigBreakDeadTime(encoder_simulator->pwm_timer, &sBreakDeadTimeConfig) != HAL_OK) {
				status = ENCODER_SIM_ERROR;
			}
		}

		// Post-Initialization e avvio del PWM
		if (status == ENCODER_SIM_OK) {
			HAL_TIM_MspPostInit(encoder_simulator->pwm_timer);
			if (HAL_TIM_PWM_Start(encoder_simulator->pwm_timer, encoder_simulator->tim_channel) != HAL_OK) {
				status = ENCODER_SIM_ERROR;
			}

		}
	}
    return status; // Singola istruzione di return
}




EncoderSimulator_StatusTypeDef encoder_simulator_set_speed(encoder_simulator_t *encoder_simulator, float speed){
	EncoderSimulator_StatusTypeDef status = ENCODER_SIM_ERROR;
	if(encoder_simulator != NULL && speed > 0){
		float update_freq_f = (speed/60)*(encoder_simulator->ppr)*(encoder_simulator->gear_ratio);
		uint32_t apb_freq = 0;
		uint32_t prescaler = 0;
		if(__get_timer_clock_frequency(encoder_simulator->pwm_timer, &apb_freq) == ENCODER_SIM_OK){
			prescaler = (uint32_t)(((float) apb_freq) / (1024.0f * update_freq_f) - 1);
			encoder_simulator->pwm_timer->Init.Prescaler = prescaler;
			if (HAL_TIM_PWM_Init(encoder_simulator->pwm_timer) == HAL_OK &&
				HAL_TIM_PWM_Start(encoder_simulator->pwm_timer, encoder_simulator->tim_channel) == HAL_OK) {
				uint32_t pulse = __HAL_TIM_GET_AUTORELOAD(encoder_simulator->pwm_timer) / 2;
				__HAL_TIM_SET_COMPARE(encoder_simulator->pwm_timer, encoder_simulator->tim_channel, pulse);
				status = ENCODER_SIM_OK;

			}

		}
	}


	return status;
}
