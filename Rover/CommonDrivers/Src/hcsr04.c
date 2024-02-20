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

#include "hcsr04.h"
#include "delayus.h"

static ultrasounds_t *tmp_ultrasounds;


static int8_t __select_active_channel(HCSR04_t* hcsr04){

	int8_t status = HCSR04_OK;

	switch (hcsr04->config->channel) {
	    case TIM_CHANNEL_1:
	        hcsr04->active_channel = HAL_TIM_ACTIVE_CHANNEL_1;
	        break;
	    case TIM_CHANNEL_2:
	        hcsr04->active_channel = HAL_TIM_ACTIVE_CHANNEL_2;
	        break;
	    case TIM_CHANNEL_3:
	        hcsr04->active_channel = HAL_TIM_ACTIVE_CHANNEL_3;
	        break;
	    case TIM_CHANNEL_4:
	        hcsr04->active_channel = HAL_TIM_ACTIVE_CHANNEL_4;
	        break;
	    default:
	    	status = HCSR04_ERR;
	        break;
	}

	return status;

}

static int8_t __read_and_capture(HCSR04_t *hcsr04){

	int8_t status = HCSR04_OK;

	if(hcsr04->is_first_captured == CAPTURED_FALSE && hcsr04->timer_callback == 1) {

		hcsr04->capture1 = HAL_TIM_ReadCapturedValue(hcsr04->config->timer, hcsr04->config->channel);
		hcsr04->is_first_captured = CAPTURED_TRUE;
		__HAL_TIM_SET_CAPTUREPOLARITY(hcsr04->config->timer, hcsr04->config->channel, TIM_INPUTCHANNELPOLARITY_FALLING);

	}else if(hcsr04->is_first_captured == CAPTURED_TRUE && hcsr04->timer_callback == 2) {

		hcsr04->capture2 = HAL_TIM_ReadCapturedValue(hcsr04->config->timer, hcsr04->config->channel);
		__HAL_TIM_SET_COUNTER(hcsr04->config->timer, 0);


		if(hcsr04->capture2 > hcsr04->capture1) {
			hcsr04->difference = hcsr04->capture2 - hcsr04->capture1;
		}else if(hcsr04->capture1 > hcsr04->capture2) {
			hcsr04->difference = (hcsr04->config->timer->Init.Period - hcsr04->capture1) + hcsr04->capture2;
		}

		if((hcsr04->difference * 0.034 / 2) <= HCSR04_MAX_DISTANCE){
			hcsr04->distance = hcsr04->difference * 0.034 / 2;
			hcsr04->current_distance = hcsr04->distance;
		}
		else{
			hcsr04->current_distance = HCSR04_MAX_DISTANCE;
		}

		hcsr04->is_first_captured = CAPTURED_FALSE;
		__HAL_TIM_SET_CAPTUREPOLARITY(hcsr04->config->timer, hcsr04->config->channel, TIM_INPUTCHANNELPOLARITY_RISING);
		hcsr04->timer_callback = 0;

		if(hcsr04->distance <= 0)
			status = HCSR04_ERR;
	}

	return status;

}

void __send_trig(HCSR04_t *hcsr04){

	HAL_GPIO_WritePin(hcsr04->config->Trig_Port, hcsr04->config->Trig_Pin, GPIO_PIN_SET);
	delay_us(WAIT_TRIG_PIN);
	HAL_GPIO_WritePin(hcsr04->config->Trig_Port, hcsr04->config->Trig_Pin, GPIO_PIN_RESET);
	hcsr04->distance = -1;

}

int8_t HCSR04_read(ultrasounds_t *ultrasounds){

	int8_t status = HCSR04_ERR;

	uint32_t stop_time;

	if(ultrasounds){

		tmp_ultrasounds = ultrasounds;

		__send_trig(&ultrasounds->left);
		__send_trig(&ultrasounds->center);
		__send_trig(&ultrasounds->right);

		stop_time = HAL_GetTick() + HCSR04_TIMEOUT;

		while(HAL_GetTick() <= stop_time && (ultrasounds->left.distance <= 0 ||
											 ultrasounds->center.distance <= 0 ||
											 ultrasounds->right.distance <= 0)){

			if(__read_and_capture(&ultrasounds->left) == HCSR04_OK &&
			   __read_and_capture(&ultrasounds->center) == HCSR04_OK &&
			   __read_and_capture(&ultrasounds->right) == HCSR04_OK){

				status = HCSR04_OK;

			}

		}

	}

	return status;
}

int8_t HCSR04_init(ultrasounds_t *ultrasounds, const HCSR04_config_t* HCSR04_left_config,
											   const HCSR04_config_t* HCSR04_center_config,
											   const HCSR04_config_t* HCSR04_right_config){

	int status = HCSR04_OK;

	if(ultrasounds){

		ultrasounds->left.config = HCSR04_left_config;
		ultrasounds->left.is_first_captured = CAPTURED_FALSE;
		ultrasounds->left.capture1 = 0;
		ultrasounds->left.capture2 = 0;
		ultrasounds->left.difference = 0;
		ultrasounds->left.distance = -1;
		ultrasounds->left.timer_callback = 0;
		HAL_TIM_IC_Start_IT(ultrasounds->left.config->timer, ultrasounds->left.config->channel);

		ultrasounds->center.config = HCSR04_center_config;
		ultrasounds->center.is_first_captured = CAPTURED_FALSE;
		ultrasounds->center.capture1 = 0;
		ultrasounds->center.capture2 = 0;
		ultrasounds->center.difference = 0;
		ultrasounds->center.distance = -1;
		ultrasounds->center.timer_callback = 0;
		HAL_TIM_IC_Start_IT(ultrasounds->center.config->timer, ultrasounds->center.config->channel);

		ultrasounds->right.config = HCSR04_right_config;
		ultrasounds->right.is_first_captured = CAPTURED_FALSE;
		ultrasounds->right.capture1 = 0;
		ultrasounds->right.capture2 = 0;
		ultrasounds->right.difference = 0;
		ultrasounds->right.distance = -1;
		ultrasounds->right.timer_callback = 0;
		HAL_TIM_IC_Start_IT(ultrasounds->right.config->timer, ultrasounds->right.config->channel);

		if(__select_active_channel(&ultrasounds->left) != HCSR04_OK ||
		   __select_active_channel(&ultrasounds->center) != HCSR04_OK ||
		   __select_active_channel(&ultrasounds->right) != HCSR04_OK)

			status = HCSR04_ERR;

	} else {

		status = HCSR04_ERR;
	}

	return status;

}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {

	if(tmp_ultrasounds){
		if(htim->Instance == tmp_ultrasounds->left.config->timer->Instance && htim->Channel == tmp_ultrasounds->left.active_channel) {
			tmp_ultrasounds->left.timer_callback += 1;
		} else if(htim->Instance == tmp_ultrasounds->center.config->timer->Instance && htim->Channel == tmp_ultrasounds->center.active_channel){
			tmp_ultrasounds->center.timer_callback += 1;
		} else if(htim->Instance == tmp_ultrasounds->right.config->timer->Instance && htim->Channel == tmp_ultrasounds->right.active_channel){
			tmp_ultrasounds->right.timer_callback += 1;
		}
	}

}
