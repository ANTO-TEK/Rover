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

#include "encoder_manager.h"
#include "main.h"

#define PERIOD_MAX  											((float) 32768.0f)
#define MAX_VALUE	  											((float) 65535.0f)

#define PERIOD_MIN  											((float) 0.0f)
#define PRESCALER_MIN	  										((float) 0.0f)
#define SAMPLING_TIME_MIN  										((float) 0.0f)



/* Private data types */
/**
 * @brief Structure to hold encoder manager data.
 */
typedef struct {
	encoder_manager_config_t *config;
	uint32_t last_read_tick; ///< Sampling time in seconds.
} encoder_manager_t;

/* Private instance declaration */
/**
 * @brief Instance of the encoder manager.
 *
 * This declaration creates a single, modifiable instance of encoder_manager_t,
 * but the pointer to this instance (instance) is constant.
 *
 * A constant pointer (denoted by * const) means that the pointer itself cannot
 * be redirected to point to another object or variable. However, the content of
 * the object it points to can be modified. This is a common practice in C to
 * create a singleton-like instance that is globally accessible but cannot be
 * re-assigned to point to a different object or memory location.
 */
static encoder_manager_t em_instance = {
	.last_read_tick = 0
};


EncoderManager_StatusTypeDef encoder_manager_init(encoder_manager_config_t *config){

	EncoderManager_StatusTypeDef status = ENCODER_MANAGER_ERROR;

	if(config != NULL){
		Encoder_StatusTypeDef encoder_status = ENCODER_OK;
		em_instance.config = config;
		for(uint8_t i = 0; i < ENCODER_NUM && (encoder_status == ENCODER_OK); i++){
			encoder_status = encoder_init(&(em_instance.config->encoders[i]), config->encoders_timer[i]);
		}
		if(encoder_status == ENCODER_OK){
			status = ENCODER_MANAGER_OK;
		}
	}

	return status;
}



static inline void __read_encoder_rpm(float *result, uint8_t encoder_number){
		uint16_t diff = 0;
		float tick_to_sec = 0.0f;
		uint32_t counter1 = ((em_instance.config)->encoders[encoder_number]).counter1;
		uint32_t counter2 = __HAL_TIM_GET_COUNTER(em_instance.config->encoders[encoder_number].timer);
		TIM_HandleTypeDef *timer = em_instance.config->encoders[encoder_number].timer;


		if(counter2 == counter1){
			diff = 0; // In this case the speed is 0
			em_instance.config->encoders[encoder_number].rotation_status = DEFAULT_DIRECTION;
		}
		else if (__HAL_TIM_IS_TIM_COUNTING_DOWN(timer)){
			if (counter2 < counter1){				// Counter underflow
				diff = counter1 - counter2;
			}
			else{
				diff = (timer->Init.Period - counter2) + counter1;
			}
			em_instance.config->encoders[encoder_number].rotation_status = BACKWARD_DIRECTION;
		}else{
			if (counter2 > counter1){ 					// Counter overflow
			  diff = counter2 - counter1;
			}
			else{
			  diff = (timer->Init.Period - counter1) + counter2;
			}
			em_instance.config->encoders[encoder_number].rotation_status = FORWARD_DIRECTION;
		}

		em_instance.config->encoders[encoder_number].counter2 = counter2;
		tick_to_sec = ((float)(HAL_GetTick()-em_instance.last_read_tick))/1000.0f;
		em_instance.config->encoders[encoder_number].last_rpm = result[encoder_number] =(diff*60)/(tick_to_sec*CPR*GEAR_RATIO*em_instance.config->encoders[encoder_number].n_channels);
		em_instance.config->encoders[encoder_number].counter1 = __HAL_TIM_GET_COUNTER(timer);        // Update the counter1 in order to prepare it for the next reading

}

void encoder_manager_update_rpm(float *result){

	for(uint8_t i = 0; i < ENCODER_NUM; i++){
			__read_encoder_rpm(result,i);
	}

	em_instance.last_read_tick = HAL_GetTick();

}
