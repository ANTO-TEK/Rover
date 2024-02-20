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

#include "encoder.h"

#define MAX_CHANNELS 										(2U)
#define MIN_CHANNELS 										(1U)

Encoder_StatusTypeDef encoder_init(encoder_t *encoder, TIM_HandleTypeDef *timer){
	Encoder_StatusTypeDef status = ENCODER_ERROR;
	if(encoder != NULL && timer != NULL && HAL_TIM_Encoder_Start(timer, TIM_CHANNEL_ALL) == HAL_OK){
		encoder->timer = timer;
		encoder->n_channels = (encoder->timer->Instance->SMCR & 0x3) == 0x3 ? 2 : 1;
		encoder->counter1 = 0;
		encoder->counter2 = 0;
		encoder->rotation_status = DEFAULT_DIRECTION;
		encoder->last_rpm = 0;
		status = ENCODER_OK;
	}
	return status;
}
