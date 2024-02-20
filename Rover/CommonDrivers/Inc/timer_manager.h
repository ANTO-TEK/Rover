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
 * @file timer_manager.h
 * @author Adinolfi Teodoro, Amato Emilio, Bove Antonio, Guarini Alessio
 * @brief  This file contains function for managing the timer peripherals.
 * @version 1.0
 * @date 2024-02-15
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef TIMER_MANAGER_H
#define TIMER_MANAGER_H

#include <stdint.h>

#include "tim.h"


#define SAMPLING_TIME_MIN           (0.0f)
#define MAX_VALUE	  				(65535.0f)

/**
 * @brief Start a timer with a given timeout
 * 
 * @param timer pointer to the timer peripheral
 * @param timeout timeout in seconds
 */
void start_timer(TIM_HandleTypeDef *timer, float timeout);

/**
 * @brief Stop a timer
 * 
 * @param timer pointer to the timer peripheral
 */
void stop_timer(TIM_HandleTypeDef *timer);

#endif
