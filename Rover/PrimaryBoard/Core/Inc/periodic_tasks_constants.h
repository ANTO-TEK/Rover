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
 * This file is part of PrimaryBoard.
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

#ifndef PERIODIC_TASLS_CONSTANTS
#define PERIODIC_TASLS_CONSTANTS

#define READ_SENSORS_TASK_PHASE 		pdMS_TO_TICKS(0)
#define READ_SENSORS_TASK_PERIOD		pdMS_TO_TICKS(60)
#define READ_SENSORS_TASK_WCET 		    pdMS_TO_TICKS(26)
#define READ_SENSORS_TASK_DEADLINE 		pdMS_TO_TICKS(60)

#define DECISION_TASK_PHASE  			pdMS_TO_TICKS(0)
#define DECISION_TASK_PERIOD 			pdMS_TO_TICKS(60)
#define DECISION_TASK_WCET 				pdMS_TO_TICKS(24)
#define DECISION_TASK_DEADLINE 			pdMS_TO_TICKS(60)

#define ACTUATION_TASK_PHASE 			pdMS_TO_TICKS(0)
#define ACTUATION_TASK_PERIOD 			pdMS_TO_TICKS(60)
#define ACTUATION_TASK_WCET 			pdMS_TO_TICKS(17)
#define ACTUATION_TASK_DEADLINE			pdMS_TO_TICKS(60)

#define SINGLE_BOARD_TASK_PHASE 		pdMS_TO_TICKS(0)
#define SINGLE_BOARD_TASK_PERIOD		pdMS_TO_TICKS(60)
#define SINGLE_BOARD_TASK_WCET   		pdMS_TO_TICKS(2)
#define SINGLE_BOARD_TASK_DEADLINE  	pdMS_TO_TICKS(60)

#endif
