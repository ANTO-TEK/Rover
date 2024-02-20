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
 * @file hcsr04.h
 * @author Adinolfi Teodoro, Amato Emilio, Bove Antonio, Guarini Alessio
 * @brief Driver for the HCSR04 ultrasonic sensor.
 * @version 1.0
 * @date 2024-02-15
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef INC_HCSR04_H_
#define INC_HCSR04_H_


#include "stm32f4xx_hal.h"
/**
 * @struct HCSR04_config_t
 * @brief  This structure is used to configure the HCSR04 module
 * 
 * This structure is used to configure the HCSR04 module, setting all the parameters needed to manage the module.
 * 
 * @param Trig_Port: Trig GPIO port selected for the module
 * @param Trig_Pin: Trig GPIO pin selected for the module
 * @param timer: Timer selected for the module
 * @param channel: Timer channel selected for the module
 */
typedef struct {

	GPIO_TypeDef* Trig_Port;
	uint16_t Trig_Pin;
	TIM_HandleTypeDef* timer;
	uint32_t channel;

} HCSR04_config_t;


/**
 * @struct HCSR04_t
 * @brief  This structure is used to manage the HCSR04 module
 * 
 * This structure is used to manage the HCSR04 module, setting all the parameters needed to manage the module.
 * 
 * @param config: Pointer to the configuration structure
 * @param active_channel: Active channel of the timer
 * @param capture1: First capture value
 * @param capture2: Second capture value
 * @param difference: Difference between capture1 and capture2
 * @param is_first_captured: Flag to check if the first capture is done
 * @param timer_callback: Flag to check if the timer callback is done
 * @param distance: Distance value used to update at every reading the computed distance
 * @param current_distance: Current distance value updated at every reading if the distance is valid
 * 
 * 
 */
typedef struct {

	HCSR04_config_t *config;
	uint8_t  active_channel;
	uint32_t capture1;
	uint32_t capture2;
	uint32_t difference;
	uint8_t is_first_captured;
	uint8_t timer_callback;
	float distance;
	float current_distance;

} HCSR04_t;


/**
 * @struct ultrasounds_t
 * @brief  This structure is used to manage the three HCSR04 modules
 * 
 * This structure is used to manage the three HCSR04 modules, setting all the parameters needed to manage the modules.
 * 
 * @param left: HCSR04 module for the left sensor
 * @param center: HCSR04 module for the center sensor
 * @param right: HCSR04 module for the right sensor
 * 
 * 
 */
typedef struct {
	HCSR04_t left, center, right;
} ultrasounds_t;


/**
 * @def WAIT_TRIG_PIN
 * @brief Time to wait after trig pin is set in ordert to start the reading, in microseconds
 */
#define WAIT_TRIG_PIN		(12)

/**
 * @def CAPTURED_TRUE
 * @brief Value of is_first_captured when the capture1 is done
 */
#define CAPTURED_TRUE		(1)

/**
 * @def CAPTURED_FALSE
 * @brief Value of is_first_captured when the capture2 is done
 */
#define CAPTURED_FALSE		(2)

/**
 * @def HCSR04_OK
 * @brief Value of status when the operation is done
 */
#define HCSR04_OK		   (0)

/**
 * @def HCSR04_ERR
 * @brief Value of status when the operation is not done
 */
#define HCSR04_ERR		    (-1)

/**
 * @def HCSR04_TIMEOUT
 * @brief Value of timeout for the reading, in milliseconds
 *
 */
#define HCSR04_TIMEOUT      (22)

/**
 * @def HCSR04_MAX_DISTANCE
 * @brief Value of the maximum distance that the sensor can read, in centimeters, based on the maximum time conceded to the sensor to read the distance
 *
 */
#define HCSR04_MAX_DISTANCE (330.00)


/**
 * @brief This function initializes the HCSR04 module
 * 
 * @param ultrasounds: Pointer to the ultrasounds structure that contains the three HCSR04 modules
 * @param HCSR04_left_config: Pointer to the configuration structure for the left sensor
 * @param HCSR04_center_config: Pointer to the configuration structure for the center sensor 
 * @param HCSR04_right_config: Pointer to the configuration structure for the right sensor 
 * @return int8_t 
 */
int8_t HCSR04_init(ultrasounds_t *ultrasounds, const HCSR04_config_t* HCSR04_left_config, const HCSR04_config_t* HCSR04_center_config, const HCSR04_config_t* HCSR04_right_config);

/**
 * @brief This function sends the trig signal to the HCSR04 module and starts the reading, updating the distance value of the module
 * 
 * @param hcsr04: Pointer to the HCSR04 module
 */
int8_t HCSR04_read(ultrasounds_t *ultrasounds);


#endif /* INC_THREE_HCSR04_H_ */
