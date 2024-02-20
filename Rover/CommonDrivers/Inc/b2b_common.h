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
 * @file b2b_common.h
 * @author Adinolfi Teodoro, Amato Emilio, Bove Antonio, Guarini Alessio
 * @brief This file contains the common definitions for the B2B (board to board) protocol.
 * @version 0.1
 * @date 2024-02-04
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef B2B_COMMON_H
#define B2B_COMMON_H

#include <stdint.h>
#include <stdlib.h>
#include "usart.h"
#include "gpio.h"
#include "state_manager.h"

/**
 * @brief Specifies the number of times the board will try to resend a message before giving up.
 */
#define B2B_RESEND_NUM					(2U)

// ----------------------- TEMPERATURE SECTION ----------------------------------

/**
 * @brief Specifies the minimum operative temperature for the board.
 */
#define MIN_NORMAL_TEMP					(0.0F)
/**
 * @brief Specifies the maximum operative temperature for the board. If the temperature exceeds this value
 * 		  the temperature will be considered "above normal". Staing in this condition for a long time will
 * 		  cause the board state to be set to "critical".
 */
#define MAX_NORMAL_TEMP					(35.0F)
/**
 * @brief Specifies the critical temperature for the board. The system state will be set to "critical" and
 *        the system will be immediately stopped.
 */
#define CRITICAL_TEMP                  	(50.0F)

/**
 * @brief Specifies the maximum number of ticks in witch the temperature can be above normal before the state
 * 		  is set to "critical".
 */
#define ABOVE_NORMAL_MAX_TIME       (100)

// ----------------------- DISTANCE SECTION ----------------------------------

/**
 * @brief Specifies the distance from the board to the nearest obstacle that will be considered "nearby".
 *        The rover will start to slow down when the distance is less than this value.
 */
#define DISTANT_OBSTACLE_DISTANCE      	(200.0F)
/**
 * @brief Specifies the distance from the board to the nearest obstacle that will be considered "nearby".
 *        The rover will immediately try to avoid the obstacle automatically when the distance is less than this value.
 */
#define NEARBY_OBSTACLE_DISTANCE       	(70.0F)

/**
 * @brief Specifies the distance from the board to the nearest lateral obstacle, if the distance is less than this value
 * 	  	  the rover won't be able to turn on itself in that direction. 
 */
#define MIN_SIDE_DISTANCE				(10.0F)    // to change

// -------------------------------------------------------------------------

#define ALT_BACKWARD_ROT_ANGLE			(180U)
#define OBSTACLE_ROT_ANGLE				(90U)

//-------------------  MASK SECTION ------------------------------

// Vedi relazione tabella 4.3
/**
 * @brief Specifies the mask that must be applyed to the <code>control_value</code> variable to check the joypad status.
 */
#define JOYPAD_MASK						(0x01)
/**
 * @brief Specifies the mask that must be applyed to the <code>control_value</code> variable to check the MPU status.
 */
#define MPU_MASK						(0x02)

//--------------------------------------------------------------------------

/**
 * @brief Define the b2b_status type. Will be used to return the status of the protocol functions.
 */
typedef uint8_t b2b_status;

#define B2B_ERR				(1U)
#define B2B_OK				(0U)

/**
 * @brief The check time enum is used to enable or disable the time check for the protocol.
 * 		  Is used for the above normal temperature check.
 */
typedef enum {
	CHECK_TIME_ENABELED = 0,
	CHECK_TIME_DISABELED
}CHECK_TIME;

/**
 * @brief The b2b_protocol_t struct is used to store the configuration of the protocol. Contains all the
 * 		  necessary information to communicate with the other board such as the UART handle and the CRC handle
 * 		  and the GPIO pins used to send and receive the RTR and ACK signals.
 */
typedef struct {
	UART_HandleTypeDef* uartHandle;
	CRC_HandleTypeDef* crcHandle;
	GPIO_TypeDef *ack_in_Port,
				 *rtr_in_Port,
				 *ack_out_Port,
				 *rtr_out_Port;
	uint16_t ack_in_Pin,
			 rtr_in_Pin,
			 ack_out_Pin,
			 rtr_out_Pin;
	TIM_HandleTypeDef *timer;
} b2b_config_t;

/**
 * @brief The b2b_protocol_t struct is used to store the status of the protocol. Contains the
 * 		  configuration of the protocol and the status of the RTR and ACK signals. It also contains
 * 		  the last tick value used to check the time for the above normal temperature check and the
 * 		  rover intention computed and received from the other board.
 */
typedef struct {

	b2b_config_t* config;

	uint8_t *buffer;

	CHECK_TIME check_time;					/**< Enable or disable the time check for the above normal temperature check */
	uint32_t last_tick;						/**< The last tick at the normal value used to check the time for the above normal temperature check */

	rover_intention_t computed_intention;	/**< The intention computed by the board and that will be sent to the other*/
	rover_intention_t received_intention;   /**< The intention received from the other board*/
	substate_b2_b1 substate_b2_b1;

	uint8_t protocol_step;					/**< The current step of the protocol, used to check the status of the protocol */
	uint8_t protocol_status;				/**< Used to check the status of the protocol */
	uint8_t retransmissions_num;			/**< The number of retransmissions for the state or the intention */

} b2b_protocol_t;

/**
 * @brief Initialize the b2b_protocol_t structure at the default values adding the required configuration.
 * @param config a pointer to the b2b_config_t structure containing the configuration of the protocol.
 */
b2b_status b2b_protocol_init(const b2b_config_t* config, uint8_t *buffer);

/**
 * @brief Get the b2b_protocol_t instance.
 * @return b2b_protocol_t* a pointer to the b2b_protocol_t instance.
 */
b2b_protocol_t* const get_b2b_instance();

// --------------PUBLIC PROTOCOL COMMON FUNCTIONS --------------

/**
 * @brief Send the RTR signal to the other board.
 */
void b2b_send_RTR();
/**
 * @brief Send the ACK signal to the other board.
 */
void b2b_send_ACK();

/**
 * @brief Send the intention to the other board. The intention is stored in the b2b_protocol_t structure as <code> computed_intention </code>.
 * 		  The intention is transmitted using the UART interface, parity and the CRC is used to check the integrity of the message.
 * @return b2b_status will be B2B_OK if the intention is sent correctly, B2B_ERR otherwise. 
 */
b2b_status b2b_send_intention();

/**
 * @brief Receive the intention from the other board. The intention is stored in the b2b_protocol_t structure as <code> received_intention </code>.
 * @return b2b_status will be B2B_OK if the intention is received correctly, B2B_ERR otherwise. 
 */
b2b_status b2b_request_intention(uint8_t *buffer);

/**
 * @brief Deserialized the received intention from the other board. The intention is stored in the b2b_protocol_t structure as <code> received_intention </code>.
 *        The function will also check the integrity of the message using the CRC.
 * @param buffer a pointer to the buffer containing the received intention.
 * @return b2b_status will be B2B_OK if the intention is deserialized correctly and the integrity is confirmed, B2B_ERR otherwise.
 */
b2b_status deserialize_intention(uint8_t* buffer);

/**
 * @brief Check if the received intention and the computed intention are the same, this means that both the boards agree on the next state. The function
 * 		  will also check if the intention is not for the critical state.
 * @return b2b_status will be B2B_OK if the intentions are the same and are not for a crytical state, B2B_ERR otherwise.
 */
b2b_status b2b_check_intention();


// ------------- STATE CHECK ----------
/**
 * @brief Check if the obstacle is nearby and in which direction.
 * @param obstacle_distances the array containing the distances from the system to the obstacles.
 * @return OBSTACLE_DIR the direction of the nearby obstacle.
 */
OBSTACLE_DIR nearby_obstacle_direction(float* obstacle_distances);

/**
 * @brief Update the next state of the system
 * 
 * @param obstacle_distances array containing the distances from the system to the obstacles
 * @param control_buttons    array containing the status of the control buttons
 * @param gyroZ              the value of the Z axis of the gyroscope
 * @param obstacle           the direction of the nearby obstacle
 * @param y_analog           the value of the y axis of the analog stick
 * @param x_analog           the value of the x axis of the analog stick
 * @param rpm_motors         the array containing the rpm of the motors
 * @param next_state         output parameter containing the next state of the system
 * @param braking            output parameter containing the type of braking
 */
void update_next_state(float *obstacle_distances, uint8_t *control_buttons, float gyroZ, OBSTACLE_DIR obstacle,
		 	 	 	   int8_t y_analog, int8_t x_analog, float *rpm_motors, ROVER_STATE *next_state, BRAKING_TYPE *braking);

/**
 * @brief Check the joypad status and return it's status.
 * @param control_value this value contains the status of the joypad and the MPU.
 * @return PS2X_StatusTypeDef the status of the joypad.
 */
PS2X_StatusTypeDef check_joypad(uint8_t control_value);

/**
 * @brief Check the MPU status and return it's status.
 * 
 * @param control_value this value contains the status of the joypad and the MPU.
 * @return MPU60X0_StatusTypeDef the status of the MPU.
 */
MPU60X0_StatusTypeDef check_mpu(uint8_t control_value);

/**
 * @brief Check the temperature values and return a sintetic status.
 * 
 * @param temperature_B1 temperature value from the board1.
 * @param temperature_B2 temperature value from the board2.
 * @return TEMPERATURE_STATUS the status of the temperature.
 */
TEMPERATURE_STATUS check_temperature_values(float temperature_B1, float temperature_B2);

#endif
