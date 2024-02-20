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

/**
 * @file b2b_protocol.c
 * @author Adinolfi Teodoro, Amato Emilio, Bove Antonio, Guarini Alessio
 * @brief Implementation file for the B2B communication protocol (PrimaryBoard side), providing functions for managing the communication between boards.
 * @version 1.0
 * @date 2024-02-15
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "b2b_protocol.h"
#include "usart.h"
#include "state_manager.h"
#include "string.h"
#include "crc.h"
#include "math.h"

#include "b2b_common.h"

/**
 * @brief Store the received substate from the counterpart board.
 * 
 */
static substate_b2_b1 received_substate;

b2b_status deserialize_status(uint8_t* buffer){

	b2b_protocol_t* b2b= get_b2b_instance();
	b2b_status status = B2B_ERR;
	uint32_t crc;

	memcpy(&crc, &buffer[CRC_B2B1_INDEX], CRC_SIZE);

	if (HAL_CRC_Calculate(b2b->config->crcHandle, (uint32_t*)buffer, (uint32_t) SUBSTATE_B2_B1_BYTE_SIZE/sizeof(uint32_t)) == crc){ 

		memcpy( received_substate.float_buffer,buffer,FLOAT_B2B1_BYTE); 
		memcpy( received_substate.int_buffer,buffer + FLOAT_B2B1_BYTE, UINT_B2B1_BYTE); 

		status = B2B_OK;
	}

	return status;
}

b2b_status inline b2b_request_state(){

	b2b_protocol_t* b2b = get_b2b_instance();
	b2b_status status = B2B_ERR;
	if ( HAL_UART_Receive_IT(b2b->config->uartHandle, b2b->buffer, SUBSTATE_B2_B1_BYTE_SIZE + CRC_SIZE) == HAL_OK){
		b2b_send_RTR();
		status = B2B_OK;
	}
	return status;
}

b2b_status b2b_send_state(){

	b2b_protocol_t* b2b = get_b2b_instance();
	b2b_status status = B2B_ERR;
	substate_b1_b2 temp = capture_board1_substate();
	
	memcpy( b2b->buffer, temp.float_buffer,FLOAT_B1B2_BYTE); // As described insiede the table 4.5 of the documentation
	memcpy( &b2b->buffer[FLOAT_B1B2_BYTE], temp.int_buffer,INT_B1B2_BYTE);
	memcpy( &b2b->buffer[FLOAT_B1B2_BYTE + INT_B1B2_BYTE], temp.uint_buffer  , UINT_B1B2_BYTE); 

	uint32_t crc = HAL_CRC_Calculate(b2b->config->crcHandle,(uint32_t*) b2b->buffer, (uint32_t)SUBSTATE_B1_B2_BYTE_SIZE/sizeof(uint32_t));
	memcpy(&b2b->buffer[CRC_B1B2_INDEX ], &crc, CRC_SIZE); 

	if ( HAL_UART_Transmit( b2b->config->uartHandle , b2b->buffer, SUBSTATE_B1_B2_BYTE_SIZE + CRC_SIZE, HAL_MAX_DELAY) == HAL_OK){
		status = B2B_OK;
	}
	return status;

}

void b2b_processing_state(){

	rover_state_t *global_status = rover_state_get_instance();
	b2b_protocol_t* b2b= get_b2b_instance();
	PS2X_StatusTypeDef ps2x_status   =  check_joypad(global_status->control_value );
	MPU60X0_StatusTypeDef mpu_status = 	check_mpu(global_status->control_value 	);
	TEMPERATURE_STATUS temperature_status;

	if(global_status->single_board_is_active == 0U)
		temperature_status = check_temperature_values(global_status->temperature_B1, received_substate.float_buffer[SUBSTATE_B2B1_TEMPB2_INDEX]);
	else
		temperature_status = check_temperature_values(global_status->temperature_B1, global_status->temperature_B1);

	if (ps2x_status == PS2X_ERROR || mpu_status == MPU60X0_ERROR || temperature_status == CRITICAL ){
		b2b->computed_intention.next_state = EMERGENCY;
	} else{

		if( mpu_status == MPU60X0_ERROR || temperature_status == ABOVE_NORMAL ) {
			b2b->computed_intention.degraded = ACTIVE;
		} else{
			b2b->computed_intention.degraded = INACTIVE;
		}

		global_status->obstacle = nearby_obstacle_direction(global_status->obstacle_distances);

		if(global_status->single_board_is_active == 0U)
			update_next_state(global_status->obstacle_distances, &global_status->control_buttons, global_status->gyroZ,
						  	  global_status->obstacle, global_status->y_analog, global_status->x_analog,
						  	  &received_substate.float_buffer[SUBSTATE_B2B1_RPM_MOTOR_0_INDEX],
							  &(b2b->computed_intention.next_state), &(b2b->computed_intention.braking));
		else
			update_next_state(global_status->obstacle_distances, &global_status->control_buttons, global_status->gyroZ,
							  global_status->obstacle, global_status->y_analog, global_status->x_analog,
							  NULL, &(b2b->computed_intention.next_state), &(b2b->computed_intention.braking));


	}
}


void b2b_update_global_state(){
	rover_state_t *global_status = rover_state_get_instance();
	b2b_protocol_t* b2b = get_b2b_instance();
	// The intention is accepted and the exchanged data becames part of the global state
	global_status->next_state = b2b->computed_intention.next_state;
	global_status->braking = b2b->computed_intention.braking;
	global_status->degraded = b2b->computed_intention.degraded;
	// The data specific to board 2, now that the intention is common, are made global
	if(global_status->single_board_is_active == 0U){
		global_status->temperature_B2 = received_substate.float_buffer[SUBSTATE_B2B1_TEMPB2_INDEX];
		memcpy(global_status->rpm_motors,&received_substate.float_buffer[SUBSTATE_B2B1_RPM_MOTOR_0_INDEX], 4*sizeof(float));
		memcpy(global_status->motors_direction, received_substate.int_buffer,4*sizeof(uint8_t));
	}
	global_status->current_state = global_status->next_state;
}




