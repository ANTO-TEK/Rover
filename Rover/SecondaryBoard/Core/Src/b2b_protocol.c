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
 * This file is part of SecondaryBoard.
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

#include "b2b_protocol.h"
#include "usart.h"
#include "state_manager.h"
#include "string.h"
#include "crc.h"
#include "math.h"


static substate_b1_b2 received_substate;

// ********************************* UTILITY ************************************

// valutare questa funzione.
b2b_status deserialize_status(uint8_t* buffer){

	b2b_protocol_t* b2b = get_b2b_instance();
	b2b_status status = B2B_ERR;
	uint32_t crc;

	memcpy(&crc, &buffer[CRC_B1B2_INDEX], CRC_SIZE); // Seleziono il CRC che dovrebbe essere a fine buffer

	if (HAL_CRC_Calculate(b2b->config->crcHandle, (uint32_t*)buffer, SUBSTATE_B1_B2_BYTE_SIZE/4) == crc){ // Se il CRC è corretto, NOTA: è diviso 4 perché deve essere in multipli di 32 la dimenisone

		memcpy(received_substate.float_buffer,buffer,FLOAT_B1B2_BYTE);
		memcpy(received_substate.int_buffer,  buffer+FLOAT_B1B2_BYTE,INT_B1B2_BYTE);
		memcpy(received_substate.uint_buffer, buffer+FLOAT_B1B2_BYTE+INT_B1B2_BYTE,UINT_B1B2_BYTE);
		status = B2B_OK;

	}

	return status;
}


// ********************************* FUNZIONI PUBBLICHE ************************************

b2b_status b2b_request_state(){
	b2b_protocol_t* b2b = get_b2b_instance();
	b2b_status status = B2B_ERR;

	if( HAL_UART_Receive_IT(b2b->config->uartHandle, b2b->buffer, SUBSTATE_B1_B2_BYTE_SIZE + CRC_SIZE) == HAL_OK){
		b2b_send_RTR();
		status = B2B_OK;
	}
	return status;
}

b2b_status b2b_send_state(){

	b2b_protocol_t* b2b= get_b2b_instance();
	b2b_status status = B2B_ERR;
	// costruire un array di uint8_t in accordo alla tabella e trasmettilo

	//substate_b2_b1 temp;
	capture_board2_substate(&b2b->substate_b2_b1);

	//uint8_t buffer[SUBSTATE_B2_B1_BYTE_SIZE + CRC_SIZE];

	memcpy( b2b->buffer, (uint8_t *)b2b->substate_b2_b1.float_buffer, FLOAT_B2B1_BYTE);				   // temperatura B2 + 4 valori di rpm
	memcpy( &b2b->buffer[ FLOAT_B2B1_BYTE ], (uint8_t *) b2b->substate_b2_b1.int_buffer, UINT_B2B1_BYTE); // direzioni dei motori


//	uint8_t crc_under_test[2] = {5,10};
//	uint32_t crc = HAL_CRC_Calculate(b2b->config->crcHandle, (uint32_t *)crc_under_test, sizeof(crc_under_test)/4);


	uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) b2b->buffer, (uint32_t)(SUBSTATE_B2_B1_BYTE_SIZE/4));  // calcolo il CRC sui dati

	memcpy(&b2b->buffer[CRC_B2B1_INDEX], &crc, CRC_SIZE); 						   // inserisco il CRC infondo al buffer

	if ( HAL_UART_Transmit( &huart1 , b2b->buffer, SUBSTATE_B2_B1_BYTE_SIZE + CRC_SIZE, HAL_MAX_DELAY) == HAL_OK){
		status = B2B_OK;
	}

	return status;
}

void b2b_processing_state(){
	rover_state_t *global_status = rover_state_get_instance();
	b2b_protocol_t* b2b= get_b2b_instance();
	PS2X_StatusTypeDef ps2x_status   =  check_joypad(received_substate.uint_buffer[SUBSTATE_B1B2_CONTROL_VALUE_INDEX]);
	MPU60X0_StatusTypeDef mpu_status = 	check_mpu(received_substate.uint_buffer[SUBSTATE_B1B2_CONTROL_VALUE_INDEX]);
	TEMPERATURE_STATUS temperature_status = check_temperature_values(received_substate.float_buffer[SUBSTATE_B1B2_TEMPB1_INDEX],global_status->temperature_B2);

	if (ps2x_status == PS2X_ERROR || mpu_status == MPU60X0_ERROR || temperature_status == CRITICAL ){
		b2b->computed_intention.next_state = EMERGENCY;
	} else{

		if(mpu_status == MPU60X0_ERROR || temperature_status == ABOVE_NORMAL) {
			b2b->computed_intention.degraded = ACTIVE;
		} else{
			b2b->computed_intention.degraded = INACTIVE;
		}

		global_status->obstacle = nearby_obstacle_direction(&received_substate.float_buffer[SUBSTATE_B1B2_OBSTACLE_DIST_LEFT_INDEX]);

		update_next_state(&received_substate.float_buffer[SUBSTATE_B1B2_OBSTACLE_DIST_LEFT_INDEX],
						  &received_substate.uint_buffer[SUBSTATE_B1B2_CONTROL_BUTTON_INDEX],
						  received_substate.float_buffer[SUBSTATE_B1B2_GYRO_Z],
						  global_status->obstacle,
						  received_substate.int_buffer[SUBSTATE_B1B2_Y_ANALOG_INDEX],
						  received_substate.int_buffer[SUBSTATE_B1B2_X_ANALOG_INDEX],
						  &b2b->substate_b2_b1.float_buffer[SUBSTATE_B2B1_RPM_MOTOR_0_INDEX],
						  &(b2b->computed_intention.next_state), &(b2b->computed_intention.braking));

	}
}


void b2b_update_global_state(){
	rover_state_t *global_status = rover_state_get_instance();
	b2b_protocol_t* b2b= get_b2b_instance();
	// Update dell'intention
	global_status->next_state = b2b->computed_intention.next_state;
	global_status->braking = b2b->computed_intention.braking;
	global_status->degraded = b2b->computed_intention.degraded;
	// Update dello stato globale
	global_status->temperature_B1 = received_substate.float_buffer[SUBSTATE_B1B2_TEMPB1_INDEX];
    global_status->obstacle_distances[0] = received_substate.float_buffer[SUBSTATE_B1B2_OBSTACLE_DIST_LEFT_INDEX];
	global_status->obstacle_distances[1] = received_substate.float_buffer[SUBSTATE_B1B2_OBSTACLE_DIST_CENTER_INDEX];
	global_status->obstacle_distances[2] = received_substate.float_buffer[SUBSTATE_B1B2_OBSTACLE_DIST_RIGHT_INDEX];
	global_status->accX = received_substate.float_buffer[SUBSTATE_B1B2_ACC_X];
	global_status->gyroZ = received_substate.float_buffer[SUBSTATE_B1B2_GYRO_Z];
	global_status->y_analog = received_substate.int_buffer[SUBSTATE_B1B2_Y_ANALOG_INDEX];
	global_status->x_analog = received_substate.int_buffer[SUBSTATE_B1B2_X_ANALOG_INDEX];
	global_status->control_buttons = received_substate.uint_buffer[SUBSTATE_B1B2_CONTROL_BUTTON_INDEX];
	global_status->control_value = received_substate.uint_buffer[SUBSTATE_B1B2_CONTROL_VALUE_INDEX];
	global_status->current_state = global_status->next_state;


}

// ****************************************************************************************************************
