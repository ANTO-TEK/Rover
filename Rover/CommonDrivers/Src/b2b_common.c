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

#include "b2b_common.h"
#include "usart.h"
#include "state_manager.h"
#include "string.h"
#include "crc.h"
#include "math.h"
#include "delayus.h"

#define  PADDING 		(1U)


static b2b_protocol_t b2b;

static b2b_protocol_t * const instance = &b2b;

b2b_protocol_t* const get_b2b_instance(){
	return instance;
}

/**
 * @brief Check b2b_config_t structure pointers for NULL.
 * @param config A pointer to the b2b_config_t structure to check.
 * @return b2b_status Return B2B_OK if all pointers are not NULL, otherwise return B2B_ERR.
 */
static b2b_status __check_config(const b2b_config_t *config) {
    b2b_status status = B2B_ERR; // Assume configuration is invalid initially

    // Check each pointer in the structure. If all pointers are not NULL, set status to B2B_OK.
    if ((config != NULL) &&
        (config->uartHandle != NULL) &&
        (config->crcHandle != NULL) &&
        (config->ack_in_Port != NULL) &&
        (config->rtr_in_Port != NULL) &&
        (config->ack_out_Port != NULL) &&
        (config->rtr_out_Port != NULL) &&
        (config->timer != NULL)) {
        status = B2B_OK;
    }

    return status;
}


b2b_status b2b_protocol_init(const b2b_config_t* config, uint8_t *buffer){
	b2b_status status = B2B_ERR;
	if((config!=NULL) && (buffer!=NULL) && (__check_config(config) == B2B_OK)){
		b2b.config = config;
		b2b.buffer = buffer;
		b2b.last_tick = 0;
		b2b.retransmissions_num = 0;
		b2b.protocol_step = 0;
		b2b.protocol_status = 0;
		b2b.check_time = CHECK_TIME_DISABELED;
		status = B2B_OK;
	}
	return status;

}


b2b_status deserialize_intention(uint8_t* buffer){
	b2b_status status = B2B_ERR;
	uint32_t crc;

	memcpy((uint8_t*)&crc, &buffer[INTENTION_CRC_INDEX], CRC_SIZE);

	if (HAL_CRC_Calculate(b2b.config->crcHandle, (uint32_t*)buffer,(INTENTION_SIZE+PADDING)/sizeof(uint32_t)) == crc){
		memcpy((uint8_t*)&b2b.received_intention, buffer,INTENTION_SIZE);
		status = B2B_OK;
	}

	return status;

}


void inline b2b_send_RTR(){
	HAL_GPIO_WritePin(b2b.config->rtr_out_Port, b2b.config->rtr_out_Pin, 1);
	delay_us(1);
	HAL_GPIO_WritePin(b2b.config->rtr_out_Port, b2b.config->rtr_out_Pin, 0);
}


void inline b2b_send_ACK(){
	HAL_GPIO_WritePin(b2b.config->ack_out_Port, b2b.config->ack_out_Pin, 1);
	delay_us(1);
	HAL_GPIO_WritePin(b2b.config->ack_out_Port, b2b.config->ack_out_Pin, 0);
}

b2b_status b2b_send_intention(){
	b2b_status status = B2B_ERR;
	uint8_t buffer[INTENTION_SIZE+ PADDING + CRC_SIZE];

	buffer[0] = b2b.computed_intention.next_state;
	buffer[1] = b2b.computed_intention.braking;
	buffer[2] = b2b.computed_intention.degraded;
	buffer[3] = 0; // PADDING

	uint32_t crc = HAL_CRC_Calculate(b2b.config->crcHandle, (uint32_t*)buffer, (uint32_t)((INTENTION_SIZE + PADDING)/sizeof(uint32_t)));

	if (	(memcpy(&buffer[INTENTION_CRC_INDEX], (uint8_t *)&crc, CRC_SIZE) == (void *)(&buffer[INTENTION_CRC_INDEX])) &&
			(HAL_UART_Transmit(b2b.config->uartHandle, buffer, INTENTION_SIZE+ PADDING + CRC_SIZE ,HAL_MAX_DELAY) == HAL_OK)){
		status = B2B_OK;
	}
	return status;
}

b2b_status b2b_request_intention(uint8_t *buffer){
	b2b_status status = B2B_ERR;
	if ( HAL_UART_Receive_IT(b2b.config->uartHandle, buffer, INTENTION_SIZE+PADDING+CRC_SIZE ) == HAL_OK){
		b2b_send_RTR();
		status = B2B_OK;
	}

	return status;
}

b2b_status b2b_check_intention(){
	b2b_status status = B2B_ERR;

	if ((b2b.received_intention.next_state == b2b.computed_intention.next_state &&
		 b2b.received_intention.braking == b2b.computed_intention.braking &&
		 b2b.received_intention.degraded == b2b.computed_intention.degraded) &&
		!( (b2b.received_intention.next_state == EMERGENCY) &&  (b2b.computed_intention.next_state == EMERGENCY) )){
		status = B2B_OK;
	}
	return status;
}

// ------------- SECTION CONTAINING THE CHECK ON THE STATUS --------------

TEMPERATURE_STATUS check_temperature_values(float temperature_B1, float temperature_B2){
	TEMPERATURE_STATUS temperature_status = NORMAL;
	  if (fabs(temperature_B1 - temperature_B2) >= 10.0 || ( b2b.check_time == CHECK_TIME_ENABELED && b2b.last_tick - HAL_GetTick() > ABOVE_NORMAL_MAX_TIME)) {
	    temperature_status = CRITICAL;
	  } else {
		  const float max_temp = (temperature_B1 >= temperature_B2)? temperature_B1 : temperature_B2;

	    if ((max_temp < MIN_NORMAL_TEMP) || (max_temp > MAX_NORMAL_TEMP))
	    {
	      if ((max_temp > MAX_NORMAL_TEMP) && (max_temp < CRITICAL_TEMP)) {
	        temperature_status = ABOVE_NORMAL;
	        if(b2b.check_time == CHECK_TIME_DISABELED ){
	        	b2b.check_time = CHECK_TIME_ENABELED;
	        	b2b.last_tick = HAL_GetTick();
	        }
	      } else if (max_temp >= CRITICAL_TEMP) {
	        temperature_status = CRITICAL;
	      }
	    }
	  }
	  if (temperature_status == NORMAL && b2b.check_time == CHECK_TIME_ENABELED) b2b.check_time = CHECK_TIME_DISABELED;
	  return temperature_status;
}

MPU60X0_StatusTypeDef check_mpu(uint8_t control_value){
	MPU60X0_StatusTypeDef status = MPU60X0_ERROR;
	if ( (control_value & MPU_MASK ) == 0){
		status = MPU60X0_OK;
	}
	return status;
}

PS2X_StatusTypeDef check_joypad(uint8_t control_value){
	PS2X_StatusTypeDef status = PS2X_ERROR;
	if ( (control_value & JOYPAD_MASK ) == 0){
		status = PS2X_OK;
	}
	return status;
}

typedef enum {
	OBSTACLE_MINIMUM_SEVERITY = 0,
	OBSTACLE_INTERMEDIATE_SEVERITY,
	OBSTACLE_CRITICAL_SEVERITY,
	OBSTACLE_CANT_MOVE_SEVERITY
} OBSTACLE_SEVERITY;

static inline OBSTACLE_SEVERITY __check_obstacle_severity(float obstacle_distance){
	OBSTACLE_SEVERITY severity;
	if(obstacle_distance < MIN_SIDE_DISTANCE){
		severity = OBSTACLE_CANT_MOVE_SEVERITY;
	} else if (obstacle_distance < NEARBY_OBSTACLE_DISTANCE){
		severity = OBSTACLE_CRITICAL_SEVERITY;
	} else if (obstacle_distance < DISTANT_OBSTACLE_DISTANCE){
		severity = OBSTACLE_INTERMEDIATE_SEVERITY;
	} else {
		severity = OBSTACLE_MINIMUM_SEVERITY;
	}
	return severity;
}

static inline uint8_t __is_stopped(float *rpm_motors){

	rover_state_t *rover_state = rover_state_get_instance();

	if(rover_state->single_board_is_active == 0U){

		return (rpm_motors[0] == 0 &&
				rpm_motors[1] == 0 &&
				rpm_motors[2] == 0 &&
				rpm_motors[3] == 0);

	}

	return 1;

}
/**
 * @brief Check if the rover is in rotation and update the next state accordingly.
 * 
 * @param next_state 		future state of the rover
 * @param braking    		type of braking
 * @param gyroZ      		current value of the Z axis of the MPU
 * @param control_buttons   value of the buttons pressed on the joypad
 * @param dir           	direction of the rotation
 * @param angle         	angle of the rotation
 * @return uint8_t          1 if the rover is in rotation, 0 otherwise
 */
static uint8_t __rotate_angle(ROVER_STATE *next_state, BRAKING_TYPE *braking, float gyroZ,
							  uint8_t *control_buttons, ROVER_STATE dir, float angle){

	static uint8_t in_rotation = 0;
	static float initial_angle = 0;

	rover_state_t *rover_state = rover_state_get_instance();

	if(rover_state->current_state != AVOID_LEFT_OBST && rover_state->current_state != AVOID_RIGHT_OBST
	  && rover_state->current_state != ALTERNATIVE_BACKWARD){
		in_rotation = 0;
	}

	if(in_rotation == 0U){

		initial_angle = gyroZ;
		*next_state = dir;
		in_rotation = 1;

	} else if(abs(gyroZ - initial_angle) >= angle){

		*next_state = BRAKING;
		*braking = HARD;
		*control_buttons = *control_buttons & 0xE7;
		in_rotation = 0;

	}

	return in_rotation;

}

void update_next_state(float *obstacle_distances, uint8_t *control_buttons, float gyroZ, OBSTACLE_DIR obstacle,
		 	 	 	   int8_t y_analog, int8_t x_analog, float *rpm_motors, ROVER_STATE *next_state, BRAKING_TYPE *braking){

	rover_state_t *rover_state = rover_state_get_instance();

	uint8_t radial_rotation_check = (obstacle_distances[0] >= MIN_SIDE_DISTANCE) &&
	    		  	  	  	  	  	(obstacle_distances[1] >= MIN_SIDE_DISTANCE) &&
								    (obstacle_distances[2] >= MIN_SIDE_DISTANCE);

	uint8_t alt_backward_mask = (*control_buttons & ALT_BACKWARD_CMD_MASK) !=0U;

	OBSTACLE_SEVERITY left_severity = 	__check_obstacle_severity(obstacle_distances[0]);
	OBSTACLE_SEVERITY center_severity = __check_obstacle_severity(obstacle_distances[1]);
	OBSTACLE_SEVERITY right_severity = 	__check_obstacle_severity(obstacle_distances[2]);

	static uint8_t alt_backward_is_active = 0U,
				   avoiding_right_obst_is_active = 0U,
				   avoiding_left_obst_is_active = 0U,
	   	   	   	   obstacle_is_critical = 0U;

	obstacle_is_critical = 0;

	if(left_severity >= OBSTACLE_CRITICAL_SEVERITY ||
	   center_severity >= OBSTACLE_CRITICAL_SEVERITY ||
	   right_severity >= OBSTACLE_CRITICAL_SEVERITY){
		obstacle_is_critical = 1U;
	}


	if ((rover_state->current_state == IDLE) && (alt_backward_mask == 1U)) {

	    alt_backward_is_active = 1U;
	}

	if(y_analog >= 0 && obstacle_is_critical) {
		if(obstacle != LEFT_RIGHT && obstacle != NOTONE && ((__is_stopped(rpm_motors) != 1U) || rover_state->current_state != IDLE)){
			if(obstacle != RIGHT_SIDE){
				avoiding_left_obst_is_active = 1U;
			} else {
				avoiding_right_obst_is_active = 1U;
			}
		} else if(__is_stopped(rpm_motors) != 1) {
			*next_state = BRAKING;
			*braking = HARD;
			avoiding_left_obst_is_active = 0;
			avoiding_right_obst_is_active = 0;
		} else {
			*next_state = IDLE;
			*braking = NONE;
			avoiding_left_obst_is_active = 0;
			avoiding_right_obst_is_active = 0;
		}
	}

	if(avoiding_left_obst_is_active == 0 && avoiding_right_obst_is_active == 0 && y_analog < 0){
		if (x_analog > 0){
			*next_state = BACKWARD_RIGHT;
		} else if(x_analog < 0){
			*next_state = BACKWARD_LEFT;
        } else {
			*next_state = BACKWARD;
		}
	}else if(avoiding_left_obst_is_active == 1U){
		  avoiding_left_obst_is_active = __rotate_angle(next_state, braking, gyroZ, control_buttons, AVOID_LEFT_OBST, OBSTACLE_ROT_ANGLE);
		  obstacle_is_critical = 0;

    } else if(avoiding_right_obst_is_active){
		  avoiding_right_obst_is_active = __rotate_angle(next_state, braking, gyroZ, control_buttons, AVOID_RIGHT_OBST, OBSTACLE_ROT_ANGLE);
		  obstacle_is_critical = 0;

	} else if(alt_backward_is_active == 1U){

    	alt_backward_is_active = __rotate_angle(next_state, braking, gyroZ, control_buttons, ALTERNATIVE_BACKWARD, ALT_BACKWARD_ROT_ANGLE);

    } else if (obstacle_is_critical == 0 && y_analog == 0) {

    	if ((x_analog > 0) && (radial_rotation_check == 1U)) {
    		*next_state = ON_RIGHT;
		} else if ((x_analog < 0) && (radial_rotation_check == 1U)) {
			*next_state = ON_LEFT;
		} else if (__is_stopped(rpm_motors) != 1U) {
			*next_state = BRAKING;
			*braking = HARD;
		} else {
			*next_state = IDLE;
			*braking = NONE;
		}

    } else if(obstacle_is_critical == 0 && x_analog > 0) {

	  if(center_severity == OBSTACLE_INTERMEDIATE_SEVERITY ||
	     right_severity == OBSTACLE_INTERMEDIATE_SEVERITY){

		  if (__is_stopped(rpm_motors) != 1U) {
			  *next_state = BRAKING;
			  *braking = SMOOTH;
		  } else {
			  *next_state = IDLE;
			  *braking = NONE;
	      }

	  } else {
		  *next_state = FORWARD_RIGHT;
	}

  } else if(obstacle_is_critical == 0 && x_analog < 0) {

	  if(center_severity == OBSTACLE_INTERMEDIATE_SEVERITY ||
	     left_severity == OBSTACLE_INTERMEDIATE_SEVERITY){

		  if (__is_stopped(rpm_motors) != 1U) {
			  *next_state = BRAKING;
			  *braking = SMOOTH;
		  } else {
			  *next_state = IDLE;
			  *braking = NONE;
		  }

	  } else {
		  *next_state = FORWARD_LEFT;
	  }

  } else if(obstacle_is_critical == 0 && y_analog > 0){

	  if(center_severity == OBSTACLE_INTERMEDIATE_SEVERITY){

		  if (__is_stopped(rpm_motors) != 1U) {
			  *next_state = BRAKING;
			  *braking = SMOOTH;
		  } else {
			  *next_state = IDLE;
			  *braking = NONE;
		  }

	  } else {
		*next_state = FORWARD;
	}

  } else {

	  if(__is_stopped(rpm_motors) == 1U){
	  		*next_state = IDLE;
	  		*braking = NONE;
	  	}
  }

}



OBSTACLE_DIR nearby_obstacle_direction(float* obstacle_distances) {
  OBSTACLE_DIR obstacle;
  obstacle = NOTONE;
  if (obstacle_distances[2] <= NEARBY_OBSTACLE_DISTANCE) {
    if (obstacle_distances[0] <= NEARBY_OBSTACLE_DISTANCE) {
      obstacle = LEFT_RIGHT;
    } else {
      obstacle = RIGHT_SIDE;
    }
  } else if (obstacle_distances[0] <= NEARBY_OBSTACLE_DISTANCE) {
    obstacle = LEFT_SIDE;
  } else if (obstacle_distances[1] <= NEARBY_OBSTACLE_DISTANCE) {
    obstacle = FRONT;
  }

  return obstacle;
}

