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

#include "state_manager.h"
#include "string.h"

/* Initialization of rover_state */
static rover_state_t rover_state = {
    .rpm_motors = {0.0f, 0.0f, 0.0f, 0.0f},                              /* RPM values initialized to 0.0f */
    .obstacle_distances = {500.0f, 500.0f, 500.0f},                   /* Obstacle distances initialized to 5000.0f */
    .temperature_B1 = 25.0f,                                             /* Board 1 temperature initialized to 25.0f */
    .temperature_B2 = 25.0f,                                             /* Board 2 temperature initialized to 25.0f */
    .accX = 0.0f,                                                        /* Acceleration X initialized to 0.0f */
    .gyroZ = 0.0f,                                                       /* Gyroscope Z initialized to 0.0f */
    .braking = NONE,                                                     /* Braking type initialized to NONE */
    .current_state = IDLE,                                               /* Current state initialized to IDLE */
    .next_state = IDLE,                                                  /* Next state initialized to IDLE */
    .degraded = INACTIVE,                                                /* Degraded status initialized to INACTIVE */
    .joypad_status = PS2X_OK,                                            /* Joypad status initialized to PS2X_OK */
    .temperature_status = NORMAL,                                        /* Temperature status initialized to NORMAL */
    .mpu_status = MPU60X0_OK,                                            /* MPU6050 status initialized to MPU60X0_OK */
    .sabertooth_status = SABERTOOTH_OK,                                  /* Sabertooth status initialized to SABERTOOTH_OK */
    .obstacle = NOTONE,                                                  /* Obstacle direction initialized to NOTONE */
    .motors_direction = {DEFAULT, DEFAULT, DEFAULT, DEFAULT},            /* Motor directions initialized to DEFAULT */
    .y_analog = 0,                                                       /* Y-axis analog value initialized to 0 */
    .x_analog = 0,                                                       /* X-axis analog value initialized to 0 */
    .control_buttons = 32,                                                /* Control buttons initialized to 0 */
    .control_value = 0,                                                   /* Control value initialized to 0 */
	.single_board_is_active = 0
};

/* Constant pointer to the RoverFirmwareParams_t instance */
static rover_state_t * const instance = &rover_state;


rover_state_t * const rover_state_get_instance() {
    return instance;
}


substate_b1_b2  capture_board1_substate(){
	substate_b1_b2 temp;

	temp.float_buffer[0] = instance->temperature_B1;
	temp.float_buffer[1] = instance->obstacle_distances[0];
	temp.float_buffer[2] = instance->obstacle_distances[1];
	temp.float_buffer[3] = instance->obstacle_distances[2];
	temp.float_buffer[4] = instance->accX;
	temp.float_buffer[5] = instance->gyroZ;
	// Variabili intere
	temp.int_buffer[0] = instance->y_analog;
	temp.int_buffer[1] = instance->x_analog;

	temp.uint_buffer[0] = instance->control_buttons;
	temp.uint_buffer[1] = instance->control_value;

	return temp;

}

void capture_board2_substate(substate_b2_b1 *temp){
	temp->float_buffer[0] = instance->temperature_B2;
	memcpy(&temp->float_buffer[1],instance->rpm_motors,4*sizeof(float));
	memcpy(temp->int_buffer,instance->motors_direction,4*sizeof(uint8_t));
}
