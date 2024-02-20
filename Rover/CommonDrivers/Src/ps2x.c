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

#include <ps2x.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#ifdef USE_INTERRUPT_MODE
	#include "cmsis_os.h"
	#include "event_groups.h"
#endif

#ifdef PS2X_DEBUG

#include "usart.h"

#endif

#define N_PING								(5)

#define DIGITAL_MODE_FLAG					((uint8_t)0x40)
#define ANALOG_MODE_FLAG					((uint8_t)0x70)
#define CONFIG_MODE_FLAG					((uint8_t)0xF0)



// -------------------------------- PRIVATE UTILITY FUNCTIONS -----------------------------
#ifdef USE_INTERRUPT_MODE
extern osEventFlagsId_t controllerEventHandle;

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;

    // Imposta il bit nel gruppo di eventi da un ISR
    xEventGroupSetBitsFromISR(controllerEventHandle, PS2X_TX_RX_CPLT, &pxHigherPriorityTaskWoken);

    // Richiede uno yield del task se un task di priorità superiore è stato risvegliato
    portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;

    // Imposta il bit nel gruppo di eventi da un ISR
    xEventGroupSetBitsFromISR(controllerEventHandle, PS2X_TX_CPLT, &pxHigherPriorityTaskWoken);

    // Richiede uno yield del task se un task di priorità superiore è stato risvegliato
    portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}

/*
 * send a command and receive the response from the controller updating ps2x  object state
 */

PS2X_StatusTypeDef __send_receive_command(PS2X* ps2x, const uint8_t* to_send, uint8_t* to_receive, uint8_t len, uint32_t Timeout){

	#ifdef PS2X_DEBUG
		char debug_string_buffer[200];
	#endif
	PS2X_StatusTypeDef status = PS2X_ERROR;
	EventBits_t event_bits;
	HAL_GPIO_WritePin(ps2x->ATT_Port,ps2x->ATT_GPIO_Pin, GPIO_PIN_RESET);
	if(HAL_SPI_TransmitReceive_IT(ps2x->hspi, to_send, to_receive, len) == HAL_OK){
		event_bits = xEventGroupWaitBits(controllerEventHandle, PS2X_TX_RX_CPLT, pdTRUE, pdTRUE, Timeout);
		if((event_bits & PS2X_TX_RX_CPLT) != 0){
			status = HAL_OK;
		}
	}
	HAL_GPIO_WritePin(ps2x->ATT_Port,ps2x->ATT_GPIO_Pin, GPIO_PIN_SET);

	#ifdef PS2X_DEBUG
		sprintf(debug_string_buffer,"\r\nSEND COMMAND STRING\r\nOUT:IN \r\n");
		for(int i=0; i<len; i++){
			sprintf(debug_string_buffer + strlen(debug_string_buffer),"\r\n%02X:%02X",to_send[i],to_receive[i]);
		}
		HAL_UART_Transmit(ps2x->huart, (uint8_t*)debug_string_buffer, strlen(debug_string_buffer), Timeout);
	#endif
	return status;
}


/*
 *  send a command to the controller
 */

static PS2X_StatusTypeDef __send_command(PS2X* ps2x, const uint8_t* to_send, uint8_t len, uint32_t Timeout){
	//Add error checking
	PS2X_StatusTypeDef status = PS2X_ERROR;
	EventBits_t event_bits;
	#ifdef PS2X_DEBUG
		uint8_t to_receive_debug[PS2X_DATA_DIM];
		status = __send_receive_command(ps2x, to_send, to_receive_debug, len, Timeout);
	#else
		HAL_GPIO_WritePin(ps2x->ATT_Port,ps2x->ATT_GPIO_Pin, GPIO_PIN_RESET);
		if(HAL_SPI_Transmit_IT(ps2x->hspi, to_send, len) == HAL_OK){
			event_bits = xEventGroupWaitBits(controllerEventHandle, PS2X_TX_CPLT, pdTRUE, pdTRUE, Timeout);
			if((event_bits & PS2X_TX_CPLT) != 0){
				status = HAL_OK;
			}
		}
		HAL_GPIO_WritePin(ps2x->ATT_Port,ps2x->ATT_GPIO_Pin, GPIO_PIN_SET);
	#endif
		 return status;
}
#endif


#ifndef USE_INTERRUPT_MODE
/*
 * send a command and receive the response from the controller updating ps2x  object state
 */

PS2X_StatusTypeDef __send_receive_command(PS2X* ps2x, const uint8_t* to_send, uint8_t* to_receive, uint8_t len, uint32_t Timeout){

	#ifdef PS2X_DEBUG
		char debug_string_buffer[200];
	#endif
	PS2X_StatusTypeDef status = PS2X_ERROR;
	HAL_GPIO_WritePin(ps2x->config->ATT_Port, ps2x->config->ATT_GPIO_Pin, GPIO_PIN_RESET);
	if(HAL_SPI_TransmitReceive(ps2x->config->hspi, to_send, to_receive, len, Timeout) == HAL_OK){
			status = HAL_OK;
		}
	HAL_GPIO_WritePin(ps2x->config->ATT_Port, ps2x->config->ATT_GPIO_Pin, GPIO_PIN_SET);

	#ifdef PS2X_DEBUG
		sprintf(debug_string_buffer,"\r\nSEND COMMAND STRING\r\nOUT:IN \r\n");
		for(int i=0; i<len; i++){
			sprintf(debug_string_buffer + strlen(debug_string_buffer, Timeout),"\r\n%02X:%02X",to_send[i],to_receive[i]);
		}
		HAL_UART_Transmit(ps2x->huart, (uint8_t*)debug_string_buffer, strlen(debug_string_buffer), Timeout);
	#endif
	return status;
}


/*
 *  send a command to the controller
 */

static PS2X_StatusTypeDef __send_command(PS2X* ps2x, const uint8_t* to_send, uint8_t len, uint32_t Timeout){
	//Add error checking
	PS2X_StatusTypeDef status = PS2X_ERROR;
	#ifdef PS2X_DEBUG
		uint8_t to_receive_debug[PS2X_DATA_DIM];
		status = __send_receive_command(ps2x, to_send, to_receive_debug, len, Timeout);
	#else
		HAL_GPIO_WritePin(ps2x->config->ATT_Port,ps2x->config->ATT_GPIO_Pin, GPIO_PIN_RESET);
		if(HAL_SPI_Transmit(ps2x->config->hspi, to_send, len, Timeout) == HAL_OK){
			status = HAL_OK;
		}
		HAL_GPIO_WritePin(ps2x->config->ATT_Port, ps2x->config->ATT_GPIO_Pin, GPIO_PIN_SET);
	#endif
		 return status;
}
#endif



/*
 * DA send the commnad that let the controller enter in the configuration mode
 */
static inline PS2X_StatusTypeDef __enter_config_mode(PS2X* ps2x, uint32_t Timeout){
	PS2X_StatusTypeDef status = PS2X_ERROR;
	static const uint8_t enter_config[5]={0x01,0x43,0x00,0x01,0x00};
	status = __send_command(ps2x,enter_config,5,Timeout);
	return status;
}

/*
 *  send the commnad that let the controller exit from the configuration mode
 */
static inline PS2X_StatusTypeDef __exit_config_mode(PS2X* ps2x, uint32_t Timeout){
	PS2X_StatusTypeDef status = PS2X_ERROR;
	static const uint8_t exit_config[9]={0x01,0x43,0x00,0x00,0x5A,0x5A,0x5A,0x5A,0x5A};
	status = __send_command(ps2x,exit_config,9,Timeout);
	return status;
}

static inline PS2X_StatusTypeDef __update_status(PS2X* ps2x, uint32_t Timeout){
	PS2X_StatusTypeDef status = PS2X_ERROR;
	static const uint8_t update_status[2]={0x01,0x42};
	uint8_t received[2] = {0x5A, 0x5A};
	if ( __send_receive_command(ps2x,update_status,received,2, Timeout) == PS2X_OK ){
		uint8_t masked_byte = received[1] & (uint8_t)0xf0;
		if( (masked_byte == DIGITAL_MODE_FLAG) || (masked_byte == ANALOG_MODE_FLAG) || (masked_byte == CONFIG_MODE_FLAG)){
			ps2x->mode = received[1];
			status = PS2X_OK;
		}
		// in this else i can detect that the second byte is not one of the expected.
	}
	return status;
}

/*
 * send the needed command for setting device in analog mode
 */
static inline PS2X_StatusTypeDef __set_digital_mode(PS2X* ps2x, uint32_t Timeout){
	PS2X_StatusTypeDef status = PS2X_ERROR;
	static const uint8_t set_digital_mode[9]={0x01,0x44,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

	 if (
			 __enter_config_mode(ps2x,Timeout) == PS2X_OK &&
			 __send_command(ps2x,set_digital_mode,9,Timeout) == PS2X_OK &&
			 __exit_config_mode(ps2x,Timeout) == PS2X_OK &&
			 __update_status(ps2x,Timeout) == PS2X_OK
	 ){
		 if( (ps2x->mode & (uint8_t)0xf0) == DIGITAL_MODE_FLAG) {
			 status = PS2X_OK;
		 }
	 }

	return status;
}


/*
 * send the needed command for setting device in analog mode
 */
static inline PS2X_StatusTypeDef __set_analog_mode(PS2X* ps2x, uint32_t Timeout){
	PS2X_StatusTypeDef status = PS2X_ERROR;
	static const uint8_t set_analog_mode[9]={0x01,0x44,0x00,0x01,0x03,0x00,0x00,0x00,0x00};

	 if (
			 __enter_config_mode(ps2x,Timeout) == PS2X_OK &&
			 __send_command(ps2x,set_analog_mode,9,Timeout) == PS2X_OK &&
			 __exit_config_mode(ps2x,Timeout) == PS2X_OK &&
			 __update_status(ps2x,Timeout) == PS2X_OK
	 ){
		 if( (ps2x->mode & (uint8_t)0xf0) == ANALOG_MODE_FLAG){
			 status = PS2X_OK;
		 }
	 }

	return status;
}


#ifdef USE_EFFICIENT_NORMALIZE_ANALOG_VALUE

static int8_t normalized_values[256];

static inline int8_t __clamp_int8(int16_t value) {
    int8_t clamped_value;
    static const int8_t min = -128;
    static const int8_t max = 127;
    if (value > max) {
        clamped_value = max;
    } else if (value < min) {
        clamped_value = min;
    } else {
        clamped_value = (int8_t)value;
    }
    return clamped_value;
}

static void __LUT_normalization_init(uint8_t dead_zone) {
    for (int16_t i = 0; i < (int16_t)256; i++) {
        int16_t normalized = i - (int16_t)128;
        if (((uint8_t)abs(normalized)) < dead_zone) {
        	normalized_values[i] = (int8_t) 0;
        } else {
        	normalized_values[i] = __clamp_int8(normalized);
        }
    }
}

static inline int8_t __normalize_analog_value(uint8_t value){
	return normalized_values[value];
}
#endif


#ifndef USE_EFFICIENT_NORMALIZE_ANALOG_VALUE
static int8_t __normalize_analog_value(uint8_t value) {
    static const uint8_t dead_zone = 5;
    int16_t ret = (int16_t)(value - 128);

    if(abs(ret) < dead_zone){
    	ret = 0;
    } else if(ret > 127){
    	ret = 127;
    } else if(ret < -128){
    	ret = -128;
    }

    return (int8_t)ret;
}
#endif



PS2X_StatusTypeDef PS2X_init(PS2X *ps2x, const PS2X_config_t *ps2x_config, uint8_t set_analog, uint32_t Timeout){
	#ifdef USE_EFFICIENT_NORMALIZE_ANALOG_VALUE
		__LUT_normalization_init(ANALOG_DEAD_ZONE);
	#endif
	PS2X_StatusTypeDef status = PS2X_ERROR;
	if((ps2x != NULL) && (ps2x_config != NULL)){
		ps2x->config = ps2x_config;
		ps2x->mode = 0;
		#ifdef PS2X_DEBUG
		ps2x->huart = &huart2;
		#endif

		for(uint8_t i=0; i<PS2X_DATA_DIM; i++){
			ps2x->PS2Xdata[i] = 128;
		}

		HAL_GPIO_WritePin(ps2x_config->ATT_Port, ps2x_config->ATT_GPIO_Pin, GPIO_PIN_RESET);
		if (ping_controller(ps2x, N_PING, Timeout) == PS2X_OK){
			if(set_analog == ((uint8_t) 1U)){
				status = __set_analog_mode(ps2x, Timeout);
			}else{
				status = PS2X_OK;
			}
		}
	}
	return status;
}



PS2X_StatusTypeDef ping_controller(PS2X* ps2x, uint8_t n_ping, uint32_t Timeout){
	static const uint8_t hello_command[5] = {0x01,0x42,0x00,0x00,0x00};
	uint8_t to_receive[5];
	PS2X_StatusTypeDef status = PS2X_ERROR;

	for (uint8_t i = 0; (i < n_ping) && (status!=PS2X_OK); i++) {
		//Send the command to send button and joystick data;
		status =__send_receive_command(ps2x, hello_command, to_receive, 5, Timeout);

	}

	status = __update_status(ps2x,Timeout);

	return status;
}


uint8_t check_button_digital(PS2X* ps2x, uint16_t button){
    uint16_t last_read = (ps2x->PS2Xdata[3] << 8) | ps2x->PS2Xdata[4];
    return !(last_read & button);
}


PS2X_StatusTypeDef read_gamepad_digital(PS2X* ps2x, uint32_t Timeout){
	static const uint8_t digital_poll_command[5] ={0x01,0x42,0x00,0x00,0x00};

	PS2X_StatusTypeDef status = PS2X_OK;
	uint8_t mode_section = ps2x->mode & ((uint8_t)0xf0);

    if ((mode_section != ANALOG_MODE_FLAG) || (mode_section != DIGITAL_MODE_FLAG)) {
        status = __set_digital_mode(ps2x, Timeout);
    }
	if(	status == PS2X_OK &&
		__send_receive_command(ps2x, digital_poll_command,  (uint8_t*) ps2x->PS2Xdata, 5, Timeout)== PS2X_OK &&
		(ps2x->PS2Xdata[1] & ((uint8_t)0x0F)) > ((uint8_t)0U)){
		status = PS2X_OK;
	}
	return status;
}

PS2X_StatusTypeDef read_gamepad_analog(PS2X* ps2x, uint32_t Timeout) {
    static const uint8_t analog_poll_command[9] = {0x01,0x042,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

    PS2X_StatusTypeDef status = PS2X_OK;
    uint8_t mode_section = ps2x->mode & ((uint8_t) 0xf0);

    if (mode_section != ANALOG_MODE_FLAG) {
        status = __set_analog_mode(ps2x, Timeout);
    }

    if (status == PS2X_OK) {
            status = __send_receive_command(ps2x, analog_poll_command, (uint8_t*) ps2x->PS2Xdata, 9, Timeout);
    }

    return status;
}

PS2X_StatusTypeDef get_analog_value(PS2X* ps2x, uint8_t analog_index, Cartesian2D *analog_value) {
    PS2X_StatusTypeDef status = PS2X_ERROR;
    if ((ps2x != NULL) && (analog_value != NULL) && !((analog_index != LEFT_ANALOG) && (analog_index != RIGHT_ANALOG))) {
        analog_value->x = __normalize_analog_value(ps2x->PS2Xdata[analog_index]);
        analog_value->y = __normalize_analog_value(~(ps2x->PS2Xdata[analog_index + ((uint8_t)1U)]));
        status = PS2X_OK;
    }

    return status;
}
