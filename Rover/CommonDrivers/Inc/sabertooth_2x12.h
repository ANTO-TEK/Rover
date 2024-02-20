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
 * @file sabertooth_2x12.h
 * @author Antonio Bove && Emilio Amato
 * @brief Sabertooth 2x12 Motor Controller Library for STM32
 * @version 1.0
 * @date 2023-11-18
 *
 * @copyright Copyright (c) 2024
 *
 * This library provides an interface for controlling Sabertooth 2x12 dual motor controllers using STM32 microcontrollers.
 * It includes functions for initializing the controller, setting various parameters, and driving motors in different modes.
 */

#ifndef INC_SABERTOOTH_2X12_H_
#define INC_SABERTOOTH_2X12_H_

#include "usart.h"

#define SABERTOOTH_OK 								(0U)
#define SABERTOOTH_ERR								(-1)
#define SABERTOOTH_INVALID_VALUE					(-2)

/************** COMMANDS ***************/
#define DRIVE_FORWARD_MOTOR_1 		0x00
#define DRIVE_BACKWARD_MOTOR_1 		0x01
#define MIN_VOLTAGE					0x02
#define MAX_VOLTAGE					0x03
#define DRIVE_FORWARD_MOTOR_2 		0x04
#define DRIVE_BACKWARD_MOTOR_2 		0x05
#define DRIVE_MOTOR_1_7BIT			0x06
#define DRIVE_MOTOR_2_7BIT			0x07

/********** MIXED COMMANDS *************/
#define DRIVE_FORWARD_MIXED_MODE	0x08
#define DRIVE_BACKWARD_MIXED_MODE	0x09
#define TURN_RIGHT_MIXED_MODE		0x0A
#define TURN_LEFT_MIXED_MODE		0x0B
#define DRIVE_FORWARD_BACK_7BIT		0x0C
#define TURN_7BIT					0x0D

/********* SETTING COMMANDS ***********/
#define SERIAL_TIMEOUT				0x0E
#define BAUD_RATE					0x0F
#define RAMPING						0x10
#define DEADBAND					0x11

/**************************************/

#define MIN_VALUE_RANGE				(0U)
#define MAX_VALUE_RANGE				(127U)

#define POWER_MIN					MIN_VALUE_RANGE
#define POWER_MAX					MAX_VALUE_RANGE

#define MIN_VOLTAGE_MIN				(0U)
#define MIN_VOLTAGE_MAX				(120U)

#define MAX_VOLTAGE_MIN				MIN_VALUE_RANGE
#define MAX_VOLTAGE_MAX				MAX_VALUE_RANGE

#define MILLISECONDS_TIMEOUT_MIN	(0U)
#define MILLISECONDS_TIMEOUT_MAX	(12700U)

#define DEADBAND_MIN				MIN_VALUE_RANGE
#define DEADBAND_MAX				MAX_VALUE_RANGE

#define RAMPING_MIN					(0U)
#define RAMPING_MAX					(80U)

/************************** Baud Rate enumeration *************************/

typedef enum {
	BAUD_RATE_2400 = 1,
	BAUD_RATE_9600,
	BAUD_RATE_19200,
	BAUD_RATE_38400,
} BAUDRATE_VALUE;

/**************************************************************************/

/*************************** Ramping enumeration **************************/

typedef enum {
	DEFAULT_RAMPING = 0,
	FAST_RAMP_1,
	FAST_RAMP_2,
	FAST_RAMP_3,
	FAST_RAMP_4,
	FAST_RAMP_5,
	FAST_RAMP_6,
	FAST_RAMP_7,
	FAST_RAMP_8,
	FAST_RAMP_9,
	FAST_RAMP_10,
	SLOW_RAMP_11,
	SLOW_RAMP_12,
	SLOW_RAMP_13,
	SLOW_RAMP_14,
	SLOW_RAMP_15,
	SLOW_RAMP_16,
	SLOW_RAMP_17,
	SLOW_RAMP_18,
	SLOW_RAMP_19,
	SLOW_RAMP_20,
	INTERMEDIATE_RAMP_21,
	INTERMEDIATE_RAMP_22,
	INTERMEDIATE_RAMP_23,
	INTERMEDIATE_RAMP_24,
	INTERMEDIATE_RAMP_25,
	INTERMEDIATE_RAMP_26,
	INTERMEDIATE_RAMP_27,
	INTERMEDIATE_RAMP_28,
	INTERMEDIATE_RAMP_29,
	INTERMEDIATE_RAMP_30,
	INTERMEDIATE_RAMP_31,
	INTERMEDIATE_RAMP_32,
	INTERMEDIATE_RAMP_33,
	INTERMEDIATE_RAMP_34,
	INTERMEDIATE_RAMP_35,
	INTERMEDIATE_RAMP_36,
	INTERMEDIATE_RAMP_37,
	INTERMEDIATE_RAMP_38,
	INTERMEDIATE_RAMP_39,
	INTERMEDIATE_RAMP_40,
	INTERMEDIATE_RAMP_41,
	INTERMEDIATE_RAMP_42,
	INTERMEDIATE_RAMP_43,
    INTERMEDIATE_RAMP_44,
	INTERMEDIATE_RAMP_45,
	INTERMEDIATE_RAMP_46,
	INTERMEDIATE_RAMP_47,
	INTERMEDIATE_RAMP_48,
	INTERMEDIATE_RAMP_49,
	INTERMEDIATE_RAMP_50,
	INTERMEDIATE_RAMP_51,
	INTERMEDIATE_RAMP_52,
	INTERMEDIATE_RAMP_53,
	INTERMEDIATE_RAMP_54,
	INTERMEDIATE_RAMP_55,
	INTERMEDIATE_RAMP_56,
	INTERMEDIATE_RAMP_57,
	INTERMEDIATE_RAMP_58,
	INTERMEDIATE_RAMP_59,
	INTERMEDIATE_RAMP_60,
	INTERMEDIATE_RAMP_61,
	INTERMEDIATE_RAMP_62,
	INTERMEDIATE_RAMP_63,
	INTERMEDIATE_RAMP_64,
	INTERMEDIATE_RAMP_65,
	INTERMEDIATE_RAMP_66,
	INTERMEDIATE_RAMP_67,
	INTERMEDIATE_RAMP_68,
	INTERMEDIATE_RAMP_69,
	INTERMEDIATE_RAMP_70,
	INTERMEDIATE_RAMP_71,
	INTERMEDIATE_RAMP_72,
	INTERMEDIATE_RAMP_73,
	INTERMEDIATE_RAMP_74,
	INTERMEDIATE_RAMP_75,
	INTERMEDIATE_RAMP_76,
	INTERMEDIATE_RAMP_77,
	INTERMEDIATE_RAMP_78,
	INTERMEDIATE_RAMP_79,
	INTERMEDIATE_RAMP_80
} RAMPING_VALUE;

/**************************************************************************/

/**************************** Motor enumeration ***************************/

typedef enum {
	MOTOR_1 = 1,
	MOTOR_2
} MOTOR;

/**************************************************************************/

/***************************** Turn enumeration ***************************/

typedef enum {
	LEFT = 0,
	RIGHT
} TURN_DIRECTION;

/**************************************************************************/

/***************************** Mode enumeration ***************************/

typedef enum {
	INDEPENDENT = 1,
	MIXED
} MODE;

/**************************************************************************/

// Struct for Sabertooth configuration
typedef struct {

	uint8_t address;					/**< Sabertooth address */
	UART_HandleTypeDef* uartHandle;	    /**< Pointer to UART_HandleTypeDef for communication */

} sabertooth_config_t;

// Struct for Sabertooth controller
typedef struct {

	sabertooth_config_t *config;		/**< Pointer to the Sabertooth configuration */
	MODE mode;							/**< Current mode of the Sabertooth controller (INDEPENDENT or MIXED) */

} sabertooth_t;

/**
 * @typedef SABERTOOTH_StatusTypeDef
 * @brief Defines the SABERTOOTH_StatusTypeDef type, used to return the status of functions.
 *        It can be SABERTOOTH_OK or SABERTOOTH_ERR or SABERTOOTH_INVALID_VALUE.
 */
typedef int8_t SABERTOOTH_StatusTypeDef;

/******************************** Init method *****************************/

/**
 * @brief Initializes the Sabertooth controller.
 *
 * @param sabertooth Pointer to the sabertooth_t structure.
 * @param sabertooth_config Pointer to the configuration structure.
 * @return SABERTOOTH_OK if the operation is successful, otherwise SABERTOOTH_ERR.
 */

SABERTOOTH_StatusTypeDef sabertooth_init(sabertooth_t* sabertooth, const sabertooth_config_t *sabertooth_config);

/**************************************************************************/

/****************************** Getter method *****************************/

/**
 * @brief Retrieves the address of the Sabertooth controller.
 *
 * @param sabertooth Pointer to the sabertooth_t structure.
 * @param address Pointer to store the address.
 * @return SABERTOOTH_OK if the operation is successful, otherwise SABERTOOTH_ERR.
 */

SABERTOOTH_StatusTypeDef get_address(sabertooth_t* sabertooth, uint8_t* address);


/**
 * @brief Retrieves the UART handle used by the Sabertooth controller.
 *
 * @param sabertooth Pointer to the sabertooth_t structure.
 * @param uartHandle Pointer to store the UART handle.
 * @return SABERTOOTH_OK if the operation is successful, otherwise SABERTOOTH_ERR.
 */

SABERTOOTH_StatusTypeDef get_uartHandle(sabertooth_t* sabertooth, UART_HandleTypeDef* uartHandle);

/**************************************************************************/

/******************************** Commands ********************************/

/**
 * @brief Drives a specific motor forward.
 *
 * @param sabertooth Pointer to the sabertooth_t structure.
 * @param motor Motor to drive (MOTOR_1 or MOTOR_2).
 * @param power Power level (0-100).
 * @return SABERTOOTH_OK if the operation is successful, otherwise SABERTOOTH_ERR.
 */

SABERTOOTH_StatusTypeDef drive_forward_motor(sabertooth_t* sabertooth, MOTOR motor, uint8_t power);

/**
 * @brief Drives a specific motor backward.
 *
 * @param sabertooth Pointer to the sabertooth_t structure.
 * @param motor Motor to drive (MOTOR_1 or MOTOR_2).
 * @param power Power level (0-100).
 * @return SABERTOOTH_OK if the operation is successful, otherwise SABERTOOTH_ERR.
 */

SABERTOOTH_StatusTypeDef drive_backward_motor(sabertooth_t* sabertooth, MOTOR motor, uint8_t power);

/**
 * @brief Drives a specific motor with a 7-bit power level.
 *
 * @param sabertooth Pointer to the sabertooth_t structure.
 * @param motor Motor to drive (MOTOR_1 or MOTOR_2).
 * @param power Power level (0-127).
 * @return SABERTOOTH_OK if the operation is successful, otherwise SABERTOOTH_ERR.
 */

SABERTOOTH_StatusTypeDef drive_motor_7bit(sabertooth_t* sabertooth, MOTOR motor, uint8_t power);

/**
 * @brief Sets the minimum voltage for the Sabertooth controller.
 *
 * @param sabertooth Pointer to the sabertooth_t structure.
 * @param value Minimum voltage value.
 * @return SABERTOOTH_OK if the operation is successful, otherwise SABERTOOTH_ERR.
 */

SABERTOOTH_StatusTypeDef set_min_voltage(sabertooth_t* sabertooth, int8_t value);

/**
 * @brief Sets the maximum voltage for the Sabertooth controller.
 *
 * @param sabertooth Pointer to the sabertooth_t structure.
 * @param value Maximum voltage value.
 * @return SABERTOOTH_OK if the operation is successful, otherwise SABERTOOTH_ERR.
 */

SABERTOOTH_StatusTypeDef set_max_voltage(sabertooth_t* sabertooth, int8_t value);

/**************************************************************************/

/***************************** Mixed Commands *****************************/

/**
 * @brief Drives both motors forward in mixed mode.
 *
 * @param sabertooth Pointer to the sabertooth_t structure.
 * @param power Power level (0-100).
 * @return SABERTOOTH_OK if the operation is successful, otherwise SABERTOOTH_ERR.
 */

SABERTOOTH_StatusTypeDef drive_forward_motors(sabertooth_t* sabertooth, uint8_t power);

/**
 * @brief Drives both motors backward in mixed mode.
 *
 * @param sabertooth Pointer to the sabertooth_t structure.
 * @param power Power level (0-100).
 * @return SABERTOOTH_OK if the operation is successful, otherwise SABERTOOTH_ERR.
 */

SABERTOOTH_StatusTypeDef drive_backward_motors(sabertooth_t* sabertooth, uint8_t power);

/**
 * @brief Drives both motors with a 7-bit power level in mixed mode.
 *
 * @param sabertooth Pointer to the sabertooth_t structure.
 * @param power Power level (0-127).
 * @return SABERTOOTH_OK if the operation is successful, otherwise SABERTOOTH_ERR.
 */

SABERTOOTH_StatusTypeDef drive_motors_7bit(sabertooth_t* sabertooth, uint8_t power);

/**
 * @brief Turns both motors in the specified direction.
 *
 * @param sabertooth Pointer to the sabertooth_t structure.
 * @param direction Direction to turn (LEFT or RIGHT).
 * @param power Power level (0-100).
 * @return SABERTOOTH_OK if the operation is successful, otherwise SABERTOOTH_ERR.
 */

SABERTOOTH_StatusTypeDef turn(sabertooth_t* sabertooth, TURN_DIRECTION direction, uint8_t power);

/**
 * @brief Turns both motors with a 7-bit power level in the specified direction.
 *
 * @param sabertooth Pointer to the sabertooth_t structure.
 * @param power Power level (0-127).
 * @return SABERTOOTH_OK if the operation is successful, otherwise SABERTOOTH_ERR.
 */

SABERTOOTH_StatusTypeDef turn_7bit(sabertooth_t* sabertooth, uint8_t power);

/**************************************************************************/

/**************************** Setting Commands ****************************/

/**
 * @brief Sets the serial timeout for the Sabertooth controller.
 *
 * @param sabertooth Pointer to the sabertooth_t structure.
 * @param milliseconds Timeout value in milliseconds.
 * @return SABERTOOTH_OK if the operation is successful, otherwise SABERTOOTH_ERR.
 */

SABERTOOTH_StatusTypeDef set_serial_timeout(sabertooth_t* sabertooth, int16_t milliseconds);

/**
 * @brief Sets the baud rate for the Sabertooth controller.
 *
 * @param sabertooth Pointer to the sabertooth_t structure.
 * @param baudrate Baud rate value.
 * @return SABERTOOTH_OK if the operation is successful, otherwise SABERTOOTH_ERR.
 */

SABERTOOTH_StatusTypeDef set_baud_rate(sabertooth_t* sabertooth, BAUDRATE_VALUE baudrate);

/**
 * @brief Sets the ramping value for the Sabertooth controller.
 *
 * @param sabertooth Pointer to the sabertooth_t structure.
 * @param ramping Ramping value.
 * @return SABERTOOTH_OK if the operation is successful, otherwise SABERTOOTH_ERR.
 */

SABERTOOTH_StatusTypeDef set_ramping(sabertooth_t* sabertooth, RAMPING_VALUE ramping);

/**
 * @brief Sets the deadband value for the Sabertooth controller.
 *
 * @param sabertooth Pointer to the sabertooth_t structure.
 * @param value Deadband value.
 * @return SABERTOOTH_OK if the operation is successful, otherwise SABERTOOTH_ERR.
 */

SABERTOOTH_StatusTypeDef set_deadband(sabertooth_t* sabertooth, uint8_t value);

/**************************************************************************/

#endif /* INC_SABERTOOTH_2X12_H_ */
