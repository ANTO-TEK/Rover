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
 * @file ps2x.h
 * @author Teodoro Adinolfi && Alessio Guarini
 * @brief Driver for the lynxmotion PS2X controller with an STM32 microcontroller F4 series using HAL as abstraction layer
 * @version 1.0
 * @date 2023-11-24
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef PS2X_H
#define PS2X_H

#include "ps2x_costants.h"
#include "spi.h"


typedef struct {
	GPIO_TypeDef* ATT_Port;
	uint16_t ATT_GPIO_Pin;
	SPI_HandleTypeDef *hspi;
} PS2X_config_t;

/**
 * @struct PS2X
 * @brief This structure is used to store the controller status including the data read from the controller.
 *        It contains the buttons status and the analog stick values.
 *
 * @param huart Pointer to the UART_HandleTypeDef used to print debug messages (only if PS2X_DEBUG is defined).
 * @param mode Indicates the mode of the controller, can be PS2X_DIGITAL_MODE or PS2X_ANALOG_MODE.
 * @param ATT_Port Pointer to the GPIO_TypeDef used to set the ATT (NSS) pin.
 * @param ATT_GPIO_Pin Indicates the pin of the ATT (NSS) pin.
 * @param hspi Pointer to the SPI_HandleTypeDef used to communicate with the controller.
 * @param PS2Xdata Array used to store the data read from the controller.
 */
typedef struct {
    #ifdef PS2X_DEBUG
    UART_HandleTypeDef* huart;
    #endif
    uint8_t mode;
    PS2X_config_t *config;
    uint8_t PS2Xdata[PS2X_DATA_DIM];
} PS2X;

/**
 * @brief Initialize the PS2X structure at the default values 
 *
 *  This function must be called before using the PS2X structure, it initialize the PS2X structure at the default values and set
 *  the ATT pin as output and set it to high level (the controller is not selected so this act as the NSS signal in SPI protocol)
 *  the function also try to ping the controller to check if it is connected and working correctly. Also update the mode variable
 *  of the PS2X structure for check if the controller is in digital or analog mode.
 *
 * @param ps2x Pointer to the PS2X structure connected to the controller.
 * @param ps2x_config Pointer to the PS2X_config_t structure containing the configuration of the controller.
 * @param Timeout Timeout duration for HAL blocking functions in milliseconds.
 * @param set_analog Flag to set the controller in analog mode (1) or digital mode (0).
 *
 * @return PS2X_StatusTypeDef Returns PS2X_OK if initialization is successful, PS2X_ERROR otherwise.
 * @note Ensure that the SPI and GPIO interfaces have been correctly initialized before calling this function.
 */
PS2X_StatusTypeDef PS2X_init(PS2X *ps2x, const PS2X_config_t *ps2x_config, uint8_t set_analog, uint32_t Timeout);

/**
 * @brief ping the controller to check if it is connected and working correctly
 * 
 * @param ps2x a pointer to the PS2X structure connected to the controller
 * @param n_ping indicate the number of ping
 * @param Timeout indicate the timeout used in the HAL  blocking functions
 * @return PS2X_StatusTypeDef  PS2X_OK if the controller is working correctly, PS2X_ERROR otherwise
 */
PS2X_StatusTypeDef ping_controller(PS2X* ps2x, uint8_t n_ping, uint32_t Timeout);

/**
 * @brief Check what button is actually pressed
 * 
 * @param ps2x a pointer to the PS2X structure connected to the controller
 * @param buttons indicate the buttons to check, this parameter must be one of the mask defined in this header file
 * @return uint8_t is 1 if the button is pressed, 0 otherwise
 */
uint8_t check_button_digital(PS2X* ps2x, uint16_t button);

/**
 * @brief Reads the gamepad data from the controller in digital or analog mode.
 * The function sends a 5-byte digital poll command and receives the button states.
 * If the controller is not in digital or analog mode, it sets it to digital mode.
 * 
 * @param ps2x a pointer to the PS2X structure connected to the controller
 * @param Timeout indicate the timeout used in the HAL  blocking functions
 * @return PS2X_StatusTypeDef PS2X_OK if the controller is working correctly, PS2X_ERROR otherwise
 */
PS2X_StatusTypeDef read_gamepad_digital(PS2X* ps2x, uint32_t Timeout);

/**
 * @brief Performs a 9-byte full-duplex communication with the controller to retrieve the status
 * of buttons and analog sticks. If the controller is in digital mode, it is automatically set to
 * analog mode for this reading.
 * 
 * @param ps2x a pointer to the PS2X structure connected to the controller
 * @param Timeout indicate the timeout used in the HAL  blocking functions
 * @return PS2X_StatusTypeDef PS2X_OK if the controller is working correctly, PS2X_ERROR otherwise
 */
PS2X_StatusTypeDef read_gamepad_analog(PS2X* ps2x, uint32_t Timeout);

/**
 * @brief Read the analog stick value from the controller
 * 
 * @param ps2x a pointer to the PS2X structure connected to the controller
 * @param analog_index indicate the analog stick to read, this parameter must be RIGHT_ANALOG or LEFT_ANALOG
 * @param analog_value a pointer to a Cartesian2D structure to store the x and y value of the analog stick
 * @return PS2X_StatusTypeDef PS2X_OK if the controller is working correctly, PS2X_ERROR otherwise
 */
PS2X_StatusTypeDef get_analog_value(PS2X* ps2x,uint8_t analog_index, Cartesian2D *analog_value);

#endif
