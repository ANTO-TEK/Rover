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

#ifndef PS2X_COSTANTS_H
#define PS2X_COSTANTS_H

#include <stdint.h>
#include "gpio.h"
#include "geometry.h"

//#define PS2X_DEBUG

#define PS2X_TX_RX_CPLT	((const EventBits_t) 1<<0)
#define PS2X_TX_CPLT	((const EventBits_t) 2<<0)

/**
 * @def USE_EFFICIENT_NORMALIZE_ANALOG_VALUE
 * @brief Uncomment this to use an efficient normalization method for analog values.
 * @warning This causes higher memory usage because an additional array of 256 bytes will be allocated statically.
 */
#define USE_EFFICIENT_NORMALIZE_ANALOG_VALUE
//#define USE_INTERRUPT_MODE
/**
 * @def PS2X_X
 * @brief Mask for the X button.
 */
#define PS2X_X        ((uint16_t)0x0040U)

/**
 * @def PS2X_SQUARE
 * @brief Mask for the Square button.
 */
#define PS2X_SQUARE   ((uint16_t)0x0080U)

/**
 * @def PS2X_TRIANGLE
 * @brief Mask for the Triangle button.
 */
#define PS2X_TRIANGLE ((uint16_t)0x0010U)

/**
 * @def PS2X_CIRCLE
 * @brief Mask for the Circle button.
 */
#define PS2X_CIRCLE   ((uint16_t)0x0020U)

/**
 * @def PS2X_R1
 * @brief Mask for the R1 button.
 */
#define PS2X_R1       ((uint16_t)0x0008U)

/**
 * @def PS2X_R2
 * @brief Mask for the R2 button.
 */
#define PS2X_R2       ((uint16_t)0x0002U)

/**
 * @def PS2X_L1
 * @brief Mask for the L1 button.
 */
#define PS2X_L1       ((uint16_t)0x0004U)

/**
 * @def PS2X_L2
 * @brief Mask for the L2 button.
 */
#define PS2X_L2       ((uint16_t)0x0001U)

/**
 * @def PS2X_UP
 * @brief Mask for the Up button.
 */
#define PS2X_UP       ((uint16_t)0x1000U)

/**
 * @def PS2X_RIGHT
 * @brief Mask for the Right button.
 */
#define PS2X_RIGHT    ((uint16_t)0x2000U)

/**
 * @def PS2X_DOWN
 * @brief Mask for the Down button.
 */
#define PS2X_DOWN     ((uint16_t)0x4000U)

/**
 * @def PS2X_LEFT
 * @brief Mask for the Left button.
 */
#define PS2X_LEFT     ((uint16_t)0x8000U)

/**
 * @def PS2X_START
 * @brief Mask for the Start button.
 */
#define PS2X_START    ((uint16_t)0x0800U)

/**
 * @def PS2X_SELECT
 * @brief Mask for the Select button.
 */
#define PS2X_SELECT   ((uint16_t)0x0100U)

/**
 * @def RIGHT_ANALOG
 * @brief Indicates the position in the PS2Xdata array of the right analog stick value.
 */
#define RIGHT_ANALOG  ((uint8_t)5U)

/**
 * @def LEFT_ANALOG
 * @brief Indicates the position in the PS2Xdata array of the left analog stick value.
 */
#define LEFT_ANALOG   ((uint8_t)7U)

/**
 * @def PS2X_DATA_DIM
 * @brief Defines the dimension of the PS2X data array.
 */
#define PS2X_DATA_DIM ((uint8_t)9U)

/**
 * @def ANALOG_DEAD_ZONE
 * @brief Defines the dead zone value for the analog sticks.
 */
#define ANALOG_DEAD_ZONE ((uint8_t)4U)

/**
 * @typedef PS2X_StatusTypeDef
 * @brief Defines the PS2X_StatusTypeDef type, used to return the status of functions.
 *        It can be PS2X_OK or PS2X_ERROR.
 */
typedef uint8_t PS2X_StatusTypeDef;


 /**
  * @def PS2X_OK
  * @brief Indicates that the function is correctly executed.
  */
 #define PS2X_OK       ((PS2X_StatusTypeDef) 0)

 /**
  * @def PS2X_ERROR
  * @brief Indicates that the function is not correctly executed.
  */
 #define PS2X_ERROR    ((PS2X_StatusTypeDef) 1)


#endif
