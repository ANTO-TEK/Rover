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
 * @file state_manager.h
 * @author EA^2T
 * @brief   This file contains the state manager module, which is responsible for managing the state of the rover. 
 *          The state contains all the information about the rover, this kind of information can arrive from the sensors
 *          that are distributed on the rover and that are connected to the two boards.
 * @version 0.1
 * @date 02-04-2024
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef INC_STATE_H_
#define INC_STATE_H_

#include "stdint.h"
#include "geometry.h"
#include "sabertooth_2x12.h"
#include "ps2x_costants.h"
#include "mpu60x0_constants.h"

//--------------------- SUBSTATE B1_B2 -----------------------------

/**
 * @brief  The CRC calculated by the HAL library is an uint32_t we abstract it with a define for a better readability
 */
#define CRC_SIZE   										            (sizeof(uint32_t))

// SUBSTATE_B1_B2 DIMENSIONS
/**
 * @brief The number of float values that are contained in the substate_b1_b2 struct
 */
#define FLOAT_B1B2_NUM									          (6U)
/**
 * @brief The number of int8_t values that are contained in the substate_b1_b2 struct
 */
#define INT_B1B2_NUM									            (2U)
/**
 * @brief The number of uint8_t values that are contained in the substate_b1_b2 struct
 */
#define UINT_B1B2_NUM									            (2U)
/**
 * @brief The number of bytes that are contained in the substate_b1_b2 struct note that
 *        this number is calculated automatically by the compiler based on the number of
 *        float values specified above
 */
#define FLOAT_B1B2_BYTE 								          (sizeof(float)*FLOAT_B1B2_NUM)
/**
 * @brief The number of bytes that are contained in the substate_b1_b2 struct note that
 *       this number is calculated automatically by the compiler based on the number of
 *      int8_t values specified above
 */
#define INT_B1B2_BYTE   								          (sizeof(int8_t)*INT_B1B2_NUM)
/**
 * @brief The number of bytes that are contained in the substate_b1_b2 struct note that
 *       this number is calculated automatically by the compiler based on the number of
 *       uint8_t values specified above
 */
#define UINT_B1B2_BYTE  								          (sizeof(uint8_t)*UINT_B1B2_NUM)
/**
 * @brief The number of bytes that are contained in the substate_b1_b2 struct note that
 *       this number is calculated automatically by the compiler based on the number of
 *       float, int8_t and uint8_t values specified above
 */
#define SUBSTATE_B1_B2_BYTE_SIZE 						      (FLOAT_B1B2_BYTE + INT_B1B2_BYTE + UINT_B1B2_BYTE)
/**
 * @brief The index of the CRC in the substate_b1_b2 struct while sending the data,
 *        this index is used to calculate the CRC and to put it in the right position
 *        in the uint8_t buffer sent with the HAL library via UART
 */
#define CRC_B1B2_INDEX									          SUBSTATE_B1_B2_BYTE_SIZE

// SUBSTATE_B1_B2 FLOAT

/**
 * @brief Index of the temperature of the board 1 in the float buffer of the substate_b1_b2 struct
 */
#define SUBSTATE_B1B2_TEMPB1_INDEX						    (0U)
/**
 * @brief Index of the distance calculated by the left HCSR04 in the float buffer of the substate_b1_b2 struct
 */
#define SUBSTATE_B1B2_OBSTACLE_DIST_LEFT_INDEX		(1U)
/**
 * @brief Index of the distance calculated by the center HCSR04 in the float buffer of the substate_b1_b2 struct
 */
#define SUBSTATE_B1B2_OBSTACLE_DIST_CENTER_INDEX	(2U)
/**
 * @brief Index of the distance calculated by the right HCSR04 in the float buffer of the substate_b1_b2 struct
 */
#define SUBSTATE_B1B2_OBSTACLE_DIST_RIGHT_INDEX		(3U)
/**
 * @brief Index of the accX value in the float buffer of the substate_b1_b2 struct
 */
#define SUBSTATE_B1B2_ACC_X								        (4U)
/**
 * @brief Index of the gyroZ value in the float buffer of the substate_b1_b2 struct
 */
#define SUBSTATE_B1B2_GYRO_Z							        (5U)

// SUBSTATE_B1_B2 INT
/**
 * @brief Index of the y analog value in the int buffer of the substate_b1_b2 struct
 */
#define SUBSTATE_B1B2_Y_ANALOG_INDEX 					    (0U)
/**
 * @brief Index of the x analog value in the int buffer of the substate_b1_b2 struct
 */
#define SUBSTATE_B1B2_X_ANALOG_INDEX 					    (1U)

// SUBSTATE_B1_B2 UINT
/**
 * @brief Index of the control buttons in the uint buffer of the substate_b1_b2 struct
 */
#define SUBSTATE_B1B2_CONTROL_BUTTON_INDEX 				(0U)
/**
 * @brief Index of the control value in the uint buffer of the substate_b1_b2 struct
 */
#define SUBSTATE_B1B2_CONTROL_VALUE_INDEX  				(1U)

//------------------------ SUBSTATE B2_B1 -----------------------

/**
 * @brief The number of float values that are contained in the substate_b2_b1 struct
 */
#define FLOAT_B2B1_NUM									          (5U)
/**
 * @brief The number of uint8_t values that are contained in the substate_b2_b1 struct
 */
#define UINT_B2B1_NUM									            (4U)
/**
 * @brief The number of bytes that are contained in the substate_b2_b1 struct note that
 *        this number is calculated automatically by the compiler based on the number of
 *        float values specified above
 */
#define FLOAT_B2B1_BYTE 								          (sizeof(float)*FLOAT_B2B1_NUM)
/**
 * @brief The number of bytes that are contained in the substate_b2_b1 struct note that
 *       this number is calculated automatically by the compiler based on the number of
 *       uint8_t values specified above
 */
#define UINT_B2B1_BYTE  								          (sizeof(uint8_t)*UINT_B2B1_NUM)
/**
 * @brief The number of bytes that are contained in the substate_b2_b1 struct note that
 *       this number is calculated automatically by the compiler based on the number of
 *       float and uint8_t values specified above
 */
#define SUBSTATE_B2_B1_BYTE_SIZE						      (FLOAT_B2B1_BYTE + UINT_B2B1_BYTE)
/**
 * @brief The index of the CRC in the substate_b2_b1 struct while sending the data,
 *        this index is used to calculate the CRC and to put it in the right position
 *        in the uint8_t buffer sent with the HAL library via UART
 */
#define CRC_B2B1_INDEX									          SUBSTATE_B2_B1_BYTE_SIZE
/**
 * @brief Index of the temperature of the board 2 in the float buffer of the substate_b2_b1 struct
 */
#define SUBSTATE_B2B1_TEMPB2_INDEX						    (0U)
/**
 * @brief Index of the rpm of the motor 0 in the float buffer of the substate_b2_b1 struct
 */
#define SUBSTATE_B2B1_RPM_MOTOR_0_INDEX					  (1U)
/**
 * @brief Index of the rpm of the motor 1 in the float buffer of the substate_b2_b1 struct
 */
#define SUBSTATE_B2B1_RPM_MOTOR_1_INDEX					  (2U)
/**
 * @brief Index of the rpm of the motor 2 in the float buffer of the substate_b2_b1 struct
 */
#define SUBSTATE_B2B1_RPM_MOTOR_2_INDEX					  (3U)
/**
 * @brief Index of the rpm of the motor 3 in the float buffer of the substate_b2_b1 struct
 */
#define SUBSTATE_B2B1_RPM_MOTOR_3_INDEX					  (4U)
/**
 * @brief Index of the motor direction 0 in the uint buffer of the substate_b2_b1 struct
 */
#define SUBSTATE_B2B1_MOTOR_DIRECTION_O_INDEX			(0U)
/**
 * @brief Index of the motor direction 1 in the uint buffer of the substate_b2_b1 struct
 */
#define SUBSTATE_B2B1_MOTOR_DIRECTION_1_INDEX			(1U)
/**
 * @brief Index of the motor direction 2 in the uint buffer of the substate_b2_b1 struct
 */
#define SUBSTATE_B2B1_MOTOR_DIRECTION_2_INDEX			(2U)
/**
 * @brief Index of the motor direction 3 in the uint buffer of the substate_b2_b1 struct
 */
#define SUBSTATE_B2B1_MOTOR_DIRECTION_3_INDEX			(3U)

// ------------------------ INTENTION -------------------------------
/**
 * @brief Size of the intention buffer
 */
#define INTENTION_SIZE              			        (3U)

/**
 * @brief Index of the CRC in the intention buffer
 */
#define INTENTION_CRC_INDEX					        	(4U)


/****************** MASCHERE CONTROBUTTON ******************/


typedef uint8_t control_buttons_mask_t;

#define R_WHITE_LED_MASK 					(0b00000001U)
#define L_WHITE_LED_MASK 					(0b00000010U)
#define BOTH_WHITE_LED_MASK 				(0b00000100U)
#define ALT_BACKWARD_COMBO_MASK 			(0b00001000U)
#define ALT_BACKWARD_CMD_MASK 				(0b00010000U)
#define EMERGENCY_STOP_MASK 				(0b00100000U)
#define BUZZER_MASK 						(0b01000000U)



/**
 * @brief The BRAKING_TYPE enum is used to specify the type of braking that the rover has to perform.
 */
typedef enum {
  SMOOTH = 0,
  HARD,
  NONE
} BRAKING_TYPE;

/**
 * @brief The ROVER_STATE enum is used to specify the state of the rover, in every moment the phisical state of the rover
 *        is represented by one of the values of this enum.
 */
typedef enum {
  IDLE = 0,
  FORWARD,
  ON_RIGHT,
  ON_LEFT,
  FORWARD_RIGHT,
  FORWARD_LEFT,
  BACKWARD,
  BACKWARD_RIGHT,
  BACKWARD_LEFT,
  BRAKING,
  AVOID_RIGHT_OBST,
  AVOID_LEFT_OBST,
  ALTERNATIVE_BACKWARD,
  EMERGENCY
} ROVER_STATE;

/**
 * @brief The DEGRADED_STATUS enum is used to specify the status of the rover, if the rover is in a degraded status this means
 *        that is not working properly and limuted functionalities are available.
 */
typedef enum {
  ACTIVE = 0,
  INACTIVE = -1
} DEGRADED_STATUS;

/**
 * @brief The TEMPERATURE_STATUS enum is used to specify the status of the temperature of the rover, if the temperature is normal
 *        the rover is working properly, if the temperature is above normal the rover is still working but after staing too much time
 *        in this status the rover will be in a critical status. If the temperature is critical the rover is not able to perform it's
 *       functionalities.
 */
typedef enum {
  NORMAL = 0,
  ABOVE_NORMAL,
  CRITICAL
} TEMPERATURE_STATUS;

/**
 * @brief The OBSTACLE_DIR enum is used to specify the direction of the obstacle that the rover has to avoid.
 */
typedef enum {
  NOTONE = 0,
  LEFT_SIDE,
  RIGHT_SIDE,
  FRONT,
  LEFT_RIGHT
} OBSTACLE_DIR;

/**
 * @brief The MOTOR_DIR enum is used to specify the direction of the motors.
 */
typedef enum {
  DEFAULT = 0,
  BACKWARD_DIR,
  FORWARD_DIR
} MOTOR_DIR;

/**
 * @brief The substate_b1_b2 struct is used to store the information that comes from the board 1 and that is sent to the board 2.
 *        This struct is made by three buffers, this choice is made to semplify the creation of a single uint8_t buffer that is sent
 *        via UART with the HAL library.
 */
typedef struct{
	float  float_buffer[FLOAT_B1B2_NUM];
	int8_t int_buffer[INT_B1B2_NUM];
	uint8_t uint_buffer[UINT_B1B2_NUM];
} substate_b1_b2;

/**
 * @brief The substate_b2_b1 struct is used to store the information that comes from the board 2 and that is sent to the board 1.
 *        This struct is made by two buffers, this choice is made to semplify the creation of a single uint8_t buffer that is sent
 *        via UART with the HAL library.
 */
typedef struct {

	float float_buffer[FLOAT_B2B1_NUM]; 
	uint8_t int_buffer[UINT_B2B1_NUM];  

} substate_b2_b1;

/**
 * @brief The rover_state_t struct is used to store all the information about the rover,
 *        including the state, intention, sensor status, and actuator status.
 */
typedef struct {
    float rpm_motors[4];                                          /**< RPM values for the four motors. */
    float obstacle_distances[3];                                  /**< Distances from the left center and right obstacle sensors. */
    float temperature_B1,                                         /**< Temperature read from the board1 CPU */
          temperature_B2;                                         /**< Temperature read from the board2 CPU */
    float accX,                                                   /**< Acceleration along the X-axis. */
          gyroZ;                                                  /**< Gyroscope reading along the Z-axis. */
    BRAKING_TYPE braking;                                         /**< Type of braking that must be performed. */
    ROVER_STATE current_state;                                    /**< Current phisical state of the rover. */
    ROVER_STATE next_state;                                       /**< Next intended state of the rover. */
    DEGRADED_STATUS degraded;                                     /**< Degraded status of the rover active or not. */
    PS2X_StatusTypeDef joypad_status;                             /**< Joypad status. */
    TEMPERATURE_STATUS temperature_status;                        /**< Temperature sensor status. */
    MPU60X0_StatusTypeDef mpu_status;                             /**< MPU6050 sensor status. */
    SABERTOOTH_StatusTypeDef sabertooth_status;                   /**< Sabertooth motor controller status. */
    OBSTACLE_DIR obstacle;                                        /**< Direction of detected obstacle. */
    uint8_t motors_direction[4];                                  /**< Direction of each motor. */
    int8_t y_analog;                                              /**< Y-axis value from analog joystick. */
    int8_t x_analog;                                              /**< X-axis value from analog joystick. */
    uint8_t control_buttons;                                      /**< Control button pressed, the meaning of each bit is specified on the documentation */
    uint8_t control_value;                                        /**< Each bit is associated with the status of a sensor. */
    uint8_t single_board_is_active;                               /**< Flag to indicate if the single board is active. */
} rover_state_t;

/**
 * @brief The rover_intention_t struct is used to store the intention of the single board, this struct is used to send the intention
 *        from the board 1 to the board 2 and viceversa. Based on the same view of the system each board has to agree on the next state
 *        and the type of braking that has to be eventualy performed.
 */
typedef struct {

	ROVER_STATE next_state;
	BRAKING_TYPE braking;
	DEGRADED_STATUS degraded;

} rover_intention_t;


/**
 * @brief The rover_state_get_instance function is used to get the single instance of the rover state. 
 * @return rover_state_t* pointer to the instance of the rover state.
 */
rover_state_t * const rover_state_get_instance();

/**
 * @brief  This function is used to capture the portion of the state that is mantained by the board 1 and that has to be sent to the board 2.
 * @return substate_b1_b2 the substate that has to be sent to the board 2. 
 */
substate_b1_b2   capture_board1_substate();

/**
 * @brief This function is used to capture the portion of the state that is mantained by the board 2 and that has to be sent to the board 1.
 * @return substate_b2_b1 the substate that has to be sent to the board 1.
 */
void capture_board2_substate(substate_b2_b1 *temp);
#endif
