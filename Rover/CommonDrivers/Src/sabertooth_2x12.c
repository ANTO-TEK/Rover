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

#include <stdbool.h>
#include "sabertooth_2x12.h"

#define IS_VALID_MOTOR_VALUE(VALUE)			(((VALUE) == MOTOR_1) || ((VALUE) == MOTOR_2))

#define IS_VALID_DIRECTION_VALUE(VALUE)		(((VALUE) == LEFT) || ((VALUE) == RIGHT))

#define IS_VALID_BAUDRATE_VALUE(VALUE)		(((VALUE) == BAUD_RATE_2400) ||\
											((VALUE) == BAUD_RATE_9600)  ||\
											((VALUE) == BAUD_RATE_19200) ||\
											((VALUE) == BAUD_RATE_38400))

#define IS_VALID_RAMPING_VALUE(VALUE)		((VALUE) >= RAMPING_MIN && (VALUE) <= RAMPING_MAX)

#define IS_VALID_POWER_VALUE(VALUE)			((VALUE) >= POWER_MIN && (VALUE) <= POWER_MAX)

#define IS_VALID_MIN_VOLTAGE_VALUE(VALUE)	((VALUE) >= MIN_VOLTAGE_MIN && (VALUE) <= MIN_VOLTAGE_MAX)

#define IS_VALID_MAX_VOLTAGE_VALUE(VALUE)	((VALUE) >= MAX_VOLTAGE_MIN && (VALUE) <= MAX_VOLTAGE_MAX)

#define IS_VALID_DEADBAND_VALUE(VALUE)		((VALUE) >= DEADBAND_MIN && (VALUE) <= DEADBAND_MAX)

#define assert_parameter(expr)				((expr) ? SABERTOOTH_OK : SABERTOOTH_INVALID_VALUE)

#define MILLISECONDS_PER_UNIT				(100)

#define BUFFER_LEN							(4)

/**************************** Private methods *****************************/

/**
 * @brief Sends a command to the Sabertooth motor controller.
 *
 * @param sabertooth Pointer to the sabertooth_t structure.
 * @param command Command byte to be sent.
 * @param value Value byte to be sent.
 * @return SABERTOOTH_OK if the operation is successful, otherwise SABERTOOTH_ERR.
 */

static inline SABERTOOTH_StatusTypeDef __command(sabertooth_t* sabertooth, uint8_t command, uint8_t value){

	SABERTOOTH_StatusTypeDef status = SABERTOOTH_ERR;

	uint8_t checksum = (sabertooth->config->address | command | value) & 0b01111111U;

	uint8_t buffer[BUFFER_LEN] = {sabertooth->config->address, command, value, checksum};

	if(HAL_UART_Transmit(sabertooth->config->uartHandle, (uint8_t*)buffer, BUFFER_LEN, 1000) == HAL_OK){
		status = SABERTOOTH_OK;
	}

	return status;

}

/**
 * @brief Converts time in milliseconds to the corresponding value for Sabertooth's serial timeout command.
 *
 * @param milliseconds Time in milliseconds.
 * @return Sabertooth serial timeout value.
 */

static inline SABERTOOTH_StatusTypeDef __from_milliseconds_to_value(int16_t milliseconds) {

    if (milliseconds < MILLISECONDS_TIMEOUT_MIN) {
        milliseconds = MILLISECONDS_TIMEOUT_MIN;
    } else if (milliseconds > MILLISECONDS_TIMEOUT_MAX) {
        milliseconds = MILLISECONDS_TIMEOUT_MAX;
    }
    return (uint8_t)((milliseconds + 99) / MILLISECONDS_PER_UNIT);

}

/**
 * @brief Drives the motors forward in mixed mode.
 *
 * @param sabertooth Pointer to the sabertooth_t structure.
 * @param power Power level (0-100).
 * @return SABERTOOTH_OK if the operation is successful, otherwise SABERTOOTH_ERR.
 */

static SABERTOOTH_StatusTypeDef __forward_mixed_mode(sabertooth_t* sabertooth, uint8_t power){

	return __command(sabertooth, DRIVE_FORWARD_MIXED_MODE, power);

}

/**
 * @brief Drives the motors backward in mixed mode.
 *
 * @param sabertooth Pointer to the sabertooth_t structure.
 * @param power Power level (0-100).
 * @return SABERTOOTH_OK if the operation is successful, otherwise SABERTOOTH_ERR.
 */

static SABERTOOTH_StatusTypeDef __backward_mixed_mode(sabertooth_t* sabertooth, uint8_t power){

	return __command(sabertooth, DRIVE_BACKWARD_MIXED_MODE, power);

}

/**
 * @brief Turns the motors to the left in mixed mode.
 *
 * @param sabertooth Pointer to the sabertooth_t structure.
 * @param power Power level (0-100).
 * @return SABERTOOTH_OK if the operation is successful, otherwise SABERTOOTH_ERR.
 */

static SABERTOOTH_StatusTypeDef __turn_left(sabertooth_t* sabertooth, uint8_t power){

	return __command(sabertooth, TURN_LEFT_MIXED_MODE, power);

}

/**
 * @brief Turns the motors to the right in mixed mode.
 *
 * @param sabertooth Pointer to the sabertooth_t structure.
 * @param power Power level (0-100).
 * @return SABERTOOTH_OK if the operation is successful, otherwise SABERTOOTH_ERR.
 */

static SABERTOOTH_StatusTypeDef __turn_right(sabertooth_t* sabertooth, uint8_t power){

	return __command(sabertooth, TURN_RIGHT_MIXED_MODE, power);

}

/**
 * @brief Sets the Sabertooth controller to mixed mode.
 *
 * @param sabertooth Pointer to the sabertooth_t structure.
 * @return SABERTOOTH_OK if the operation is successful, otherwise SABERTOOTH_ERR.
 */

static inline SABERTOOTH_StatusTypeDef __set_mixed_mode(sabertooth_t* sabertooth){

	SABERTOOTH_StatusTypeDef status = SABERTOOTH_ERR;

	if(sabertooth->mode != MIXED){

		if(
				__forward_mixed_mode(sabertooth, 0) == SABERTOOTH_OK &&
				__turn_left(sabertooth, 0) == SABERTOOTH_OK
		  ){
			    sabertooth->mode = MIXED;
				status = SABERTOOTH_OK;
		   }
	}else{
		status = SABERTOOTH_OK;
	}

	return status;

}

/**
 * @brief Sets the Sabertooth controller to independent mode.
 *
 * @param sabertooth Pointer to the sabertooth_t structure.
 */

static inline void __set_independent_mode(sabertooth_t* sabertooth){

	if(sabertooth->mode != INDEPENDENT){
		sabertooth->mode = INDEPENDENT;
	}

}

/**************************************************************************/

/**************************** Public methods ******************************/

SABERTOOTH_StatusTypeDef sabertooth_init(sabertooth_t* sabertooth, const sabertooth_config_t* sabertooth_config){

	SABERTOOTH_StatusTypeDef status = SABERTOOTH_ERR;

	if(sabertooth && sabertooth_config){
		sabertooth->config = sabertooth_config;
		sabertooth->mode = INDEPENDENT;
		status = SABERTOOTH_OK;
	}

	return status;
}

inline SABERTOOTH_StatusTypeDef get_address(sabertooth_t* sabertooth, uint8_t* address){

	SABERTOOTH_StatusTypeDef status = SABERTOOTH_ERR;

	if(sabertooth){
		*address = sabertooth->config->address;
		status = SABERTOOTH_OK;
	}

	return status;

}

inline SABERTOOTH_StatusTypeDef get_uartHandle(sabertooth_t* sabertooth, UART_HandleTypeDef* uartHandle){

	SABERTOOTH_StatusTypeDef status = SABERTOOTH_ERR;

	if(sabertooth){
		uartHandle = sabertooth->config->uartHandle;
		status = SABERTOOTH_OK;
	}

	return status;
}

SABERTOOTH_StatusTypeDef drive_forward_motor(sabertooth_t* sabertooth, MOTOR motor, uint8_t power){

	SABERTOOTH_StatusTypeDef status = SABERTOOTH_ERR;

	if(sabertooth){

		status = assert_parameter(IS_VALID_MOTOR_VALUE(motor)) | assert_parameter(IS_VALID_POWER_VALUE(power));

		if(status == SABERTOOTH_OK){

			__set_independent_mode(sabertooth);
			status = __command(sabertooth, motor == MOTOR_1 ? DRIVE_FORWARD_MOTOR_1 : DRIVE_FORWARD_MOTOR_2, power);

		}
	}

	return status;

}

SABERTOOTH_StatusTypeDef drive_backward_motor(sabertooth_t* sabertooth, MOTOR motor, uint8_t power){

	SABERTOOTH_StatusTypeDef status = SABERTOOTH_ERR;

	if(sabertooth){

		status = assert_parameter(IS_VALID_MOTOR_VALUE(motor)) | assert_parameter(IS_VALID_POWER_VALUE(power));

		if(status == SABERTOOTH_OK){

			__set_independent_mode(sabertooth);
			status = __command(sabertooth, motor == MOTOR_1 ? DRIVE_BACKWARD_MOTOR_1 : DRIVE_BACKWARD_MOTOR_2, power);
		}
	}

	return status;

}

SABERTOOTH_StatusTypeDef drive_motor_7bit(sabertooth_t* sabertooth, MOTOR motor, uint8_t power){

	SABERTOOTH_StatusTypeDef status = SABERTOOTH_ERR;

	if(sabertooth){

		status = assert_parameter(IS_VALID_MOTOR_VALUE(motor)) | assert_parameter(IS_VALID_POWER_VALUE(power));

		if(status == SABERTOOTH_OK){

			__set_independent_mode(sabertooth);
			status = __command(sabertooth, motor == MOTOR_1 ? DRIVE_MOTOR_1_7BIT : DRIVE_MOTOR_2_7BIT, power);

		}
	}

	return status;

}

SABERTOOTH_StatusTypeDef set_min_voltage(sabertooth_t* sabertooth, int8_t value){

	SABERTOOTH_StatusTypeDef status = SABERTOOTH_ERR;

	if(sabertooth){

		status = assert_parameter(IS_VALID_MIN_VOLTAGE_VALUE(value));

		if(status == SABERTOOTH_OK){

			status = __command(sabertooth, MIN_VOLTAGE, value);

		}
	}

	return status;

}

SABERTOOTH_StatusTypeDef set_max_voltage(sabertooth_t* sabertooth, int8_t value){

	SABERTOOTH_StatusTypeDef status = SABERTOOTH_ERR;

	if(sabertooth){

		status = assert_parameter(IS_VALID_MAX_VOLTAGE_VALUE(value));

		if(status == SABERTOOTH_OK){

			status = __command(sabertooth, MAX_VOLTAGE, value);

		}
	}

	return status;

}

SABERTOOTH_StatusTypeDef drive_forward_motors(sabertooth_t* sabertooth, uint8_t power){

	SABERTOOTH_StatusTypeDef status = SABERTOOTH_ERR;

	if(sabertooth){

		status = assert_parameter(IS_VALID_POWER_VALUE(power));

		if(status == SABERTOOTH_OK){

			if(__set_mixed_mode(sabertooth) == SABERTOOTH_OK)
				status = __forward_mixed_mode(sabertooth, power);
			else{
				status = SABERTOOTH_ERR;
			}

		}
	}

	return status;

}

SABERTOOTH_StatusTypeDef drive_backward_motors(sabertooth_t* sabertooth, uint8_t power){

	SABERTOOTH_StatusTypeDef status = SABERTOOTH_ERR;

	if(sabertooth){

		status = assert_parameter(IS_VALID_POWER_VALUE(power));

		if(status == SABERTOOTH_OK){

			if(__set_mixed_mode(sabertooth) == SABERTOOTH_OK)
				status = __backward_mixed_mode(sabertooth, power);
			else{
				status = SABERTOOTH_ERR;
			}
		}
	}

	return status;

}

SABERTOOTH_StatusTypeDef drive_motors_7bit(sabertooth_t* sabertooth, uint8_t power){

	SABERTOOTH_StatusTypeDef status = SABERTOOTH_ERR;

	if(sabertooth){

		status = assert_parameter(IS_VALID_POWER_VALUE(power));

		if(status == SABERTOOTH_OK){

			if(__set_mixed_mode(sabertooth) == SABERTOOTH_OK)
				status = __command(sabertooth, DRIVE_FORWARD_BACK_7BIT, power);
			else{
				status = SABERTOOTH_ERR;
			}
		}
	}

	return status;

}

SABERTOOTH_StatusTypeDef turn(sabertooth_t* sabertooth, TURN_DIRECTION direction, uint8_t power){

	SABERTOOTH_StatusTypeDef status = SABERTOOTH_ERR;

	if(sabertooth){

		status = assert_parameter(IS_VALID_DIRECTION_VALUE(direction)) | assert_parameter(IS_VALID_POWER_VALUE(power));

		if(status == SABERTOOTH_OK){

			if(__set_mixed_mode(sabertooth) == SABERTOOTH_OK){

				switch(direction){
					case LEFT:
						status = __turn_left(sabertooth, power);
						break;
					case RIGHT:
						status = __turn_right(sabertooth, power);
						break;
				}

			}else{
				status = SABERTOOTH_ERR;
			}
		}
	}

	return status;

}

SABERTOOTH_StatusTypeDef turn_7bit(sabertooth_t* sabertooth, uint8_t power){

	SABERTOOTH_StatusTypeDef status = SABERTOOTH_ERR;

	if(sabertooth){

		status = assert_parameter(IS_VALID_POWER_VALUE(power));

		if(status == SABERTOOTH_OK){

			if(__set_mixed_mode(sabertooth) == SABERTOOTH_OK)
				status = __command(sabertooth, TURN_7BIT, power);
			else
				status = SABERTOOTH_ERR;
		}
	}

	return status;

}

SABERTOOTH_StatusTypeDef set_serial_timeout(sabertooth_t* sabertooth, int16_t milliseconds){

	SABERTOOTH_StatusTypeDef status = SABERTOOTH_ERR;

	if(sabertooth){

		status = __command(sabertooth, SERIAL_TIMEOUT, __from_milliseconds_to_value(milliseconds));

	}

	return status;

}

SABERTOOTH_StatusTypeDef set_baud_rate(sabertooth_t* sabertooth, BAUDRATE_VALUE baudrate){

	SABERTOOTH_StatusTypeDef status = SABERTOOTH_ERR;

	if(sabertooth){

		status = assert_parameter(IS_VALID_BAUDRATE_VALUE(baudrate));

		if(status == SABERTOOTH_OK){

			status = __command(sabertooth, BAUD_RATE, baudrate);
			HAL_Delay(250);
		}
	}

	return status;

}

SABERTOOTH_StatusTypeDef set_ramping(sabertooth_t* sabertooth, RAMPING_VALUE ramping){

	SABERTOOTH_StatusTypeDef status = SABERTOOTH_ERR;

	if(sabertooth){

		status = assert_parameter(IS_VALID_RAMPING_VALUE(ramping)); //Ho dovuto correggere, suppongo sia così, prima c'era "value" come argomento... (alexios_)

		if(status == SABERTOOTH_OK){

			status = __command(sabertooth, RAMPING, ramping); // Stesso di sopra anche qui.
		}
	}

	return status;

}

SABERTOOTH_StatusTypeDef set_deadband(sabertooth_t* sabertooth, uint8_t value){

	SABERTOOTH_StatusTypeDef status = SABERTOOTH_ERR;

	if(sabertooth){

		status = assert_parameter(IS_VALID_DEADBAND_VALUE(value));

		if(status == SABERTOOTH_OK){

			status = __command(sabertooth, DEADBAND, value);
		}
	}

	return status;

}

/**************************************************************************/

