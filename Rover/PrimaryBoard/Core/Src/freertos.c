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

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim.h"
#include "adc.h"
#include "crc.h"
/* Include Dependencies */
#include "geometry.h"
#include "sabertooth_2x12.h"
#include "ps2x.h"
#include "smoothed_ps2x_analog.h"
#include "mpu60x0.h"
#include "hcsr04.h"
#include "state_manager.h"

#include "b2b_common.h"
#include "b2b_protocol.h"
#include "delayus.h"

#include "math.h"

#include "liquid_crystal_I2C.h"

#include <stdlib.h>
#include <stdio.h>


/**
 * @file scheduler.h
 * @brief Replaces the FreeRTOS scheduler with the custom scheduler "ESFree".
 *
 * This file includes the necessary definitions to implement the "ESFree" scheduler,
 * designed to support the Timeline Scheduling policy in FreeRTOS environments.
 * It includes functions for initializing and managing the custom scheduler.
 * 
 * For the use of the Timeline Scheduling policy, the macro schedSCHEDULING_POLICY_MANUAL has been configured.
 * 
 * Furthermore, for the correct use of the following policy, in the FreeRTOSConfig.h configuration file,
 * time slicing has been disabled by setting the macro configUSE_TIME_SLICING to 0, as it is normally active.
 */
#include "scheduler.h"

/**
 * @file periodic_tasks_constants.h
 * @brief Defines constants for periodic tasks.
 *
 * This header file contains the definitions of constants necessary to configure
 * periodic tasks. It includes definitions for the phase, period, worst-case execution time (WCET),
 * and deadline of each task.
 */
#include "periodic_tasks_constants.h"



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MAX_VELOCITY_NORMAL				(POWER_MAX)							/**< Defines the max velocity of the rover with the normal behaviour. */
#define MAX_VELOCITY_DEGRADED			((int8_t)POWER_MAX / 2)				/**< Defines the max velocity of the rover with the degraded behaviour. */
#define USE_B2B_PROTOCOL													/**< Enables the use of the B2B protocol. */
#define USE_PS2X               							  					/**< Enables the use of the PS2X controller. */
#define USE_SABERTOOTHS 	  							  					/**< Enables the use of the Sabertooth motor drivers. */
#define USE_MPU6050 		  							  					/**< Enables the use of the MPU6050 accelerometer. */
#define USE_HCSR04 			  							  					/**< Enables the use of the HCSR04 ultrasonic sensors. */
//#define USE_LCD 			  							  					/**< Enables the use of the LCD display. */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/********************** B2B protocol buffer *****************/
/**
 * @brief Buffer used for the B2B protocol. This buffer is used to store the data exchanged between the two boards.
 * this buffer is used for all the data exchanged between the two boards.
*/
static uint8_t buffer[SUBSTATE_B1_B2_BYTE_SIZE + CRC_SIZE];

/********************** B2B configuration parameter *****************/
static const b2b_config_t b2b_config = {
		  .ack_in_Pin = ACK_IN_Pin,
		  .ack_in_Port = ACK_IN_GPIO_Port,
		  .ack_out_Pin = ACK_OUT_Pin,
		  .ack_out_Port = ACK_OUT_GPIO_Port,
		  .rtr_in_Pin = RTR_IN_Pin,
		  .rtr_in_Port = RTR_IN_GPIO_Port,
		  .rtr_out_Pin = RTR_OUT_Pin,
		  .rtr_out_Port = RTR_OUT_GPIO_Port,
		  .uartHandle = &huart6,
		  .crcHandle  = &hcrc,
		  .timer = &htim11
};

/*******************************************************************/

/******************* PS2X configuration parameter *******************/
static PS2X ps2x;
static const PS2X_config_t ps2x_config = {
		  .ATT_Port = ATT_GPIO_Port,
		  .ATT_GPIO_Pin = ATT_Pin,
		  .hspi = &hspi1
};
/********************************************************************/

/***************** MPU60X0 configuration parameter *****************/
static MPU60X0_t mpu6050;
static const MPU60X0_config_t mpu6050_config = {
		  .hi2c = &hi2c3
};
/*******************************************************************/

/****************** HCSR04 configuration parameter *****************/
static ultrasounds_t ultrasounds;
static const HCSR04_config_t HCSR04_left_config = {
		  .Trig_Port = TRIG_1_GPIO_Port,
		  .Trig_Pin = TRIG_1_Pin,
		  .timer = &htim5,
		  .channel = TIM_CHANNEL_2
};

static const HCSR04_config_t HCSR04_center_config = {
  		  .Trig_Port = TRIG_2_GPIO_Port,
  		  .Trig_Pin = TRIG_2_Pin,
  		  .timer = &htim2,
  		  .channel = TIM_CHANNEL_1
};

static const HCSR04_config_t HCSR04_right_config = {
  		  .Trig_Port = TRIG_3_GPIO_Port,
  		  .Trig_Pin = TRIG_3_Pin,
  		  .timer = &htim3,
  		  .channel = TIM_CHANNEL_2
};
/*******************************************************************/
/**************** Sabertooth configuration parameter ***************/
static sabertooth_t sabertooth1, sabertooth2;
static const sabertooth_config_t sabertooth_1_config = {
		  .address = 128,
		  .uartHandle = &huart1
};
static const sabertooth_config_t sabertooth_2_config = {
  		  .address = 130,
  		  .uartHandle = &huart1
};
/*******************************************************************/
/************* Liquid Crystal I2C configuration parameter **********/
lcd_i2c_t lcd;
lcd_i2c_config_t lcd_config = {
		._addr = LCD_ADDRESS,
		._cols = 16,
		._i2cHandle = &hi2c3,
		._rows = 2
};


static uint8_t timer_elapsed_first = 0;	/**< Flag to check if the first timer has elapsed, only for internal purpose. */

/** 
 * @brief Handle for the sensor reading task.
 *
 * This variable is used to maintain the reference to the task responsible
 * for reading the sensors.
 */
TaskHandle_t read_sensors_task_handle           = NULL;

/** 
 * @brief Handle for the decision task.
 *
 * This variable is the handle for the task responsible for deciding the next actions
 * based on the exchanged and non-exchanged information among the boards.
 */
TaskHandle_t decision_task_handle          		= NULL;

/** 
 * @brief Handle for the actuation task.
 *
 * Handle for the task responsible for actuation operations. In particular, it is responsible 
 * for managing the actuation and control of the motors.
 */
TaskHandle_t actuation_task_handle              = NULL;

#ifdef USE_LCD
static char buff[30];
#endif

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  Task used to read all the sensors managed by the primary board
 * 
 * @param pvParameters  parameters passed to the task
 */
static void read_sensors_task(void *pvParameters);

/**
 * @brief Task used to manage the communication between the two boards, this task
 * only starts the comunication, all the protocol will be executed in background
 * useing the interrupt
 * 
 * @param pvParameters parameters passed to the task
 */
static void decision_task(void *pvParameters);

/**
 * @brief Task used to manage the actuation of the rover
 * 
 * @param pvParameters parameters passed to the task
 */
static void actuation_task(void *pvParameters);

/**
 * @brief Task used to manage the actuation of the rover
 * 
 * @param pvParameters parameters passed to the task
 */
static void single_board_task(void *pvParameters);

/********************** Control Buttons Managing *****************/
/**
 * @brief Clears impulsive action flags from the control buttons state.
 *
 * This function removes the flags related to actions that are intended to be impulsive or momentary,
 * such all flags that do not concern the backward alternative, the emergency stop, ensuring that these actions do not persist
 * beyond their intended moment of activation.
 *
 * @param control_buttons The current state of the control buttons.
 * @return The new state of the control buttons with impulsive action flags cleared.
 */
static inline uint8_t __clear_impulsive_flags(uint8_t control_buttons);

/**
 * @brief Clears the emergency stop flag from the control buttons state.
 *
 * This function specifically targets and clears the emergency stop flag, allowing for the
 * reset of the emergency stop state without affecting other control states.
 *
 * @param control_buttons The current state of the control buttons.
 * @return The new state of the control buttons with the emergency stop flag cleared.
 */
static inline uint8_t __clear_emegency_stop_flags(uint8_t control_buttons);

/**
 * @brief Determines if the alternate backward command combo is activated.
 *
 * Checks if a specific combination of buttons (e.g., SQUARE and DOWN) is pressed to activate
 * an alternate backward command. This could be used to trigger special vehicle movements or actions.
 *
 * @param ps2x Pointer to the PS2X controller object.
 * @return A mask indicating whether the alternate backward command combo is active.
 */
static inline uint8_t __get_alt_backward_combo_mask(PS2X *ps2x);

/**
 * @brief Determines if the backward command is activated based on the rover's state.
 *
 * Checks if the conditions for activating a backward command are met, considering the current
 * state of the rover and the button press state. This can include specific button combinations
 * and rover states such as being idle.
 *
 * @param ps2x Pointer to the PS2X controller object.
 * @param rover_state The current state of the rover.
 * @param control_buttons The current state of the control buttons.
 * @return A mask indicating whether the backward command is active.
 */
static inline uint8_t __get_backward_cmd_mask(PS2X *ps2x, ROVER_STATE rover_state, uint8_t control_buttons);

/**
 * @brief Clears the flags related to light control from the control buttons state.
 *
 * This function targets and clears the flags controlling the vehicle's lights, allowing for
 * the reset of the lights' state without affecting other control states.
 *
 * @param control_buttons Pointer to the current state of the control buttons.
 */
static inline void __clear_lights_flag(uint8_t *control_buttons);

/**
 * @brief Clears the emergency stop flag from the control buttons state by reference.
 *
 * @param control_buttons Pointer to the current state of the control buttons.
 */
static inline void __clear_emergency_stop_flag(uint8_t *control_buttons);

/**
 * @brief Calculates the control buttons mask for light controls based on PS2X input.
 *
 * Determines the state of the vehicle's lights (left, right, both) based on the current
 * state of the PS2X controller inputs. This includes toggling lights on or off based on specific
 * button presses.
 *
 * @param ps2x Pointer to the PS2X controller object.
 * @param control_buttons The current state of the control buttons, used to determine the initial state of the lights.
 * @return A mask indicating the new state of the lights control buttons.
 */
static inline uint8_t __get_lights_control_buttons_mask(PS2X *ps2x, uint8_t control_buttons);

/**
 * @brief Determines if the buzzer should be activated based on PS2X input.
 *
 * Checks if the button for activating the buzzer (e.g., L1) is pressed, allowing for auditory
 * feedback or alerts based on controller input.
 *
 * @param ps2x Pointer to the PS2X controller object.
 * @return A mask indicating whether the buzzer is activated.
 */
static inline uint8_t __get_buzzer_mask(PS2X *ps2x);

/**
 * @brief Determines the emergency stop mask based on PS2X input and current control buttons state.
 *
 * This function calculates whether an emergency stop action should be toggled based on the current
 * button press state and the previous state of the emergency stop control.
 *
 * @param ps2x Pointer to the PS2X controller object.
 * @param control_buttons The current state of the control buttons, used to determine the initial state of the emergency stop.
 * @return The new state of the emergency stop control button.
 */
static inline uint8_t __get_emergency_stop_mask(PS2X *ps2x, uint8_t control_buttons);

/**
 * @brief Updates the control buttons state based on PS2X input and rover state.
 *
 * This function consolidates the various control button state updates, including clearing impulsive flags,
 * updating light control states, and setting emergency stop, buzzer states, the alternative backward states, based on the current
 * inputs from the PS2X controller and the rover's state.
 *
 * @param ps2x Pointer to the PS2X controller object.
 * @param rover_state The current state of the rover.
 * @param control_buttons The current state of the control buttons.
 * @return The updated state of the control buttons incorporating all changes based on input and rover state.
 */
static inline uint8_t __update_control_buttons(PS2X *ps2x, ROVER_STATE rover_state, uint8_t control_buttons);

/**
 * @brief Read the gamepad associated to the specified ps2x structure.
 * 
 * @param ps2x structure associated to the gamepad
 */
static void __read_gamepad(PS2X *ps2x);

/**
 * @brief Read the MPU6050 sensor associated to the specified mpu6050 structure.
 * 
 * @param mpu6050 structure associated to the MPU6050 sensor
 */
static void __read_mpu6050(MPU60X0_t *mpu6050);

/**
 * @brief Read the temperature of the microcontroller.
 */
static void __read_temperature(void);

/**
 * @brief Calculates the maximum allowable velocity based on the current rover state.
 * 
 * @return int8_t maximum allowable velocity 
 */
static int8_t __get_max_velocity(void);

/**
 * @brief Drive the rover based on the current state and the received commands.
 * 
 * @param sabertooth1 pointer to the first sabertooth motor driver
 * @param sabertooth2 pointer to the second sabertooth motor driver
 * @param x_analog x-axis analog value
 * @param y_analog y-axis analog value
 * @param max_velocity maximum allowable velocity
 * @param max_increment_y maximum increment of the y-axis analog value, higher values mean faster acceleration
 * @param max_decrement_y maximum decrement of the y-axis analog value, higher values mean faster deceleration
 */
static void __drive(sabertooth_t *sabertooth1, sabertooth_t *sabertooth2, int8_t x_analog, int8_t y_analog, int8_t max_velocity,
					uint8_t max_increment_y, uint8_t max_decrement_y);

/* USER CODE END FunctionPrototypes */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

	#ifdef USE_B2B_PROTOCOL
	b2b_protocol_init(&b2b_config, buffer);
	#endif

	#ifdef USE_PS2X
	if(PS2X_init(&ps2x, &ps2x_config, 1, osWaitForever) != PS2X_OK){
		PS2X_StatusTypeDef ps2_status = PS2X_ERROR;
		do{
			ps2_status = ping_controller(&ps2x, 1, osWaitForever);
		} while(ps2_status != PS2X_OK);
	}
	#endif

	#ifdef USE_LCD
	if(liquidCrystal_I2C_init(&lcd, &lcd_config) != LCD_OK){
		Error_Handler();
	}
	#endif

	#ifdef USE_MPU6050
	if(MPU60X0_init(&mpu6050, &mpu6050_config, osWaitForever) != MPU60X0_OK){
		Error_Handler();
	}
	#endif


	#ifdef USE_HCSR04
	if(HCSR04_init(&ultrasounds, &HCSR04_left_config, &HCSR04_center_config, &HCSR04_right_config) != HCSR04_OK){
		Error_Handler();
	}
	#endif


	#ifdef USE_SABERTOOTHS
	if(sabertooth_init(&sabertooth1, &sabertooth_1_config) != SABERTOOTH_OK ||
       sabertooth_init(&sabertooth2, &sabertooth_2_config) != SABERTOOTH_OK){
	  Error_Handler();
	}
	#endif

	if(HAL_ADC_Start(&hadc1) != HAL_OK){
		Error_Handler();
	}

  /* Inizializzazione dello scheduler*/
	vSchedulerInit();
  /* Creazione dei task periodici */

  /* Creazione del task di lettura dei sensori con i parametri definiti in periodic_tasks_constants.h */
  vSchedulerPeriodicTaskCreate(read_sensors_task, "read_Sensors_task", configMINIMAL_STACK_SIZE, NULL, 3, &read_sensors_task_handle, READ_SENSORS_TASK_PHASE, READ_SENSORS_TASK_PERIOD, READ_SENSORS_TASK_WCET, READ_SENSORS_TASK_DEADLINE);
  
  /* Creazione del task di decisione con i parametri definiti in periodic_tasks_constants.h */
  vSchedulerPeriodicTaskCreate(decision_task, "decision_task", configMINIMAL_STACK_SIZE, NULL, 2, &decision_task_handle, DECISION_TASK_PHASE, DECISION_TASK_PERIOD, DECISION_TASK_WCET, DECISION_TASK_DEADLINE);
  
  /* Creazione del task di attuazione con i parametri definiti in periodic_tasks_constants.h */
  vSchedulerPeriodicTaskCreate(actuation_task, "actuation_task", configMINIMAL_STACK_SIZE, NULL, 1, &actuation_task_handle, ACTUATION_TASK_PHASE, ACTUATION_TASK_PERIOD, ACTUATION_TASK_WCET, ACTUATION_TASK_DEADLINE);
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}


/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
 * @brief Activate the single board mode. In this mnode the communication with the secondary board is disabled.
 *        The rover will work in a degraded mode, using only the sensors and the actuators of the primary board.
 */
static inline void single_board_mode(){
	/*Reschedule the original decision task with the single board one*/
	rover_state_t *rover_state = rover_state_get_instance();
	rover_state->single_board_is_active = 1U;

	rescheduleTask(single_board_task, "single_board_task", NULL, &decision_task_handle, SINGLE_BOARD_TASK_WCET);
}


/**
 * @brief Task responsible for sensor reading
 * This task is responsible for reading the following sensors:
 *    - PS2 controller reading: estimated time 1.25 ms.
 *      For a detailed analysis, see chapter 4.1.1, PS2 Controller section.
 *    - HCSR04 ultrasonic sensors reading: estimated time 23.5 ms.
 *      For a detailed analysis, see chapter 4.1.1, HCSR04 section.
 *    - Accelerometer reading: estimated time 0.30 ms.
 *      For a detailed analysis, see chapter 4.1.1, MPU6050 section.
 *    - Internal microcontroller temperature reading: not considered.
 * An approximate time of 26 ms has been considered.
 * 
 * @param pvParameters Pointer to parameters passed to the task 
 * 
 */


static void read_sensors_task(void *pvParameters){

	#ifdef USE_PS2X
	__read_gamepad(&ps2x);
	#endif

	#ifdef USE_MPU6050
	__read_mpu6050(&mpu6050);
	#endif

	__read_temperature();

	#ifdef USE_HCSR04
	HCSR04_read(&ultrasounds); 
	rover_state_t *rover_state = rover_state_get_instance();
	rover_state->obstacle_distances[0] = ultrasounds.left.current_distance;
	rover_state->obstacle_distances[1] = ultrasounds.center.current_distance;
	rover_state->obstacle_distances[2] = ultrasounds.right.current_distance;
	#endif

	#ifdef USE_LCD
	clear(&lcd);
	sprintf(buff, "%.0f %.0f %.0f", ultrasounds.left.current_distance, ultrasounds.center.current_distance, ultrasounds.right.current_distance);
	print(&lcd, buff);
	#endif

}


/**
 * @brief Task responsible for executing the communication protocol with board_1
 * The communication protocol is discussed in chapter 4.1.2 "Information Exchange Between Boards".
 * The task consists of the following phases:
 *   - Reception of information read by the sensor reading task of board_1, estimated time 2.67 ms.
 *   - Transmission of information read by the sensor reading task of the current board, estimated time 2.29 ms.
 *   - Updating the global system view and calculation of intentions, estimated time 2 ms.
 *   - Reception of intentions from board_1, estimated time 0.66 ms.
 *   - Transmission of intentions to board_2, estimated time 0.66 ms.
 * 
 *  Not all this times are important fo the scheduling because the communication is managed by the interrupt.
 *  
 * @param pvParameters Pointer to parameters passed to the task 
 */
static void decision_task(void *pvParameters){

	// Ricevo lo stato dalla board2, safe receive gestisce la richiesta di ritrasmissione.
	#ifdef USE_B2B_PROTOCOL
		b2b_protocol_t* b2b = get_b2b_instance();
		if(b2b->protocol_status == 0){
			b2b->retransmissions_num = 0;
			b2b->protocol_status = 1;
			b2b_request_state();
		}else{
			single_board_mode();
		}
	#endif
}

#ifdef USE_LCD
    /**
     * @brief Get the string associated to the starus of the rover. Used only for debug purposes with the LCD mounted on the rover.
     * 
     * @param state the actual state of the rover
     * @return const char* string associated to the state
     */
	const char* get_state_string(ROVER_STATE state) {
	switch (state) {
		case IDLE:
		return "IDLE";
		case FORWARD:
		return "FORWARD";
		case ON_RIGHT:
		return "ON_RIGHT";
		case ON_LEFT:
		return "ON_LEFT";
		case FORWARD_RIGHT:
		return "FORWARD_RIGHT";
		case FORWARD_LEFT:
		return "FORWARD_LEFT";
		case BACKWARD:
		return "BACKWARD";
		case BACKWARD_RIGHT:
		return "BACKWARD_RIGHT";
		case BACKWARD_LEFT:
		return "BACKWARD_LEFT";
		case BRAKING:
		return "BRAKING";
		case EMERGENCY:
		return "EMERGENCY";
		case AVOID_RIGHT_OBST:
			return "AVOID_RIGHT_OBST";
		case AVOID_LEFT_OBST:
			return "AVOID_left_OBST";
		default:
		return "UNKNOWN_STATE";
	}
	}
#endif

/**
 * @brief Task responsible for actuating commands.
 * 
 * The actuation task is responsible for bringing the rover to the physical state corresponding to the intention 
 * jointly agreed upon by the two boards. The task independently manages the four rover motors by commanding them 
 * via UART, with each transmission estimated to last about 4.16 ms. For a detailed analysis, see chapter 4.1.3.
 * 
 * The execution time of the task has been estimated at 17 ms.
 * 
 * @param pvParameters Pointer to parameters passed to the task 
 */
static void actuation_task(void *pvParameters){

	static int8_t max_velocity = MAX_VELOCITY_NORMAL;
	rover_state_t *rover_state = rover_state_get_instance();

	#ifdef USE_LCD
		setCursor(&lcd, 0, 1);
		print(&lcd, get_state_string(rover_state->current_state));
	#endif

	if((rover_state->control_buttons & EMERGENCY_STOP_MASK) != 0){ // spostarlo nel task di decisione (?)
		HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 1);
		__drive(&sabertooth1, &sabertooth2, 0, 0, max_velocity, MAX_INCREMENT_STEP_Y, 127);
	}else{

		HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 0);

		max_velocity = __get_max_velocity();

		if(rover_state->single_board_is_active == 1U && rover_state->current_state == IDLE){
			HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 1);
		}else if(rover_state->current_state == BRAKING){
			if(rover_state->braking == SMOOTH){
				__drive(&sabertooth1, &sabertooth2, 0, 0, max_velocity, MAX_INCREMENT_STEP_Y, MAX_DECREMENT_STEP_Y/4 + 1);
			}else if(rover_state->braking == HARD){
				__drive(&sabertooth1, &sabertooth2, 0, 0, max_velocity, MAX_INCREMENT_STEP_Y, 127);
			}
		} else if(rover_state->current_state == ALTERNATIVE_BACKWARD) {
			__drive(&sabertooth1, &sabertooth2, 40, 0, max_velocity, MAX_INCREMENT_STEP_Y, MAX_DECREMENT_STEP_Y);

		} else if(rover_state->current_state == AVOID_LEFT_OBST){
			__drive(&sabertooth1, &sabertooth2, 40, 0, max_velocity, MAX_INCREMENT_STEP_Y, MAX_DECREMENT_STEP_Y);

		} else if(rover_state->current_state == AVOID_RIGHT_OBST){
			__drive(&sabertooth1, &sabertooth2, -40, 0, max_velocity, MAX_INCREMENT_STEP_Y, MAX_DECREMENT_STEP_Y);

		} else if(rover_state->current_state != IDLE){
			__drive(&sabertooth1, &sabertooth2, rover_state->x_analog, rover_state->y_analog, max_velocity, MAX_INCREMENT_STEP_Y, MAX_DECREMENT_STEP_Y); // verificare velocità massima spostando max_velocity nella funzione drive (?)
		}

	}

}

/**
 * @brief Task responsible for updating the state in case the other board fails.
 * 
 * The task is responsible for updating the global system view and calculating intentions autonomously 
 * if the other board fails. The information on which the task will make decisions, however, is incomplete.
 * Since this task is only responsible for information processing, the estimated time in the worst case 
 * is 2 ms. This decreases significantly compared to the case where the system is operating normally, 
 * as there is no need to exchange information.
 */
static void single_board_task(void *pvParameters){

	rover_state_t *rover_state = rover_state_get_instance();

	b2b_processing_state();
	b2b_update_global_state();
	rover_state->degraded = ACTIVE;

}

//-------------------- UTILITY FUNCTIONS ----------------------

static inline uint8_t __clear_impulsive_flags(uint8_t control_buttons){
	return (control_buttons) & (ALT_BACKWARD_COMBO_MASK|ALT_BACKWARD_CMD_MASK|BOTH_WHITE_LED_MASK|EMERGENCY_STOP_MASK);
}

static inline uint8_t __clear_emegency_stop_flags(uint8_t control_buttons){
	return (control_buttons) & (~EMERGENCY_STOP_MASK);
}

static inline uint8_t __get_alt_backward_combo_mask(PS2X *ps2x){
	uint8_t or_mask = 0x00U;
	if((check_button_digital(ps2x, PS2X_SQUARE)==1U) && (check_button_digital(ps2x, PS2X_DOWN)==1U)){
		or_mask = ALT_BACKWARD_COMBO_MASK;
	}
	return or_mask;
}

static inline uint8_t __get_backward_cmd_mask(PS2X *ps2x, ROVER_STATE rover_state, uint8_t control_buttons){
	uint8_t or_mask = 0x00U;
	uint8_t alt_backward_combo_mask = (control_buttons & ALT_BACKWARD_COMBO_MASK) != 0U;
	or_mask = control_buttons & (ALT_BACKWARD_CMD_MASK);
	if((alt_backward_combo_mask == 1U) && (check_button_digital(ps2x, PS2X_CIRCLE)== 1U) && (rover_state == IDLE)){
		or_mask |= ALT_BACKWARD_CMD_MASK;
	}
	return or_mask;
}

static inline void __clear_lights_flag(uint8_t *control_buttons){
	*control_buttons = (*control_buttons) & (~(BOTH_WHITE_LED_MASK|L_WHITE_LED_MASK|R_WHITE_LED_MASK));
}

static inline void __clear_emergency_stop_flag(uint8_t *control_buttons){
	*control_buttons = (*control_buttons) & (~EMERGENCY_STOP_MASK);
}

static inline uint8_t __get_lights_control_buttons_mask(PS2X *ps2x, uint8_t control_buttons){
	uint8_t or_mask = 0x00U;

	static uint8_t all_lights_prev_state_button = 0U; // stato fisico precedente

	uint8_t all_lights_actual_state_button = check_button_digital(ps2x, PS2X_R1);
	uint8_t all_lights_actual_state = control_buttons & BOTH_WHITE_LED_MASK;

	if((all_lights_actual_state_button == 1U) && (all_lights_actual_state_button !=all_lights_prev_state_button)){
		all_lights_actual_state ^= BOTH_WHITE_LED_MASK;
	}
	all_lights_prev_state_button = all_lights_actual_state_button;

	or_mask |= all_lights_actual_state;

	if(check_button_digital(ps2x, PS2X_L2)==1U){
		or_mask |= L_WHITE_LED_MASK;
	}

	if(check_button_digital(ps2x, PS2X_R2)==1U){
		or_mask |= R_WHITE_LED_MASK;
	}

	return or_mask;
}

static inline uint8_t __get_buzzer_mask(PS2X *ps2x){
	uint8_t or_mask = 0x00U;
	if(check_button_digital(ps2x, PS2X_L1)==1U){
		or_mask|=BUZZER_MASK;
	}
	return or_mask;
}

static inline uint8_t __get_emergency_stop_mask(PS2X *ps2x, uint8_t control_buttons){
	static uint8_t emergency_stop_prev_state_button = 1U; // stato fisico precedente
	uint8_t emergency_stop_actual_state_button = check_button_digital(ps2x, PS2X_TRIANGLE);

	uint8_t emergency_stop_actual_state = control_buttons & EMERGENCY_STOP_MASK;

	if((emergency_stop_actual_state_button == 1U) && (emergency_stop_actual_state_button !=emergency_stop_prev_state_button)){
		emergency_stop_actual_state ^= EMERGENCY_STOP_MASK;
	}
	emergency_stop_prev_state_button = emergency_stop_actual_state_button;

	return emergency_stop_actual_state;



}

static inline uint8_t __update_control_buttons(PS2X *ps2x, ROVER_STATE rover_state, uint8_t control_buttons){
	uint8_t new_control_buttons = __clear_impulsive_flags(control_buttons);
	new_control_buttons |= __get_alt_backward_combo_mask(ps2x);
	new_control_buttons |= __get_backward_cmd_mask(ps2x, rover_state, new_control_buttons);
	uint8_t tmp_mask = __get_lights_control_buttons_mask(ps2x, new_control_buttons);
	__clear_lights_flag(&new_control_buttons);
	new_control_buttons |= tmp_mask;
	tmp_mask = __get_emergency_stop_mask(ps2x, new_control_buttons);
	__clear_emergency_stop_flag(&new_control_buttons);
	new_control_buttons |= tmp_mask;
	new_control_buttons |= __get_buzzer_mask(ps2x);

	return new_control_buttons;
}


static void __read_gamepad(PS2X *ps2x){

	rover_state_t *rover_state = rover_state_get_instance();

	if(read_gamepad_analog(ps2x, osWaitForever) != PS2X_OK){
		rover_state->control_value |= (1U << 1);
	}

	Cartesian2D y_analog, x_analog;
	get_analog_value(ps2x, LEFT_ANALOG, &y_analog);
	get_analog_value(ps2x, RIGHT_ANALOG, &x_analog);

	rover_state->x_analog = x_analog.x;
	rover_state->y_analog = y_analog.y;
	rover_state->control_buttons = __update_control_buttons(ps2x, rover_state->current_state, rover_state->control_buttons);


}

static void __read_mpu6050(MPU60X0_t *mpu6050){

	Cartesian3D mpu_data;
	static float current_time, previous_time;
	rover_state_t *rover_state = rover_state_get_instance();

	previous_time = current_time;
	current_time = osKernelGetTickCount();
	if(MPU60X0_get_gyro_value(mpu6050, &mpu_data) != MPU60X0_OK){
		rover_state->control_value |= 1;
	}
	if(fabs(mpu_data.z) > 1) // aggiungere una define per questo valore
		rover_state->gyroZ += mpu_data.z * ((current_time - previous_time) / 1000);

	if(MPU60X0_get_accel_value(mpu6050, &mpu_data) != MPU60X0_OK){
		rover_state->control_value |= 1;
	}

	rover_state->accX = -mpu_data.x;

}

static void __read_temperature(void){

	uint16_t raw_value;
	rover_state_t *rover_state = rover_state_get_instance();

	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	raw_value = HAL_ADC_GetValue(&hadc1);

	rover_state->temperature_B1 = ((float)raw_value) / 4095 * 3300;
	rover_state->temperature_B1 = ((rover_state->temperature_B1 - 760.0) / 2.5) + 25;

}

static int8_t __get_max_velocity(void){

	int8_t max_velocity;
	rover_state_t *rover_state = rover_state_get_instance();

	switch(rover_state->degraded){
		case ACTIVE:
			max_velocity = MAX_VELOCITY_DEGRADED;
			break;
		case INACTIVE:
			max_velocity = MAX_VELOCITY_NORMAL;
			break;
	}

	return max_velocity;

}

static void __drive(sabertooth_t *sabertooth1, sabertooth_t *sabertooth2, int8_t x_analog, int8_t y_analog, int8_t max_velocity,
					uint8_t max_increment_y, uint8_t max_decrement_y){

	static int8_t current_l_wheel = 0, next_l_wheel = 0,
				  current_r_wheel = 0, next_r_wheel = 0;

	if(smoothed_4_motors_drive((Cartesian2D){x_analog, y_analog}, current_r_wheel, current_l_wheel, (float)max_velocity, (float)max_velocity,
								max_increment_y, max_decrement_y, &next_r_wheel, &next_l_wheel) != SA_OK){
		Error_Handler();
	}

	current_l_wheel = next_l_wheel;
	current_r_wheel = next_r_wheel;

	if(current_l_wheel >= 0){
		drive_forward_motor(sabertooth1, MOTOR_1, (uint8_t)(current_l_wheel));
		drive_forward_motor(sabertooth2, MOTOR_1, (uint8_t)(current_l_wheel));
	} else {
		drive_backward_motor(sabertooth1, MOTOR_1, (uint8_t) abs(current_l_wheel));
		drive_backward_motor(sabertooth2, MOTOR_1, (uint8_t) abs(current_l_wheel));
	}

	if(current_r_wheel >= 0){
		drive_forward_motor(sabertooth1, MOTOR_2, (uint8_t)(current_r_wheel));
		drive_forward_motor(sabertooth2, MOTOR_2, (uint8_t)(current_r_wheel));

	} else {
		drive_backward_motor(sabertooth1, MOTOR_2, (uint8_t) abs(current_r_wheel));
		drive_backward_motor(sabertooth2, MOTOR_2, (uint8_t) abs(current_r_wheel));
	}

}



/************ CALLBAKS USED BY THE B2B PROTOCOL ************/


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM10) {
	  HAL_IncTick();
  } else if(htim->Instance == TIM11) {

	  if(timer_elapsed_first != 0){
		  b2b_protocol_t* b2b = get_b2b_instance();
		  stop_timer(b2b->config->timer);
		  timer_elapsed_first = 0;
		  if(b2b->retransmissions_num == 0){
			  b2b->retransmissions_num = 1;
			  if(b2b->protocol_step == 1){
				  b2b_send_state();
			  }else{
				  b2b_send_intention();
			  }
			  start_timer(b2b->config->timer, 1);
		  }else{
			  single_board_mode();
		  }
	  } else {

		  timer_elapsed_first = 1;

	  }

  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	b2b_protocol_t* b2b= get_b2b_instance();

	if (huart == b2b->config->uartHandle){

		if(b2b->protocol_step == 0){
			if(deserialize_status(b2b->buffer) == B2B_OK){
				b2b_send_ACK();
				b2b->protocol_step = 1;
			}
		} else if (b2b->protocol_step == 1){
			if (deserialize_intention(b2b->buffer) == B2B_OK){
				b2b_send_ACK();
				if(b2b_check_intention() != B2B_OK){
					Error_Handler();
				}
				b2b->protocol_step = 0;
			}
		}

	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	b2b_protocol_t* b2b = get_b2b_instance();

	if (b2b->config->rtr_in_Pin == GPIO_Pin){
		if(b2b->protocol_step == 1)
			b2b_send_state();
		else
			b2b_send_intention();

		start_timer(b2b->config->timer, 1); 
	}

	if (b2b->config->ack_in_Pin == GPIO_Pin){
		stop_timer(b2b->config->timer);
		timer_elapsed_first = 0;
		if(b2b->protocol_step == 1){
			b2b_processing_state();
			b2b_request_intention(b2b->buffer);
		} else {
			b2b_update_global_state();
			b2b->protocol_status = 0;
		}

	}

}


/* USER CODE END Application */

