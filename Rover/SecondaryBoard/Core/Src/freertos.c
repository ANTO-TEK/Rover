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
#include "adc.h"
#include "encoder_manager.h"
#include "tim.h"

#include "timer_manager.h"

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
 * This header file contains the definitions of constants needed to configure
 * periodic tasks. It includes definitions for the phase, period, worst-case execution time (WCET),
 * and the deadline of each task.
 */
#include "periodic_tasks_constants.h"

#include "b2b_protocol.h"
#include "b2b_common.h"
#include "crc.h"
#include "usart.h"
#include "encoder_manager.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define USE_ENCODER_MANAGER
#define USE_B2B

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

static uint8_t retransmission_num = 0;
static uint8_t buffer[SUBSTATE_B1_B2_BYTE_SIZE + CRC_SIZE];

static TIM_HandleTypeDef *encoder_timers[ENCODER_NUM] = {&htim1,&htim2,&htim3,&htim5};

static encoder_manager_config_t encoder_manager_config = {
		.encoders_timer = encoder_timers
};

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
		  .uartHandle = &huart1,
		  .crcHandle  = &hcrc,
		  .timer = &htim9
};
/*******************************************************************/

static uint8_t timer_elapsed_first = 0;

/** 
 * @brief Handle for the sensor reading task.
 *
 * This variable is used to maintain the reference to the task responsible
 * for reading the sensors.
 */
TaskHandle_t read_sensors_task_handle = NULL;


/** 
 * @brief Handle for the actuation task.
 *
 * Handle for the task responsible for actuation operations. In particular, it is responsible
 * for managing the actuation and control of the motors.
 */
TaskHandle_t actuation_task_handle = NULL;


/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */
/** 
 * @brief Task for reading sensors.
 * 
 * This function is the task dedicated to reading sensors. It is executed periodically
 * to collect data from the sensors and maintain such information on board 1 through the temporary data structure
 * used later by the b2b protocol for updating the global state.
 *
 * @param pvParameters Parameters passed to the task at the time of its creation.
 * @return void
 */
static void read_sensors_task(void *pvParameters);


/**
 * @brief Task dedicated to executing commands
 *
 * The following task is responsible for executing commands related to the intention agreed upon between the two boards.
 * Specifically, since only the LEDs that manage the system's headlights are present on board 1, the only
 * actuation operations that will be performed are turning the LEDs on and off depending on the intention.
 *
 * Therefore, the estimated execution time for the actuation operations, being a write operation
 * on the pin, is approximated to 1 millisecond, as it is actually much lower but cannot be configured to a lower value
 * due to the temporal resolution of the scheduler.
 * 
 * @param pvParameters
 */
static void actuation_task(void *pvParameters);

/**
 * @brief Read the temperature of the microcontroller.
 */
static void __read_temperature(void);
/* USER CODE END FunctionPrototypes */

typedef b2b_status (*B2BFunctionPointer)(void);

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	b2b_protocol_init(&b2b_config, buffer);

	#ifdef USE_ENCODER_MANAGER
	if(encoder_manager_init(&encoder_manager_config) != ENCODER_MANAGER_OK){
		Error_Handler();
	}
	#endif


	if(HAL_ADC_Start(&hadc1)!= HAL_OK){
		Error_Handler();
	}

	/* Inizializzazione dello scheduler*/
	vSchedulerInit();


	/* Creazione dei task periodici */

	/* Creazione del task di lettura dei sensori con i parametri definiti in periodic_tasks_constants.h */
	vSchedulerPeriodicTaskCreate(read_sensors_task, "read_Sensors_task", configMINIMAL_STACK_SIZE, NULL, 2, &read_sensors_task_handle, READ_SENSORS_TASK_PHASE, READ_SENSORS_TASK_PERIOD, READ_SENSORS_TASK_WCET, READ_SENSORS_TASK_DEADLINE);
	
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

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

static void __lights_control(uint8_t control_buttons){
	uint8_t right_led_actual_state = (control_buttons & R_WHITE_LED_MASK)!=0U;
	uint8_t	left_led_actual_state= (control_buttons & L_WHITE_LED_MASK)!=0U;
	uint8_t all_lights_actual_state = (control_buttons & BOTH_WHITE_LED_MASK)!=0U;

	if(all_lights_actual_state == 1U){
		HAL_GPIO_WritePin(LED_1_WHITE_GPIO_Port, LED_1_WHITE_Pin, 1);
		HAL_GPIO_WritePin(LED_2_WHITE_GPIO_Port, LED_2_WHITE_Pin, 1);
	} else {
		HAL_GPIO_WritePin(LED_1_WHITE_GPIO_Port, LED_1_WHITE_Pin, left_led_actual_state);
		HAL_GPIO_WritePin(LED_2_WHITE_GPIO_Port, LED_2_WHITE_Pin, right_led_actual_state);
	}
}

static inline void __buzzer_control(uint8_t control_buttons){

	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, ((control_buttons & BUZZER_MASK) != 0U));

}

static void read_sensors_task(void *pvParameters){

	__disable_irq();

	rover_state_t *rover_state = rover_state_get_instance();
	__read_temperature();
	#ifdef USE_ENCODER_MANAGER
		encoder_manager_update_rpm(rover_state->rpm_motors);
	#endif

	__enable_irq();

}

static void actuation_task(void *pvParameters){

	rover_state_t *rover_state = rover_state_get_instance();

	__lights_control(rover_state->control_buttons);
	__buzzer_control(rover_state->control_buttons);


}

static void __read_temperature(void){

	uint16_t raw_value;
	rover_state_t *rover_state = rover_state_get_instance();

	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	raw_value = HAL_ADC_GetValue(&hadc1);

	rover_state->temperature_B2 = ((float)raw_value) / 4095 * 3300;
	rover_state->temperature_B2 = ((rover_state->temperature_B2 - 760.0) / 2.5) + 25;

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	b2b_protocol_t* b2b= get_b2b_instance();

	if (b2b->config->rtr_in_Pin == GPIO_Pin){
		if(b2b->protocol_step == 0){
			retransmission_num = 0;
			b2b_send_state();
		}else{
			b2b_send_intention();
		}

		start_timer(b2b->config->timer, 1);
	}

	if (b2b->config->ack_in_Pin == GPIO_Pin){
		stop_timer(b2b->config->timer);
		timer_elapsed_first = 0;
		if(b2b->protocol_step == 0){
			b2b_request_state();
		}else{
			b2b_request_intention(b2b->buffer);
		}
	}

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11) {
    HAL_IncTick();
  } else if(htim->Instance == TIM9){

	  if(timer_elapsed_first != 0){
		  b2b_protocol_t* b2b = get_b2b_instance();
		  stop_timer(b2b->config->timer);
		  timer_elapsed_first = 0;
		  if(retransmission_num == 0){
			  retransmission_num = 1;
			  if(b2b->protocol_step == 0){
				  b2b_send_state();
			  }else{
				  b2b_send_intention();
			  }
			  start_timer(b2b->config->timer, 1);
		  }else{
			  Error_Handler();
		  }
	  } else {
		  timer_elapsed_first = 1;
	  }

  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	b2b_protocol_t* b2b = get_b2b_instance();

	if(huart == b2b->config->uartHandle){

		if(b2b->protocol_step == 0){
			if(deserialize_status(b2b->buffer) == B2B_OK){
				b2b_send_ACK();
				b2b_processing_state();
				b2b->protocol_step = 1;
			}
		}else{
			if(deserialize_intention(b2b->buffer) == B2B_OK){
				b2b_send_ACK();
				b2b_update_global_state();
				b2b->protocol_step = 0;
			}
		}
	}
}
/* USER CODE END Application */

