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

/**
 * @file b2b_protocol.h
 * @author Adinolfi Teodoro, Amato Emilio, Bove Antonio, Guarini Alessio
 * @brief  This file contains the board to board functions that are specific for the primary board
 * @version 1.0
 * @date 2024-02-15
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef INC_B2B_PROTOCOL_H_
#define INC_B2B_PROTOCOL_H_

#include "b2b_common.h"
#include "timer_manager.h"


/**
 * @brief Requests the current state from the counterpart board.
 *
 * Initiates a non-blocking request to receive the current state from the counterpart board using the UART interface.
 * Upon successfully initiating the receive operation, it also sends a Ready-To-Receive (RTR) signal to the counterpart board
 * to indicate readiness to receive the state information. This function is typically used to asynchronously start the process
 * of updating the local state with the state of the counterpart board.
 *
 * The function internally uses HAL_UART_Receive_IT() to initiate the UART receive operation in interrupt mode, allowing
 * the communication to occur in the background while the system continues with other tasks. Upon successful initiation,
 * the function signals the counterpart board that it is ready to receive data by calling b2b_send_RTR().
 *
 * @note This function assumes that the UART handle and the buffer used for receiving data are properly configured and set up in the
 * b2b_protocol_t instance obtained by get_b2b_instance().
 *
 * @return b2b_status Returns B2B_OK if the request to receive state information is successfully initiated, B2B_ERR otherwise.
 */
b2b_status b2b_request_state();

/**
 * @brief Sends the current state to the counterpart board.
 * 
 * Packages the current state of the board into a predefined format, calculates its CRC for data integrity verification,
 * and sends it to the counterpart board over the communication interface.
 * 
 * @return b2b_status Returns B2B_OK if the state is successfully sent, B2B_ERR otherwise.
 */
b2b_status b2b_send_state();

/**
 * @brief Starting from the information that were exchanged during the state exchange phase, the primary board
 * elaborates the new state of the system, this state will be part of the "intention" a set of data that 
 * will be sent to the secondary board. The secondary board will do the same. Only if they reach the same conclusion
 * the status will be updated.
 */
void b2b_processing_state();

/**
 * @brief Updates the global state of the system based on the exchanged infromation.
 * Applies the changes computed by b2b_processing_state() to the global state of the system.
 */
void b2b_update_global_state();


/**
 * @brief Deserializes the received buffer into the local state structure and validates its integrity using CRC.
 * 
 * This function takes a buffer containing the received state information, deserializes it into the local state structure,
 * and checks the integrity of the data using CRC. If the CRC check passes, the local state is updated.
 * 
 * @param buffer The buffer containing the serialized state information received from the counterpart board.
 * @return b2b_status Returns B2B_OK if the buffer is successfully deserialized and the data integrity is confirmed, B2B_ERR otherwise.
 */
b2b_status deserialize_status(uint8_t* buffer);

#endif /* INC_B2B_PROTOCOL_H_ */
