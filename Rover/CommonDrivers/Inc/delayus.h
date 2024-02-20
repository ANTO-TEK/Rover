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

#ifndef DELAYUS_H_
#define DELAYUS_H_

/**
 * @brief Generates a delay of a specified number of microseconds.
 *
 * This macro creates a delay loop that runs for the specified number of microseconds.
 * It is implemented in inline assembly and directly manipulates CPU registers to create
 * the delay. The macro assumes that the system clock is configured to provide a clock
 * frequency of 64 MHz to the CPU, as per the associated clock tree configuration.
 *
 * @note The actual delay is contingent upon the CPU's clock frequency being precisely
 *       64 MHz. If the clock frequency deviates from this value, the delay will not be accurate.
 *
 * @param us The number of microseconds to delay. The input parameter is scaled by a factor
 *           calculated based on the clock frequency (currently fixed at 64 MHz).
 *
 * Usage example:
 * @code
 *     delay_us(10); // Delays for 10 microseconds
 * @endcode
 */

#define delay_us(us) do { 							\
asm volatile (	"MOV R0,%[loops]\n 					\
			1: \n 									\
			SUB R0, #1\n 							\
			CMP R0, #0\n 							\
			BNE 1b \t" 								\
			: : [loops] "r" (13*us) : "memory" 		\
			); 										\
} while(0)

#endif
