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

#ifndef INC_LIQUIDCRYSTAL_I2C_H_
#define INC_LIQUIDCRYSTAL_I2C_H_

#include <stdint.h>

#include "i2c.h"

typedef uint8_t LCD_StatusTypeDef;

#define LCD_OK 							((LCD_StatusTypeDef) 0U)
#define LCD_ERROR 						((LCD_StatusTypeDef) 1U)

#define LCD_ADDRESS 					((uint8_t) 0x27<<1)

#define LCD_ERROR_MSG 					("Error in sending command\r\n")

// commands
#define LCD_CLEARDISPLAY 				(0x01U)
#define LCD_RETURNHOME 					(0x02U)
#define LCD_ENTRYMODESET 				(0x04U)
#define LCD_DISPLAYCONTROL 				(0x08U)
#define LCD_CURSORSHIFT 				(0x10U)
#define LCD_FUNCTIONSET 				(0x20U)
#define LCD_SETCGRAMADDR 				(0x40U)
#define LCD_SETDDRAMADDR 				(0x80U)

// flags for display entry mode
#define LCD_ENTRYRIGHT 					(0x00U)
#define LCD_ENTRYLEFT 					(0x02U)
#define LCD_ENTRYSHIFTINCREMENT 		(0x01U)
#define LCD_ENTRYSHIFTDECREMENT 		(0x00U)

// flags for display on/off control
#define LCD_DISPLAYON 					(0x04U)
#define LCD_DISPLAYOFF 					(0x00U)
#define LCD_CURSORON 					(0x02U)
#define LCD_CURSOROFF 					(0x00U)
#define LCD_BLINKON 					(0x01U)
#define LCD_BLINKOFF 					(0x00U)

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 				(0x08U)
#define LCD_CURSORMOVE 					(0x00U)
#define LCD_MOVERIGHT 					(0x04U)
#define LCD_MOVELEFT 					(0x00U)

// flags for function set
#define LCD_8BITMODE 					(0x10U)
#define LCD_4BITMODE 					(0x00U)
#define LCD_2LINE 						(0x08U)
#define LCD_1LINE 						(0x00U)
#define LCD_5x10DOTS 					(0x04U)
#define LCD_5x8DOTS 					(0x00U)

// flags for backlight control
#define LCD_BACKLIGHT 					(0x08U)
#define LCD_NOBACKLIGHT 				(0x00U)

#define LCD_EN	 						(0b00000100U)  // Enable bit
#define LCD_RW 							(0b00000010U)  // Read/Write bit
#define LCD_RS							(0b00000001U)  // Register select bit

typedef struct {

	uint8_t _addr;
	I2C_HandleTypeDef* _i2cHandle;
	uint8_t _cols;
	uint8_t _rows;

} lcd_i2c_config_t;


typedef struct{
	uint8_t _displayfunction;
	uint8_t _displaycontrol;
	uint8_t _displaymode;
	uint8_t _charsize;
	uint8_t _backlightval;
	lcd_i2c_config_t *config;


} lcd_i2c_t;

LCD_StatusTypeDef liquidCrystal_I2C_init(lcd_i2c_t* lcd_i2c, const lcd_i2c_config_t* lcd_config);
LCD_StatusTypeDef clear(lcd_i2c_t*);
LCD_StatusTypeDef home(lcd_i2c_t*);
LCD_StatusTypeDef noDisplay(lcd_i2c_t*);
LCD_StatusTypeDef display(lcd_i2c_t*);
LCD_StatusTypeDef noBlink(lcd_i2c_t*);
LCD_StatusTypeDef blink(lcd_i2c_t*);
LCD_StatusTypeDef noCursor(lcd_i2c_t*);
LCD_StatusTypeDef cursor(lcd_i2c_t*);

LCD_StatusTypeDef scrollDisplayLeft(lcd_i2c_t*);
LCD_StatusTypeDef scrollDisplayRight(lcd_i2c_t*);
LCD_StatusTypeDef printLeft(lcd_i2c_t*);
LCD_StatusTypeDef printRight(lcd_i2c_t*);
LCD_StatusTypeDef leftToRight(lcd_i2c_t*);
LCD_StatusTypeDef rightToLeft(lcd_i2c_t*);
LCD_StatusTypeDef shiftIncrement(lcd_i2c_t*);
LCD_StatusTypeDef shiftDecrement(lcd_i2c_t*);
LCD_StatusTypeDef noBacklight(lcd_i2c_t*);
LCD_StatusTypeDef backlight(lcd_i2c_t*);
uint8_t getBacklight(lcd_i2c_t*);
LCD_StatusTypeDef autoscroll(lcd_i2c_t*);
LCD_StatusTypeDef noAutoscroll(lcd_i2c_t*);

LCD_StatusTypeDef createChar(lcd_i2c_t*, uint8_t, uint8_t[]);
LCD_StatusTypeDef setCursor(lcd_i2c_t*, uint8_t, uint8_t);
size_t write(lcd_i2c_t*, uint8_t);
LCD_StatusTypeDef command(lcd_i2c_t*, uint8_t);
LCD_StatusTypeDef data(lcd_i2c_t*, char);
LCD_StatusTypeDef print(lcd_i2c_t*, const char*);

#endif /* INC_LIQUIDCRYSTAL_I2C_H_ */
