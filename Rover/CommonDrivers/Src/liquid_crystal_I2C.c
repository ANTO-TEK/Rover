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

#include "liquid_crystal_I2C.h"

#define I2C_TIMEOUT			(1000)

// When the display powers up, it is configured as follows:
//
// 1. Display clear
// 2. Function set:
//    DL = 1; 8-bit interface data
//    N = 0; 1-line display
//    F = 0; 5x8 dot character font
// 3. Display on/off control:
//    D = 0; Display off
//    C = 0; Cursor off
//    B = 0; Blinking off
// 4. Entry mode set:
//    I/D = 1; Increment by 1
//    S = 0; No shift

static LCD_StatusTypeDef __lcd_init(lcd_i2c_t* lcd_i2c);
static LCD_StatusTypeDef expanderWrite(lcd_i2c_t* lcd_i2c, uint8_t data);
static LCD_StatusTypeDef write4bits(lcd_i2c_t* lcd_i2c, uint8_t value);
static LCD_StatusTypeDef pulseEnable(lcd_i2c_t* lcd_i2c, uint8_t data);
static LCD_StatusTypeDef __send(lcd_i2c_t* lcd_i2c, uint8_t value, uint8_t mode);

LCD_StatusTypeDef liquidCrystal_I2C_init(lcd_i2c_t* lcd_i2c, const lcd_i2c_config_t* lcd_config){
	LCD_StatusTypeDef status = LCD_ERROR;
	if((lcd_i2c != NULL) && (lcd_config != NULL)){
		lcd_i2c->config = lcd_config;
		lcd_i2c->_charsize = LCD_5x8DOTS;
		lcd_i2c->_backlightval = LCD_BACKLIGHT;
		status = __lcd_init(lcd_i2c);
	}
	return status;
}

inline LCD_StatusTypeDef command(lcd_i2c_t* lcd_i2c, uint8_t value) {
    LCD_StatusTypeDef status = LCD_ERROR;
    if (lcd_i2c != NULL) {
        status = __send(lcd_i2c, value, 0);
    }
    return status;
}


inline LCD_StatusTypeDef data(lcd_i2c_t* lcd_i2c, char data) {
    LCD_StatusTypeDef status = LCD_ERROR;
    if (lcd_i2c != NULL) {
        status = __send(lcd_i2c, data, 1);
    }
    return status;
}

inline size_t write(lcd_i2c_t* lcd_i2c, uint8_t value) {
	__send(lcd_i2c, value, LCD_RS);
	return 1;
}

static LCD_StatusTypeDef __lcd_init(lcd_i2c_t* lcd_i2c){
	LCD_StatusTypeDef status = LCD_ERROR;
	lcd_i2c->_displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;

	if (lcd_i2c->config->_rows > 1U) {
		lcd_i2c->_displayfunction |= LCD_2LINE;
	}

	// for some 1 line displays you can select a 10 pixel high font
	if ((lcd_i2c->_charsize != 0U) && (lcd_i2c->config->_rows == 1U)) {
		lcd_i2c->_displayfunction |= LCD_5x10DOTS;
	}

	HAL_Delay(50);

	status = expanderWrite(lcd_i2c, lcd_i2c->_backlightval);
	if (status == LCD_OK){
		HAL_Delay(1000);
		status = write4bits(lcd_i2c, 0x03 << 4);
		if (status == LCD_OK){
			HAL_Delay(5);
			//delayMicroseconds(4500); // wait min 4.1ms
			status = write4bits(lcd_i2c, 0x03 << 4);
			if( status == LCD_OK){
				HAL_Delay(5);
				//delayMicroseconds(4500); // wait min 4.1ms
				status = write4bits(lcd_i2c, 0x03 << 4);
				if(status == LCD_OK){
					HAL_Delay(1);
					//delayMicroseconds(150);
					if((write4bits(lcd_i2c, 0x02 << 4) == LCD_OK) &&
						(command(lcd_i2c, LCD_FUNCTIONSET | lcd_i2c->_displayfunction) == LCD_OK)){
						lcd_i2c->_displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
						if(display(lcd_i2c) == LCD_OK && clear(lcd_i2c) == LCD_OK){
							lcd_i2c->_displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
							if ((command(lcd_i2c, LCD_ENTRYMODESET | lcd_i2c->_displaymode) == LCD_OK) &&
								(home(lcd_i2c) == LCD_OK) ){
								status = LCD_OK;
							} else {
								status = LCD_ERROR;
							}
						}
					}
				}
			}
		}
	}
    return status;

}

LCD_StatusTypeDef clear(lcd_i2c_t* lcd_i2c) {
    LCD_StatusTypeDef status = LCD_ERROR;
    if (lcd_i2c != NULL) {
        status = command(lcd_i2c, LCD_CLEARDISPLAY);
        HAL_Delay(1);
    }
    return status;
}

LCD_StatusTypeDef home(lcd_i2c_t* lcd_i2c) {
    LCD_StatusTypeDef status = LCD_ERROR;
    if (lcd_i2c != NULL) {
        status = command(lcd_i2c, LCD_RETURNHOME);
        HAL_Delay(1);
    }
    return status;
}

LCD_StatusTypeDef setCursor(lcd_i2c_t* lcd_i2c, uint8_t col, uint8_t row) {
    LCD_StatusTypeDef status = LCD_ERROR;
    static const uint8_t row_offsets[4] = { 0x00, 0x40, 0x14, 0x54 };

    if(lcd_i2c != NULL){
		if (row > lcd_i2c->config->_rows){
			row = lcd_i2c->config->_rows - 1;
		}
		if ((command(lcd_i2c, LCD_SETDDRAMADDR | (col + row_offsets[row])) == LCD_OK)){
			status = LCD_OK;
		}
    }
    return status;
}


LCD_StatusTypeDef noDisplay(lcd_i2c_t* lcd_i2c) {
	LCD_StatusTypeDef status = LCD_ERROR;
	if(lcd_i2c != NULL){
		lcd_i2c->_displaycontrol &= ~LCD_DISPLAYON;
		status = command(lcd_i2c, LCD_DISPLAYCONTROL | lcd_i2c->_displaycontrol);
	}
	return status;
}
LCD_StatusTypeDef display(lcd_i2c_t* lcd_i2c) {
	LCD_StatusTypeDef status = LCD_ERROR;
	if(lcd_i2c != NULL){
		lcd_i2c->_displaycontrol |= LCD_DISPLAYON;
		status = command(lcd_i2c, LCD_DISPLAYCONTROL | lcd_i2c->_displaycontrol);
	}
	return status;
}

LCD_StatusTypeDef noCursor(lcd_i2c_t* lcd_i2c) {
	LCD_StatusTypeDef status = LCD_ERROR;
	if(lcd_i2c != NULL){
		lcd_i2c->_displaycontrol &= ~LCD_CURSORON;
		status = command(lcd_i2c, LCD_DISPLAYCONTROL | lcd_i2c->_displaycontrol);
	}
	return status;

}
LCD_StatusTypeDef cursor(lcd_i2c_t* lcd_i2c) {
	LCD_StatusTypeDef status = LCD_ERROR;
	if(lcd_i2c != NULL){
		lcd_i2c->_displaycontrol |= LCD_CURSORON;
		status = command(lcd_i2c, LCD_DISPLAYCONTROL | lcd_i2c->_displaycontrol);
	}
	return status;
}

LCD_StatusTypeDef noBlink(lcd_i2c_t* lcd_i2c) {
	LCD_StatusTypeDef status = LCD_ERROR;
	if(lcd_i2c != NULL){
		lcd_i2c->_displaycontrol &= ~LCD_BLINKON;
		status = command(lcd_i2c, LCD_DISPLAYCONTROL | lcd_i2c->_displaycontrol);
	}
	return status;
}
LCD_StatusTypeDef blink(lcd_i2c_t* lcd_i2c) {
	LCD_StatusTypeDef status = LCD_ERROR;
	if(lcd_i2c != NULL){
		lcd_i2c->_displaycontrol |= LCD_BLINKON;
		status = command(lcd_i2c, LCD_DISPLAYCONTROL | lcd_i2c->_displaycontrol);
	}
	return status;
}

LCD_StatusTypeDef scrollDisplayLeft(lcd_i2c_t* lcd_i2c) {
	LCD_StatusTypeDef status = LCD_ERROR;
	if(lcd_i2c != NULL){
		status = command(lcd_i2c, LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
	}
	return status;
}
LCD_StatusTypeDef scrollDisplayRight(lcd_i2c_t* lcd_i2c) {
	LCD_StatusTypeDef status = LCD_ERROR;
	if(lcd_i2c != NULL){
		status = command(lcd_i2c, LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
	}
	return status;
}

LCD_StatusTypeDef leftToRight(lcd_i2c_t* lcd_i2c) {
	LCD_StatusTypeDef status = LCD_ERROR;
	if(lcd_i2c != NULL){
		lcd_i2c->_displaymode |= LCD_ENTRYLEFT;
		status = command(lcd_i2c, LCD_ENTRYMODESET | lcd_i2c->_displaymode);
	}
	return status;
}

LCD_StatusTypeDef rightToLeft(lcd_i2c_t* lcd_i2c) {
	LCD_StatusTypeDef status = LCD_ERROR;
	if(lcd_i2c != NULL){
		lcd_i2c->_displaymode &= ~LCD_ENTRYLEFT;
		status = command(lcd_i2c, LCD_ENTRYMODESET | lcd_i2c->_displaymode);
	}
	return status;
}

LCD_StatusTypeDef autoscroll(lcd_i2c_t* lcd_i2c) {
	LCD_StatusTypeDef status = LCD_ERROR;
	if(lcd_i2c != NULL){
		lcd_i2c->_displaymode |= LCD_ENTRYSHIFTINCREMENT;
		status = command(lcd_i2c, LCD_ENTRYMODESET | lcd_i2c->_displaymode);
	}
	return status;

}

LCD_StatusTypeDef noAutoscroll(lcd_i2c_t* lcd_i2c) {
	LCD_StatusTypeDef status = LCD_ERROR;
	if(lcd_i2c != NULL){
		lcd_i2c->_displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
		status = command(lcd_i2c, LCD_ENTRYMODESET | lcd_i2c->_displaymode);
	}
	return status;

}

LCD_StatusTypeDef createChar(lcd_i2c_t* lcd_i2c, uint8_t location, uint8_t charmap[]) {
	LCD_StatusTypeDef status = LCD_OK;
	if(lcd_i2c != NULL){
		location &= 0x7; // we only have 8 locations 0-7
		command(lcd_i2c, LCD_SETCGRAMADDR | (location << 3));
		for (int i = 0; (i < 8) && (status == LCD_OK); i++) {
			status = write(lcd_i2c, charmap[i]);
		}
	} else {
		status = LCD_ERROR;
	}
	return status;
}

LCD_StatusTypeDef noBacklight(lcd_i2c_t* lcd_i2c) {
	LCD_StatusTypeDef status = LCD_ERROR;
	if(lcd_i2c != NULL){
		lcd_i2c->_backlightval = LCD_NOBACKLIGHT;
		status = expanderWrite(lcd_i2c, 0);
	}
	return status;
}

LCD_StatusTypeDef backlight(lcd_i2c_t* lcd_i2c) {
	LCD_StatusTypeDef status = LCD_ERROR;
	if(lcd_i2c != NULL){
		lcd_i2c->_backlightval = LCD_BACKLIGHT;
		status = expanderWrite(lcd_i2c, 0);
	}
	return status;
}

uint8_t getBacklight(lcd_i2c_t* lcd_i2c) {
  return lcd_i2c->_backlightval == LCD_BACKLIGHT;
}

LCD_StatusTypeDef print(lcd_i2c_t* lcd_i2c, const char *str){
	LCD_StatusTypeDef status = LCD_ERROR;
	if(lcd_i2c != NULL){
		const char *tmp = str;
		if(tmp != NULL){
			do {
				status = data(lcd_i2c, *tmp++);
			} while((*tmp != '\0') && (status == LCD_OK));
		} else {
			status = LCD_OK;
		}
	}
	return status;
}


static inline LCD_StatusTypeDef expanderWrite(lcd_i2c_t* lcd_i2c, uint8_t data){
	LCD_StatusTypeDef status = LCD_ERROR;
	uint8_t to_send = data | lcd_i2c->_backlightval;
	if(HAL_I2C_Master_Transmit(lcd_i2c->config->_i2cHandle, lcd_i2c->config->_addr, &to_send, 1, I2C_TIMEOUT) == HAL_OK){
		status = LCD_OK;
	}
	return status;
}

static LCD_StatusTypeDef write4bits(lcd_i2c_t* lcd_i2c, uint8_t value){
	LCD_StatusTypeDef status = LCD_ERROR;
	if(	(expanderWrite(lcd_i2c, value) == LCD_OK) &&
		(pulseEnable(lcd_i2c, value) == LCD_OK)){
		status = LCD_OK;
	}
	return status;

}

static LCD_StatusTypeDef pulseEnable(lcd_i2c_t* lcd_i2c, uint8_t data){
	LCD_StatusTypeDef status = LCD_ERROR;
	if((expanderWrite(lcd_i2c, data | LCD_EN) == LCD_OK)){
		if(expanderWrite(lcd_i2c, data & ~LCD_EN) == LCD_OK){
			status = LCD_OK;
		}
	}
	return status;
}

static LCD_StatusTypeDef __send(lcd_i2c_t* lcd_i2c, uint8_t value, uint8_t mode){
	LCD_StatusTypeDef status = LCD_ERROR;
	uint8_t highnib = (value)&(0xf0U);
	uint8_t lownib = (value<<4)&(0xf0U);
	if(	(write4bits(lcd_i2c, (highnib)|mode) == LCD_OK) &&
		(write4bits(lcd_i2c, (lownib)|mode) == LCD_OK)){
		status = LCD_OK;
	}
	return status;
}






