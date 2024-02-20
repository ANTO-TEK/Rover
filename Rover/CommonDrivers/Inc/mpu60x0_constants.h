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

#ifndef INC_MPU60X0_CONSTANTS_H_
#define INC_MPU60X0_CONSTANTS_H_

typedef uint8_t MPU60X0_StatusTypeDef;
#define MPU60X0_OK											(0U)
#define MPU60X0_ERROR										(1U)
#define DEFAULT_SMPLRT_DIV									(0U)


/*
 * Command constants
 */

// 107, PWR_MGMT_1, RW

#define DEVICE_RESET         0b10000000  // 128U
#define SLEEP                0b01000000  // 64U
#define CYCLE                0b00100000  // 32U
#define TEMP_DIS             0b00001000  // 8U

//// EXT_SYNC_SET
#define EXT_SYNC_SET_DISABLED    0b00000000  // 0U
#define EXT_SYNC_SET_TEMP_OUT_L  0b00001000  // 1U
#define EXT_SYNC_SET_GYRO_XOUT_L 0b00010000  // 2U
#define EXT_SYNC_SET_GYRO_YOUT_L 0b00011000  // 3U
#define EXT_SYNC_SET_GYRO_ZOUT_L 0b00100000  // 4U
#define EXT_SYNC_SET_ACCEL_XOUT_L 0b00101000  // 5U
#define EXT_SYNC_SET_ACCEL_YOUT_L 0b00110000  // 6U
#define EXT_SYNC_SET_ACCEL_ZOUT_L 0b00111000  // 7U

/*
 * Registers constants
 */

#define MPU60X0_ADDRESS				  208U
#define SELF_TEST_X                   13U
#define SELF_TEST_Y                   14U
#define SELF_TEST_Z                   15U
#define SELF_TEST_A                   16U
#define SMPRT_DIV                     25U
#define CONFIG                        26U
#define GYRO_CONFIG                   27U
#define ACCEL_CONFIG                  28U
#define FIFO_EN                       35U
#define I2C_MST_CTRL                  36U
#define I2C_SLV0_ADDR                 37U
#define I2C_SLV0_REG                  38U
#define I2C_SLV0_CTRL                 39U
#define I2C_SLV1_ADDR                 40U
#define I2C_SLV1_REG                  41U
#define I2C_SLV1_CTRL                 42U
#define I2C_SLV2_ADDR                 43U
#define I2C_SLV2_REG                  44U
#define I2C_SLV2_CTRL                 45U
#define I2C_SLV3_ADDR                 46U
#define I2C_SLV3_REG                  47U
#define I2C_SLV3_CTRL                 48U
#define I2C_SLV4_ADDR                 49U
#define I2C_SLV4_REG                  50U
#define I2C_SLV4_DO                   51U
#define I2C_SLV4_CTRL                 52U
#define I2C_SLV4_DI                   53U
#define I2C_MST_STATUS                54U
#define INT_PIN_CFG                   55U
#define INT_ENABLE                    56U
#define INT_STATUS                    58U
#define ACCEL_XOUT_H                  59U
#define ACCEL_XOUT_L                  60U
#define ACCEL_YOUT_H                  61U
#define ACCEL_YOUT_L                  62U
#define ACCEL_ZOUT_H                  63U
#define ACCEL_ZOUT_L                  64U
#define TEMP_OUT_H                    65U
#define TEMP_OUT_L                    66U
#define GYRO_XOUT_H                   67U
#define GYRO_XOUT_L                   68U
#define GYRO_YOUT_H                   69U
#define GYRO_YOUT_L                   70U
#define GYRO_ZOUT_H                   71U
#define GYRO_ZOUT_L                   72U
#define EXT_SENS_DATA_00              73U
#define EXT_SENS_DATA_01              74U
#define EXT_SENS_DATA_02              75U
#define EXT_SENS_DATA_03              76U
#define EXT_SENS_DATA_04              77U
// Add more EXT_SENS_DATA
#define I2C_SLV0_DO                   99U
#define I2C_SLV1_DO                   100U
#define I2C_SLV2_DO                   101U
#define I2C_SLV3_DO                   102U
#define I2C_MST_DELAY_CTRL            103U
#define SIGNAL_PATH_RESET             104U
#define USER_CTRL                     106U
#define PWR_MGMT_1                    107U
#define PWR_MGMT_1_DEFAULT_STATUS     64U
#define PWR_MGMT_2                    108U
#define FIFO_COUNTH                   114U
#define FIFO_COUNTL                   115U
#define FIFO_R_W                      116U
#define WHO_AM_I                      117U

#endif /* INC_MPU60X0_CONSTANTS_H_ */
