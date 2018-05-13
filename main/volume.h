/* 
   This code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#ifndef VOLUME_H
#define VOLUME_H


#include "driver/i2c.h"

//TODO: sdkconfig
#define I2C_SCL_IO          19               /*!< gpio number for I2C master clock */
//TODO: sdkconfig
#define I2C_SDA_IO          18               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM             I2C_NUM_1        /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ         100000           /*!< I2C master clock frequency */


//TODO: sdkconfig
#define U1_ADDR		                   0x4C             /*!< slave address for U1 */
#define U2_ADDR		                   0x4D             /*!< slave address for U2 */
#define U3_ADDR		                   0x4E             /*!< slave address for U3 */

#define MAX_VOL		24		//TODO: sdkconfig
#define MIN_VOL		 1		//TODO: sdkconfig
#define MUTE_VOL	 0                  //TODO: sdkconfig


#endif /* VOLUME_H */
