/*  NV Mem access routines

   This code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#ifndef NV_ACC_H
#define NV_ACC_H

#include "nvs_flash.h"
#include "nvs.h"

/*#include "nvs_flash.h"
#include "nvs.h" */

/* Initialises the nvs                                                                               
 * returns 0 in case of error, 1 for success                                                         
 */  

esp_err_t init_nv(void);

/* Opens the nvs section for a certain handle 
 * returns 0 in case of error, 1 for success
 */

esp_err_t open_nv(nvs_handle *my_handle);

/* Read a uint8_t value from the nvs
 ** requires a valid handle, uint8_t pointer, key name for the nvs section  
 ** returns ESP_ERR in case of error, ESP_OK for success 
 **/

esp_err_t read_u8_nv(nvs_handle my_handle, uint8_t *val, const char *val_name);

/* Write a uint8_t to the nvs
 * requires a valid handle, uint8_t pointer, key name for the nvs section
 * returns ESP_ERR in case of error, ESP_OK for success
 */

esp_err_t write_u8_nv(nvs_handle my_handle, uint8_t val, const char *val_name); 

/* Read a uint8_t array from the nvs
 * requires a valid handle, uint8_t array pointer, array size, key name for the nvs section 
 * returns 0 in case of error, 1 for success
 */

esp_err_t read_blob_nv(nvs_handle my_handle, uint8_t *blob, uint8_t blob_size, const char *blob_name);

/* Write a uint8_t array to the nvs
 * requires a valid handle, uint8_t array pointer, array size, key name for the nvs section 
 * returns 0 in case of error, 1 for success
 */

esp_err_t write_blob_nv(nvs_handle my_handle, uint8_t *blob, uint8_t blob_size, const char *blob_name);

/* Closes the handle for the nvs page
 * requires a valid handle 
 * returns 1 for success
 */

esp_err_t close_nv(nvs_handle my_handle);

#endif
