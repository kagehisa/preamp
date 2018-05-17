/* 
   This code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#ifndef VOLUME_H
#define VOLUME_H

#define MAX_VOL		24		//TODO: sdkconfig
#define MIN_VOL		 1		//TODO: sdkconfig
#define MUTE_VOL	 0                  //TODO: sdkconfig




//get or set volume 
esp_err_t set_volume(uint8_t vol);
esp_err_t get_volume(uint8_t *vol);
esp_err_t get_fast_volume(uint8_t *vol);

//initialize module
void volume_init( void );

//write volume data to nvs
esp_err_t pers_volume( void );

#endif /* VOLUME_H */
