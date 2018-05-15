/*  Relais control driver

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#ifndef RELAIS_H
#define RELAIS_H

#include "sdkconfig.h"

/* Can run 'make menuconfig' to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define RELAIS_GPIO_1 CONFIG_RELAIS_GPIO_1
#define RELAIS_GPIO_2 CONFIG_RELAIS_GPIO_2
#define RELAIS_GPIO_3 CONFIG_RELAIS_GPIO_3
#define RELAIS_GPIO_4 CONFIG_RELAIS_GPIO_4
#define RELAIS_GPIO_5 CONFIG_RELAIS_GPIO_5

#define STATE_OFF 0
#define STATE_ON  1

#define OUTPUT_OFF 255
#define REL_ERR 128
#define RELAIS_NUM 5

typedef struct relais_state
{
   uint8_t relais[RELAIS_NUM];
   uint8_t state[RELAIS_NUM];
}relais_state;

/* Init function, that sets the last stored 
 * nv relay sate active or initializes the 
 * state struct if no values available  */
esp_err_t init_relais( void );

/* activates the gpio output responsible for the desired relais */
esp_err_t switch_relais_on(uint8_t relais_num);

/* deactivates the gpio output responsible for the desired relais  */
esp_err_t switch_relais_off(uint8_t relais_num);

/* get the currently active relais number */
uint8_t get_active_relais( void );

#endif /* RELAIS_H */
