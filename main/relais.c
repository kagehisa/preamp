/*  Relais control driver

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "driver/gpio.h"
#include "msg_stuff.h"
#include "nv_acc.h"
#include "relais.h"


/* Structure that represents the state of the relais and the corresponding GPIO pins  */
relais_state relais = { 
	{ RELAIS_GPIO_1, RELAIS_GPIO_2, RELAIS_GPIO_3, RELAIS_GPIO_4, RELAIS_GPIO_5 },
	{ STATE_OFF, STATE_OFF, STATE_OFF, STATE_OFF, STATE_OFF } 
      };

/* handle for the nvm access  */
nvs_handle relais_handle;

/* key string for the nvm values  */
const char* rel_key = "relais_state";


/* Module helper functions  */

static uint8_t init_relais_state( void )
{
   uint8_t ret = 0;

   ret = init_nv();
   if(ret)
   {
    ret = open_nv(&relais_handle);

    if(ret)
    { 
	ret = read_blob_nv(relais_handle, (&relais)->state, RELAIS_NUM, rel_key);
        if(!ret) //assuming the entry does noty exist yet
        {
          ret = write_blob_nv(relais_handle, (&relais)->state, RELAIS_NUM, rel_key);
        }
    }
   }

   return ret;
}


static uint8_t get_gpio_by_index(uint8_t index)
{
     uint8_t ret;
     ret = (index < RELAIS_NUM) ? (&relais)->relais[index] : REL_ERR;

     return ret;	
}


static uint8_t get_state_by_index(uint8_t index)
{
     uint8_t ret;
     ret = (index < RELAIS_NUM) ? (&relais)->state[index] : REL_ERR;

     return ret;	
}

static uint8_t active_relais_count( void )
{
 uint8_t i, count = 0;

 for( i=0; i<RELAIS_NUM; i++)
 {
   count += ((&relais)->state[i] == STATE_ON) ? 1 : 0;
 }

 return count;
}

/* Public functions  */

uint8_t init_relais( void )
{
/* Initialises the relays with the last state stored in nvram or
 * if no nv value is available with a clear all off state 
 * */

   uint8_t ret, active=0;

   ret = init_relais_state();
   MSG("debug msg init func; ret = %i \n", ret);
   if(ret)
   {
     active = get_active_relais();
	MSG("active = %i \n", active);     
     if(active != 0)
     {
	MSG("switching on %i \n", active);     
      switch_relais_on(active);
     }else{
      switch_relais_off(OUTPUT_OFF);
	MSG("switching off %i \n", active);     
     }
   }
 return ret;
}

uint8_t switch_relais_on(uint8_t relais_num)
{

/* Sets the GPIO that controls the apropriate relais high.
 * Returns the level that has been set.
 * relais_num is a number from 1 to NUM_RELAIS 
 * */
  if(active_relais_count() == 0 || relais_num == get_active_relais())
  {
     uint8_t gpio_num, ret;

     gpio_num = get_gpio_by_index(relais_num-1);
   
     gpio_pad_select_gpio(gpio_num);
     gpio_set_direction(gpio_num, GPIO_MODE_OUTPUT);
     gpio_set_level(gpio_num, 1);

     (&relais)->state[relais_num-1] = STATE_ON;

     ret = write_blob_nv(relais_handle, (&relais)->state, RELAIS_NUM, rel_key);

    return ( (ret) ? get_state_by_index(relais_num-1) : REL_ERR );
 
  }

  return REL_ERR; // more than on active relais and desired active relais != current active one....

}


uint8_t switch_relais_off(uint8_t relais_num)
{

/* Sets the GPIO that controls the apropriate relais high.
 * Returns the level that has been set.
 * relais_num is a number between 1 and NUM_RELAIS or OUTPUT_OFF
 * */

    uint8_t gpio_num, ret;
    
    gpio_num = (relais_num != OUTPUT_OFF) ? get_gpio_by_index(relais_num-1) : get_active_relais();

    //todo: if rel num = 0 skip the last part
    if(gpio_num != 0)
    {
       gpio_pad_select_gpio(gpio_num);
       gpio_set_direction(gpio_num, GPIO_MODE_OUTPUT);
       gpio_set_level(gpio_num, 0);

       (&relais)->state[relais_num-1] = STATE_OFF;
    
       ret = write_blob_nv(relais_handle, (&relais)->state, RELAIS_NUM, rel_key);
       return ( (ret) ? get_state_by_index(relais_num-1) : REL_ERR );
     
    }
 return REL_ERR;
}

uint8_t get_active_relais(void)
{
/* This function returns the number of the current active relais.
 * Returns 0 if none is active
 * */

   uint8_t i;

   for(i=0; i<RELAIS_NUM; i++)
   {
    if(get_state_by_index(i) == STATE_ON)
    {
      return i+1;
    }
   }
  return 0;
}


