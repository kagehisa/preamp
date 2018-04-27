/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "msg_stuff.h"
#include "relais.h"
#include "rotary.h"

void switch_task(void *pvParameter)
{
    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */

    uint8_t i;
    if(init_relais())
    {
     vTaskDelay(4000 / portTICK_PERIOD_MS);	    
     while(1) 
     {
        
        for(i = 1; i <= 5; i++)
	{
		/* relais on */
		MSG("switching on relais %i \n", i);
        	switch_relais_on(i);
        	vTaskDelay(3000 / portTICK_PERIOD_MS);
        	/* relais off */
		MSG("switching off relais %i \n", i);
        	switch_relais_off(i);
        	vTaskDelay(3000 / portTICK_PERIOD_MS);
       	}
     }
    }
}

void rotary_task_0(void *pvParameter)
{
 rotary_0_event_handler();
}

void rotary_task_1(void *pvParameter)
{
 rotary_1_event_handler();
}

void app_main()
{   
    //xTaskCreate(&switch_task, "relais_switch_task", configMINIMAL_STACK_SIZE*4, NULL, 5, NULL);
    xTaskCreate(&rotary_task_0, "rotary_task0", configMINIMAL_STACK_SIZE*4, NULL, 5, NULL);
    xTaskCreate(&rotary_task_1, "rotary_task1", configMINIMAL_STACK_SIZE*4, NULL, 5, NULL);
}
