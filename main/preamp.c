/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "evt_handler.h"


/*
void rotary_task_0(void *pvParameter)
{
//encoder_0_counter_init(1);
 while(1)
 {
  if(rotary_0_gpio_val() != -1)
  {
   MSG("counter 0 value %d\n", rotary_0_counter_val());
  }
 }
}

void rotary_task_1(void *pvParameter)
{
//encoder_1_counter_init(1);
 while(1)
 {
  if(rotary_1_gpio_val() != -1)
  {
   MSG("counter 1 value %d\n", rotary_1_counter_val());
  }
 }
}
*/

void app_main()
{
    //rotary_init(QUAD_ENC_MODE_1);
    //xTaskCreate(&rotary_task_0, "rotary_task0", configMINIMAL_STACK_SIZE*4, NULL, 5, NULL);
    //xTaskCreate(&rotary_task_1, "rotary_task1", configMINIMAL_STACK_SIZE*4, NULL, 5, NULL);


    xTaskCreate(&rotary_handler, "physicalInput_task", configMINIMAL_STACK_SIZE*4, NULL, 5, NULL);
}
