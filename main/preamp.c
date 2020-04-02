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


void app_main()
{
//    xTaskCreate(&rotary_handler, "physicalInput_task", configMINIMAL_STACK_SIZE*4, NULL, 5, NULL);

    xTaskCreatePinnedToCore(&rotary_handler, "physicalInput_task", configMINIMAL_STACK_SIZE*4, NULL, 5, NULL, 0);
}
