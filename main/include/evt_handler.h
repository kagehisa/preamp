/*  Accessory lib for message output and so on....

   This code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#ifndef EVT_HANDLER_H
#define EVT_HANDLER_H

#include "esp_err.h"

esp_err_t system_init(void);

void volume_handler(void *pvParameter);

void input_handler(void *pvParameter);

#endif /* MSG_STUFF_H */
