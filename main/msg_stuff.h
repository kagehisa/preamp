/*  Accessory lib for message output and so on....

   This code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#ifndef MSG_STUFF_H
#define MSG_STUFF_H

#include <stdio.h>
#include "sdkconfig.h"

#ifdef CONFIG_MSG_SILENT
#define MSG(...)
#else 
#define MSG printf
#endif

#endif /* MSG_STUFF_H */
