/*  Rotary control driver

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#ifndef ROTARY_H
#define ROTARY_H


typedef enum {
   QUAD_ENC_MODE_1 = 1,
   QUAD_ENC_MODE_2 = 2,
} quad_encoder_mode;

//event structures to handle the the interrupt repoorting
typedef struct {
        uint8_t unit;  // the PCNT unit that originated an interrupt;
        uint32_t status; // information on the event type that caused the interrupt
} pcnt_evt_t;


typedef struct {
        uint8_t gpio_num;  // gpio unit that originated an interrupt;
        uint8_t status; //  type that caused the interrupt
} gpio_evt_t;

//initalises the PCNT with the values needed for handling the rotary encoder
void encoder_0_counter_init(quad_encoder_mode enc_mode); 
void encoder_1_counter_init(quad_encoder_mode enc_mode); 

int8_t rotary_0_counter_val( void );
int8_t rotary_1_counter_val( void );

int8_t rotary_0_gpio_val( void );
int8_t rotary_1_gpio_val( void );

#endif /* ROTARY_H */
