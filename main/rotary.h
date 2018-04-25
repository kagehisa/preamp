/*  Rotary control driver

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#ifndef ROTARY_H
#define ROTARY_H

#define PCNT0_PULSE_GPIO            12      // gpio for PCNT0
#define PCNT1_PULSE_GPIO                  // gpio for PCNT1
#define PCNT0_CONTROL_GPIO         14
#define PCNT1_CONTROL_GPIO         

#define PCNT0_THRESH0_VAL	0  //min volume
#define PCNT0_THRESH1_VAL	23 //max volume 

#define PCNT1_THRESH0_VAL	1  //min volume
#define PCNT1_THRESH1_VAL	5 //max volume 


typedef enum {
    QUAD_ENC_MODE_1 = 1,
   QUAD_ENC_MODE_2 = 2,
} quad_encoder_mode;

/* A sample structure to pass events from the PCNT
 * interrupt handler to the main program.
 */
typedef struct {
    uint8_t unit;  // the PCNT unit that originated an interrupt
    uint32_t status; // information on the event type that caused the interrupt
} pcnt_evt_t;

//initalises the PCNT with the values needed for handling the rotary encoder
void encoder_0_counter_init(quad_encoder_mode enc_mode); 

//initalises the PCNT with the values needed for handling the rotary encoder
//void encoder_1_counter_init(quad_encoder_mode enc_mode); 

void event_handler( void );


#endif /* ROTARY_H */
