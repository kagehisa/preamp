/*  Rotary control driver

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#ifndef ROTARY_H
#define ROTARY_H

#define PCNT_PULSE_GPIO            12      // gpio for PCNT
#define PCNT_CONTROL_GPIO         14

#define PCNT_H_LIM_VAL      24 //maximal volume
#define PCNT_L_LIM_VAL     -1 //minimal volume (zero as mute is an extra event)

typedef enum {
    QUAD_ENC_MODE_1 = 1,
   QUAD_ENC_MODE_2 = 2,
   QUAD_ENC_MODE_4 = 4
} quad_encoder_mode;

/* A sample structure to pass events from the PCNT
 * interrupt handler to the main program.
 */
typedef struct {
    uint8_t unit;  // the PCNT unit that originated an interrupt
    uint32_t status; // information on the event type that caused the interrupt
} pcnt_evt_t;

//initalises the PCNT with the values needed for handling the rotary encoder
void encoder_counter_init(quad_encoder_mode enc_mode); 

void event_handler( void );


#endif /* ROTARY_H */
