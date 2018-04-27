/*  Rotary control driver

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#ifndef ROTARY_H
#define ROTARY_H

#define PCNT0_PULSE_GPIO            12      // gpio for PCNT0
#define PCNT1_PULSE_GPIO            27      // gpio for PCNT1

#define PCNT0_CONTROL_GPIO         14
#define PCNT1_CONTROL_GPIO         26

#define REP_0_MAX		24
#define REP_0_MIN		1

#define REP_1_MAX		5
#define REP_1_MIN		1


//define the reposrt function to report to the event_handler
#define REPORT_FUNC_ROTARY_0( unit, count ) MSG( unit, count )
#define REPORT_FUNC_ROTARY_1( unit, count ) MSG( unit, count )



typedef enum {
   QUAD_ENC_MODE_1 = 1,
   QUAD_ENC_MODE_2 = 2,
} quad_encoder_mode;


//initalises the PCNT with the values needed for handling the rotary encoder
void encoder_0_counter_init(quad_encoder_mode enc_mode); 
void encoder_1_counter_init(quad_encoder_mode enc_mode); 

void rotary_0_event_handler( void );
void rotary_1_event_handler( void );


#endif /* ROTARY_H */
