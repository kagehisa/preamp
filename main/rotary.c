/*  Rotary control driver

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#include "msg_stuff.h"

#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"

#include "rotary.h"

#define USED_UNITS 	2

#define PCNT0_THRESH0_VAL      -100  //neg overrun val
#define PCNT0_THRESH1_VAL       100  //overrun val

#define PCNT1_THRESH0_VAL       -10  //neg overrun val
#define PCNT1_THRESH1_VAL       10 //overrun val 

/* A sample structure to pass events from the PCNT
** interrupt handler to the main program.
**/
typedef struct {
	uint8_t unit;  // the PCNT unit that originated an interrupt;
	uint32_t status; // information on the event type that caused the interrupt
} pcnt_evt_t;


/* PCNT0 configuration please look at the init as well*/
pcnt_config_t pcnt_0_config =
{
    .pulse_gpio_num = PCNT0_PULSE_GPIO,
    .ctrl_gpio_num = PCNT0_CONTROL_GPIO,
    .channel = PCNT_CHANNEL_0,
    .unit = PCNT_UNIT_0,
    .pos_mode = PCNT_COUNT_INC,            // Count up on the positive edge
    .neg_mode = PCNT_COUNT_DIS,            // Keep the counter value on the negative edge
    .lctrl_mode = PCNT_MODE_KEEP,          // Reverse counting direction if low
    .hctrl_mode = PCNT_MODE_REVERSE,       // Keep the primary counter mode if high
    .counter_h_lim = PCNT0_THRESH1_VAL,
    .counter_l_lim = PCNT0_THRESH0_VAL,
    
};

xQueueHandle pcnt_evt_queue;   // A queue to handle pulse counter events


pcnt_config_t pcnt_1_config =
{
    .pulse_gpio_num = PCNT1_PULSE_GPIO,
    .ctrl_gpio_num = PCNT1_CONTROL_GPIO,
    .channel = PCNT_CHANNEL_0,
    .unit = PCNT_UNIT_1,
    .pos_mode = PCNT_COUNT_INC,            // Count up on the positive edge
    .neg_mode = PCNT_COUNT_DIS,            // Keep the counter value on the negative edge
    .lctrl_mode = PCNT_MODE_KEEP,          // Reverse counting direction if low
    .hctrl_mode = PCNT_MODE_REVERSE,       // Keep the primary counter mode if high
    .counter_h_lim = PCNT1_THRESH1_VAL,
    .counter_l_lim = PCNT1_THRESH0_VAL,
    
};


static void IRAM_ATTR quad_enc_isr(void *arg)
{
    uint32_t intr_status = PCNT.int_st.val;
    uint8_t i = 0;
    pcnt_evt_t evt;
    portBASE_TYPE HPTaskAwoken = pdFALSE;

    for (i = 0; i < USED_UNITS; i++) 
    {
        if (intr_status & (BIT(i))) 
        {
            evt.unit = i;
            /* Save the PCNT event type that caused an interrupt
               to pass it to the main program */
            evt.status = PCNT.status_unit[i].val;
            PCNT.int_clr.val = BIT(i);
            xQueueSendFromISR(pcnt_evt_queue, &evt, &HPTaskAwoken);
            if (HPTaskAwoken == pdTRUE) 
            {
                portYIELD_FROM_ISR();
            }
        }
    }
}

static void enc_0_gpio_init() 
{
//   gpio_pad_select_gpio(DIRECTION);
//   gpio_set_direction(DIRECTION,GPIO_MODE_INPUT);
//   gpio_set_pull_mode(DIRECTION, GPIO_PULLDOWN_ONLY);
}


static void encoder_0_counter_init(quad_encoder_mode enc_mode) 
{
    enc_0_gpio_init();

   switch (enc_mode) 
   {
    case QUAD_ENC_MODE_1:
       break;
    case QUAD_ENC_MODE_2:
       pcnt_0_config.neg_mode = PCNT_COUNT_DEC;
       break;
   }

    pcnt_unit_config(&pcnt_0_config);
    pcnt_set_filter_value(PCNT_UNIT_0, 1000);
    pcnt_filter_enable(PCNT_UNIT_0);

    pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_H_LIM);
    pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_L_LIM);

    /* Initialize PCNT's counter */
    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);

    /* Register ISR handler and enable interrupts for PCNT unit */
    pcnt_isr_register(quad_enc_isr, NULL, 0, NULL);
    pcnt_intr_enable(PCNT_UNIT_0);

    /* Everything is set up, now go to counting */
    pcnt_counter_resume(PCNT_UNIT_0);
}


static void enc_1_gpio_init() 
{
//   gpio_pad_select_gpio(DIRECTION);
//   gpio_set_direction(DIRECTION,GPIO_MODE_INPUT);
//   gpio_set_pull_mode(DIRECTION, GPIO_PULLDOWN_ONLY);
}


static void encoder_1_counter_init(quad_encoder_mode enc_mode) 
{
    enc_0_gpio_init();

   switch (enc_mode) 
   {
    case QUAD_ENC_MODE_1:
       break;
    case QUAD_ENC_MODE_2:
       pcnt_0_config.neg_mode = PCNT_COUNT_DEC;
       break;
   }

    pcnt_unit_config(&pcnt_1_config);
    pcnt_set_filter_value(PCNT_UNIT_0, 1000);
    pcnt_filter_enable(PCNT_UNIT_0);

    pcnt_event_enable(PCNT_UNIT_1, PCNT_EVT_H_LIM);
    pcnt_event_enable(PCNT_UNIT_1, PCNT_EVT_L_LIM);

    /* Initialize PCNT's counter */
    pcnt_counter_pause(PCNT_UNIT_1);
    pcnt_counter_clear(PCNT_UNIT_1);

    /* Register ISR handler and enable interrupts for PCNT unit */
    pcnt_isr_register(quad_enc_isr, NULL, 0, NULL);
    pcnt_intr_enable(PCNT_UNIT_1);

    /* Everything is set up, now go to counting */
    pcnt_counter_resume(PCNT_UNIT_1);
}

//---------------------------------------------------------------------------------------------------------------

// Function to init the used counters

void encoder_init(quad_encoder_mode enc_mode)
{
	encoder_0_counter_init(enc_mode); 
	encoder_1_counter_init(enc_mode); 
}


// return a cleaned counter value according to the given max and min values
static int16_t handle_pcnt(uint8_t max, uint8_t min, int16_t old_count, int16_t count, uint8_t rep_count)
{

   rep_count = (rep_count == 0) ? 1 : rep_count;	

   if((old_count <= count) && rep_count < max)
   {
	rep_count+= (count-old_count);
	rep_count = ((rep_count > max) || (rep_count< min)) ? max : rep_count;
   } 
   
   if((old_count > count) && rep_count > min)
   {
        rep_count-=(old_count-count);
	rep_count = ((rep_count < min) || (rep_count > max)) ? min : rep_count;
   }

return rep_count;

}


//one handler to rule them all...
void rotary_event_handler( void )
{

    /* Initialize PCNT event queue and PCNT functions */
       pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
       
       encoder_init(QUAD_ENC_MODE_1);

       int16_t count[USED_UNITS] = {0, 0};
       int16_t old_count[USED_UNITS] = {0, 0};
       uint8_t rep_count[USED_UNITS]= {0, 0};

       pcnt_evt_t evt;
       portBASE_TYPE res;

       while (1) 
       {
           /* Wait for the event information passed from PCNT's interrupt handler.
            * Once received, decode the event type and print it on the serial monitor.
            */
           res = xQueueReceive(pcnt_evt_queue, &evt, 1000 / portTICK_PERIOD_MS);

	   old_count[evt.unit]=count[evt.unit];
           pcnt_get_counter_value(evt.unit, count+(evt.unit));

           if (res != pdTRUE) 
	   {
               switch(evt.unit)
	       {
  		case 0:  rep_count[0] =  handle_pcnt(REP_0_MAX, REP_0_MIN, old_count[0], count[0], rep_count[0]);
               		 MSG("| Reportet counter 0 :%2d |\n", rep_count[0]);
			 break;
		case 1:  rep_count[1] =  handle_pcnt(REP_1_MAX, REP_1_MIN, old_count[1], count[1], rep_count[1]); 
                         MSG("| Reportet counter 1 :%2d |\n", rep_count[1]);
			 break;	
		default: break;
	       }
           }
       }
}

