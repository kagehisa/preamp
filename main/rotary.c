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
//#include <esp_log.h>
#include "driver/pcnt.h"

#include "rotary.h"

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


xQueueHandle pcnt_0_evt_queue;   // A queue to handle pulse counter events

static void IRAM_ATTR quad_enc_isr(void *arg)
{
    uint32_t intr_status = PCNT.int_st.val;
    uint8_t i = 0;
    pcnt_evt_t evt;
    portBASE_TYPE HPTaskAwoken = pdFALSE;

    //for (i = 0; i < PCNT_UNIT_MAX; i++) 
    //{
        if (intr_status & (BIT(i))) 
        {
            evt.unit = i;
            /* Save the PCNT event type that caused an interrupt
               to pass it to the main program */
            evt.status = PCNT.status_unit[i].val;
            PCNT.int_clr.val = BIT(i);
            xQueueSendFromISR(pcnt_0_evt_queue, &evt, &HPTaskAwoken);
            if (HPTaskAwoken == pdTRUE) 
            {
                portYIELD_FROM_ISR();
            }
        }
    //}
}

static void enc_0_gpio_init() 
{
//   gpio_pad_select_gpio(DIRECTION);
//   gpio_set_direction(DIRECTION,GPIO_MODE_INPUT);
//   gpio_set_pull_mode(DIRECTION, GPIO_PULLDOWN_ONLY);
}


void encoder_0_counter_init(quad_encoder_mode enc_mode) 
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

    //pcnt_set_event_value(PCNT_UNIT_0, PCNT_EVT_THRES_1, PCNT0_THRESH1_VAL);
    //pcnt_set_event_value(PCNT_UNIT_0, PCNT_EVT_THRES_0, PCNT0_THRESH0_VAL);
    //pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_THRES_1); 
    //PCNT_UNIT_0pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_THRES_0); 

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


void event_handler()
{

    /* Initialize PCNT event queue and PCNT functions */
       pcnt_0_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
       encoder_0_counter_init(QUAD_ENC_MODE_1);

       int16_t count = PCNT0_THRESH0_VAL;
       int16_t rep_count = PCNT0_THRESH0_VAL;
       int16_t old_count = count;
       uint8_t mult = 1;

       pcnt_evt_t evt;
       portBASE_TYPE res;

        volatile uint8_t min_event=0, max_event=0;

       while (1) {
           /* Wait for the event information passed from PCNT's interrupt handler.
            * Once received, decode the event type and print it on the serial monitor.
            */
           res = xQueueReceive(pcnt_0_evt_queue, &evt, 1000 / portTICK_PERIOD_MS);

	   old_count=count;
           pcnt_get_counter_value(PCNT_UNIT_0, &count);

           if (res == pdTRUE) {
               MSG("Event PCNT unit[%d]; cnt: %d\n", evt.unit, count);
               if (evt.status & PCNT_STATUS_H_LIM_M) {
                    MSG("HLIM EVT\n");
		    max_event^=1;
               }
               if (evt.status & PCNT_STATUS_L_LIM_M) {
                    MSG("LLIM EVT\n");
		    min_event^=1;
               }
               if (evt.status & PCNT_STATUS_ZERO_M) {
                   MSG("ZERO EVT\n");
               }
           } else {
	       if( max_event )
	       { 
		 if(old_count <= count)//still counting up after max value reached
		 {
                   rep_count = PCNT0_THRESH1_VAL;

	         }else{ // we are counting down but now count >= 0 and repcount = max_val

		   //the magic happens here... 
		   //
		   rep_count = count;
		   //erst setzen wenn count und repcount gleich sind
		   max_event^=1;
		 }
	       }else{
                 rep_count = count;
	       }

	       //f( min_event)

	       //count= (min_event) ? PCNT0_THRESH0_VAL : count;
               MSG("Current counter value :%d", count);
               MSG("  | Reportet counter value :%d\n", rep_count);
	       
           }
       }
}

