/*  Event Handler

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "relais.h"
#include "volume.h"
#include "rotary.h"


static uint8_t vol_change;
xQueueHandle timer_queue;

static void example_tg0_timer_init(int timer_idx, double timer_interval_sec)
{
  /* Select and initialize basic parameters of the timer */
  timer_config_t config;
  config.divider = TIMER_DIVIDER;
  config.counter_dir = TIMER_COUNT_UP;
  config.counter_en = TIMER_PAUSE;
  config.alarm_en = TIMER_ALARM_EN;
  config.intr_type = TIMER_INTR_LEVEL;
  config.auto_reload = 1;
  timer_init(TIMER_GROUP_0, timer_idx, &config);

  /* Timer's counter will initially start from value below.
   * Also, if auto_reload is set, this value will be automatically reload on alarm */
  timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);
 
  /* Configure the alarm value and the interrupt on alarm. */
  timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_sec * TIMER_SCALE);
  timer_enable_intr(TIMER_GROUP_0, timer_idx);
  timer_isr_register(TIMER_GROUP_0, timer_idx, timer_group0_isr, 
  (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);
  timer_start(TIMER_GROUP_0, timer_idx);
}




void IRAM_ATTR timer_group0_isr(void *para)
{
  int timer_idx = (int) para;

  uint8_t evt;
  TIMERG0.hw_timer[timer_idx].update = 1;


  if(vol_change == 1)
  {
    evt = 1;
    xQueueSendFromISR(timer_queue, &evt, NULL);
  }

  //re enable interrupt and re enable the alarm
  if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_0) 
  {
    TIMERG0.int_clr_timers.t0 = 1;
  }

  TIMERG0.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN;
}




/* Handling the volume case. 
 * This function is intended to be run as a Task
 * */

void volume_handler(void *pvParameter)
{
 uint8_t oldvol = 0, tmp = 0;
 uint8_t mute = 0, mutesave = 0;
 uint8_t evt;
 esp_err_t err;
 
 //pcnt0 is for volume
 //TODO:additional inits?
 
 encoder_0_counter_init();
 volume_init(); 

 

 get_fast_volume(&oldvol);

 while(1)
 {
  tmp = rotary_0_counter_val();

  if(tmp != oldvol)
  {
    set_volume(tmp);
    oldvol = tmp;
    vol_change = 1;
    loopcnt = 0;

    //add stuff like add display
  }

  if(rotary_0_gpio_val() == 1)//butten pressed mute it is...
  {
   loopcnt = 0;	  
   if(mute == 0)
   {
     err = get_volume(&mutesave);
     if(err == ESP_OK)
     {    
       mute = (set_volume(0) == ESP_OK) ? 1 : 0; //if fail enable a retry
     }
   }else{ //mute = 1
       mute = (set_volume(mutesave) == ESP_OK) ? 0 : 1; //if fail enable a retry
   }

  }//end of mute

  // may decrease the queue waiting time.. TODO: adapting...
  if(xQueueReceive( timer_queue, &evt, (1000 / portTICK_PERIOD_MS) ) == pdTRUE)
  {
    pers_volume();
    vol_change = 0;
  }

 }
}



void output_handler(void *pvParameter)
{
 while(1)
 {
 //pcnt1 is the output chooser
 }
}




















