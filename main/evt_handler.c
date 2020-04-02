/*  Event Handler

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "relais.h"
#include "volume.h"
#include "rotary.h"
#include "driver/timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "display.h"


#include "evt_handler.h"

/* TIMER_BASE_CLK defined in timer.h */

#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER_INTERVAL0_SEC   (5) // sample test interval for the first timer
#define TEST_WITHOUT_RELOAD   0        // testing will be done without auto reload
#define TEST_WITH_RELOAD 1 // testing will be done with auto reload


#define TAG "EvtHandler"


static volatile uint8_t vol_change;
xQueueHandle timer_queue;


/* Timer ISR to generate a event and enqueue it.
This Event is used as a signal to persist the
volume value in nv ram */
void IRAM_ATTR timer_group0_isr(void *para)
{
  int timer_idx = (int) para;

  uint8_t evt;
  uint32_t intr_status = TIMERG0.int_st_timers.val;

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

/* Initialize the Timer with the above defined ISR */
static void tg0_timer_init(int timer_idx, double timer_interval_sec)
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



static esp_err_t system_init(void)
{
  esp_err_t err = ESP_OK;

  err = rotary_init(QUAD_ENC_MODE_1);
  if(err != ESP_OK){ return err; }
  // volume handler inits
  volume_init();
  tg0_timer_init(TIMER_0, TIMER_INTERVAL0_SEC);
  initVolDisp();

  //input handler inits
  init_relais();
  initInpDispl();

  timer_queue = xQueueCreate( 2, sizeof( uint8_t * ) );
   if( timer_queue == 0 )
   {
       err = ESP_FAIL;
   }

  return err;
}





/* Handling the volume case.
 * This function is intended to be run as a Task
 * */

void rotary_handler(void *pvParameter)
{
 uint8_t oldVol = 0, tmpVol = 0;
 uint8_t mute = 0, mutesave = 0;
 uint8_t volEvt, gpio_VolVal, gpio_InpVal;
 uint8_t oldIn = 0, tmpIn = 0, changeIn = 0;
 esp_err_t err;

 if ( system_init() == ESP_OK )
 {
   ESP_LOGI(TAG, "System Init successfull!");
 }

 get_fast_volume(&oldVol);
 err = get_active_relais(&oldIn);
 ESP_LOGI(TAG, "Value of err: %d, Active Relais: %d", err, oldIn);

 while(1)
 {
  /*------------------------------------Input-----------------------------------*/
  ESP_LOGI(TAG, "In the loop, input section!");
  err = rotary_1_counter_val(&tmpIn);
  ESP_LOGI(TAG, "Value of tmpIn: %d, err: %d, Active Relais: %d \n", tmpIn, err, oldIn);

  if(err == ESP_OK && tmpIn != oldIn)
  {
    //remeber the first old value to switch it off
    oldIn = (changeIn == 0) ? tmpIn : oldIn;

    //no output change yet, since writing output also writes nv ram
    //update display already, when writing relais persist display as well
    //relay writing on button press
    inpDispWrite(tmpIn);
    changeIn = 1;
  }

  //switch on selected input
  err = rotary_1_gpio_val(&gpio_InpVal);

  //button was pressed so we switch the selected output on
  if( err == ESP_OK && gpio_InpVal == 1 && changeIn == 1)
  {
    gpio_InpVal = 0;
    switch_relais_off(oldIn);
    inpDispWrite(tmpIn);
    switch_relais_on(tmpIn);
    changeIn = 0;
    oldIn = tmpIn;
  }

/*--------------------------------volume--------------------------------------*/
  ESP_LOGI(TAG, "In the loop, volume section!");
  err = rotary_0_counter_val(&tmpVol);
  ESP_LOGI(TAG, "Value of tmpVol: %d, err: %d, oldVol: %d \n", tmpVol, err, oldVol);
  if(err == ESP_OK && tmpVol != oldVol && mute == 0)
  {
    set_volume(tmpVol);
    volDispWrite(tmpVol);
    oldVol = tmpVol;
    vol_change = 1;
  }

  //mute case
  err = rotary_0_gpio_val(&gpio_VolVal);
  ESP_LOGI(TAG, "Mute Case after gpio_val get, Value of err: %d, vol_change: %d, gpio_val: %d \n", err, vol_change, gpio_VolVal);
  //button pressed: mute (mute = 0) or unmute (mute = 1)
  if(err == ESP_OK && gpio_VolVal == 1)
  {
   gpio_VolVal = 0;

   if(mute == 0)
   {
     err = get_volume(&mutesave);
     if(err == ESP_OK)
     {
       //if fail enable a recovery
       mute = (set_volume(0) == ESP_OK) ? 1 : 0;
       volDispMute();
     }
   }else{ //mute = 1
        //if fail enable a recovery
       mute = (set_volume(mutesave) == ESP_OK) ? 0 : 1;
       volDispWrite(mutesave);
   }

  }//end of mute
  ESP_LOGI(TAG, "Before timer_queue, Value of err: %d, vol_change: %d \n", err, vol_change);
  // may fiddle with the queue waiting time..
  if(xQueueReceive( timer_queue, &volEvt, (50 / portTICK_PERIOD_MS) ) == pdTRUE)
  {
    ESP_LOGI(TAG, "Persisiting Volume value!");
    pers_volume();
    vol_change = 0;
  }


 }
}
