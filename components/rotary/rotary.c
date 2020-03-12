/*  Rotary control driver
 *  <F5>

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#include "esp_log.h"
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "sdkconfig.h"
#include "rotary.h"

#define TAG "Rotary"

//values provided by sdkconfig
#define PCNT0_PULSE_GPIO	CONFIG_PCNT0_PULSE_GPIO	 // gpio for PCNT0
#define PCNT0_CONTROL_GPIO      CONFIG_PCNT0_CONTROL_GPIO
#define ENC0_SW_GPIO            CONFIG_ENC0_SW_GPIO

#define PCNT1_PULSE_GPIO	CONFIG_PCNT1_PULSE_GPIO	 // gpio for PCNT1
#define PCNT1_CONTROL_GPIO      CONFIG_PCNT1_CONTROL_GPIO
#define ENC1_SW_GPIO            CONFIG_ENC1_SW_GPIO

//end sdkconfig

#define REP_0_MAX               24
#define REP_0_MIN               1
#define REP_1_MAX               5
#define REP_1_MIN               1



#define USED_UNITS 	2

#define PCNT0_THRESH0_VAL      -100  //neg overrun val
#define PCNT0_THRESH1_VAL       100  //overrun val

#define PCNT1_THRESH0_VAL       -10  //neg overrun val
#define PCNT1_THRESH1_VAL       10 //overrun val

#define ENC_0_PIN_SEL		(1ULL << ENC0_SW_GPIO)
#define ENC_1_PIN_SEL		(1ULL << ENC1_SW_GPIO)

#define GPIO_DELAY             (1000 / portTICK_PERIOD_MS)
#define PCNT_DELAY             (1000 / portTICK_PERIOD_MS)

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

gpio_config_t gpio_enc_0_config =
{
   .pin_bit_mask = ENC_0_PIN_SEL,
   .mode         = GPIO_MODE_INPUT,
   .pull_up_en   = GPIO_PULLUP_DISABLE, //using hw pullups for a remotely attached sensor
   .pull_down_en = GPIO_PULLDOWN_DISABLE,
   .intr_type = GPIO_INTR_NEGEDGE,
};

gpio_config_t gpio_enc_1_config =
{
   .pin_bit_mask = ENC_1_PIN_SEL,
   .mode         = GPIO_MODE_INPUT,
   .pull_up_en   = GPIO_PULLUP_DISABLE, //using hw pullups for an remotely attached sensor
   .pull_down_en = GPIO_PULLDOWN_DISABLE,
   .intr_type = GPIO_INTR_NEGEDGE,
};

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

xQueueHandle pcnt_evt_queues[USED_UNITS];// A queue to handle pulse counter events
xQueueHandle gpio_evt_queues[USED_UNITS];// A queue to handle pulse counter events



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
            xQueueSendFromISR(pcnt_evt_queues[i], &evt, &HPTaskAwoken);
            if (HPTaskAwoken == pdTRUE)
            {
                portYIELD_FROM_ISR();
            }
        }
    }
}

//get the level and enqueue it.
static void IRAM_ATTR gpio_isr_handler_0(void* arg)
{
   gpio_evt_t evt;
   evt.gpio_num = ENC0_SW_GPIO;
   evt.status = gpio_get_level(evt.gpio_num);
   xQueueSendFromISR(gpio_evt_queues[0], &evt, NULL);

}

//get the level and enqueue it.
static void IRAM_ATTR gpio_isr_handler_1(void* arg)
{
   gpio_evt_t evt;
   evt.gpio_num = ENC1_SW_GPIO;
   evt.status = gpio_get_level(evt.gpio_num);
   xQueueSendFromISR(gpio_evt_queues[1], &evt, NULL);

}

static esp_err_t enc_0_gpio_init(void)
{
  esp_err_t err = ESP_OK;
  ESP_LOGI(TAG, "GPIO number for rot0 SWGPIO: %d", ENC0_SW_GPIO);
  ESP_LOGI(TAG, "GPIO number for rot0 CTRL: %d", PCNT0_CONTROL_GPIO);
  ESP_LOGI(TAG, "GPIO number for rot0 PULSE: %d", PCNT0_PULSE_GPIO);
  err = gpio_config(&gpio_enc_0_config);
  if(err != ESP_OK) {return err;}
  err = gpio_isr_handler_add(ENC0_SW_GPIO, gpio_isr_handler_0, NULL);
  if(err != ESP_OK) {return err;}
  gpio_evt_queues[0]  = xQueueCreate(2, sizeof(gpio_evt_t));
  return err;
}

static void enc_1_gpio_init(void)
{
  esp_err_t err = ESP_OK;
  ESP_LOGI(TAG, "GPIO number for rot1 SWGPIO: %d", ENC1_SW_GPIO);
  ESP_LOGI(TAG, "GPIO number for rot1 CTRL: %d", PCNT1_CONTROL_GPIO);
  ESP_LOGI(TAG, "GPIO number for rot1 PULSE: %d", PCNT1_PULSE_GPIO);
  err = gpio_config(&gpio_enc_1_config);
  if(err != ESP_OK) {return err;}
  err = gpio_isr_handler_add(ENC1_SW_GPIO, gpio_isr_handler_1, NULL);
  if(err != ESP_OK) {return err;}
  gpio_evt_queues[1]  = xQueueCreate(2, sizeof(gpio_evt_t));
  return err;
}


static void encoder_0_counter_init(quad_encoder_mode enc_mode)
{
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

   pcnt_evt_queues[0]  = xQueueCreate(10, sizeof(pcnt_evt_t));
}


static void encoder_1_counter_init(quad_encoder_mode enc_mode)
{
   switch (enc_mode)
   {
    case QUAD_ENC_MODE_1:
       break;
    case QUAD_ENC_MODE_2:
       pcnt_1_config.neg_mode = PCNT_COUNT_DEC;
       break;
   }

    pcnt_unit_config(&pcnt_1_config);
    pcnt_set_filter_value(PCNT_UNIT_1, 1000);
    pcnt_filter_enable(PCNT_UNIT_1);

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

    pcnt_evt_queues[1]  = xQueueCreate(10, sizeof(pcnt_evt_t));
}

//---------------------------------------------------------------------------------------------------------------


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

//initialize all used quad encoder modules and the gpio pins for the rotary switch function
// enc mode: QUAD_ENC_MODE_1 => encoder counts upwards (standard)
//           QUAD_ENC_MODE_2 => encoder counts downwards

esp_err_t rotary_init(quad_encoder_mode enc_mode)
{

    eps_err_t err = ESP_OK;
  //there seems to be a bug that prevents multiple calls to the gpio_install_isr_service
    err = gpio_install_isr_service(0);
    if(err != ESP_OK) {return err;}

    err = enc_0_gpio_init();
    if(err != ESP_OK) {return err;}
    err = enc_1_gpio_init();
    if(err != ESP_OK) {return err;}

    encoder_0_counter_init(enc_mode);
    encoder_1_counter_init(enc_mode);

    return err;

}

//returns the value of the gpio pin that caused the interrupt
//returns -1 in case of error or no new event
//must be called from within a task
int8_t rotary_0_gpio_val( void )
{
  int8_t ret = -1;
  portBASE_TYPE res;
  gpio_evt_t evt;

  res = xQueueReceive(gpio_evt_queues[0], &evt, GPIO_DELAY );
  ret = (res == pdTRUE) ? evt.status : -1;

return ret;
}

//returns the value of the gpio pin that caused the interrupt
//returns -1 in case of error or no new event
//must be called from within a task
int8_t rotary_1_gpio_val( void )
{
  int8_t ret = -1;
  portBASE_TYPE res;
  gpio_evt_t evt;

  res = xQueueReceive(gpio_evt_queues[1], &evt, GPIO_DELAY );
  ret = (res == pdTRUE) ? evt.status : -1;

return ret;
}

//returns the sanitized counter value for the defined max, min boundaries
//returns -1 in case of no evt received or error
//must be called from within a task
int8_t rotary_0_counter_val( void )
{

   static int16_t count = 0;
   static int16_t old_count  = 0;
   static uint8_t rep_count = 0;

   static pcnt_evt_t evt;

   portBASE_TYPE res;
   int8_t ret = -1;
   /* Wait for the event information passed from PCNT's interrupt handler.
    * Once received, decode the event type and print it on the serial monitor.
    */
   res = xQueueReceive(pcnt_evt_queues[0], &evt, PCNT_DELAY);

   old_count=count;
   pcnt_get_counter_value(evt.unit, &count);

   if (res != pdTRUE)
   {
     rep_count =  handle_pcnt(REP_0_MAX, REP_0_MIN, old_count, count, rep_count);
     //MSG("| Reportet counter 0 :%2d |\n", rep_count);
     return rep_count;
   }
  return ret;
}

//returns the sanitized counter value for the defined max, min boundaries
//returns -1 in case of no evt received or error
//must be called from within a task
int8_t rotary_1_counter_val( void )
{

   static int16_t count = 0;
   int16_t old_count  = 0;
   static uint8_t rep_count = 0;

   static pcnt_evt_t evt;

   portBASE_TYPE res;
   int8_t ret = -1;
   /* Wait for the event information passed from PCNT's interrupt handler.
    * Once received, decode the event type and print it on the serial monitor.
    */
   res = xQueueReceive(pcnt_evt_queues[1], &evt, PCNT_DELAY);

   old_count=count;
   pcnt_get_counter_value(evt.unit, &count);

   if (res != pdTRUE)
   {
     rep_count =  handle_pcnt(REP_1_MAX, REP_1_MIN, old_count, count, rep_count);
     //MSG("| Reportet counter 1 :%2d |\n", rep_count);
     return rep_count;
   }
 return ret;
}
