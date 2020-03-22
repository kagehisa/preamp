/**
 * Copyright (c) 2017-2018 Tara Keeling
 *
 * This software is released under the MIT License.
 * https://opensource.org/licenses/MIT
 */

#include <stdbool.h>
#include "esp_log.h"
#include "ssd1306.h"
#include "ssd1306_draw.h"
#include "ssd1306_font.h"
#include "ssd1306_default_if.h"

#include "display.h"

#define LOG_TAG "SSD1306"

/*--------------values per Display------------*/
//derive from config
#define I2C_VOL_ADDRESS CONFIG_VOL_DISP_ADDR
#define VOL_DISP_WIDTH CONFIG_VOL_DISP_WIDTH
#define VOL_DISP_HEIGHT CONFIG_VOL_DISP_HEIGHT
#define VOLRESETPIN CONFIG_VOLRESETPIN

#define I2C_INP_ADDRESS CONFIG_INP_DISP_ADDR
#define INP_DISP_WIDTH CONFIG_INP_DISP_WIDTH
#define INP_DISP_HEIGHT CONFIG_INP_DISP_HEIGHT
#define INPRESETPIN CONFIG_INPRESETPIN


#define VOL_FONT &Font_droid_sans_fallback_24x28
#define INP_FONT &Font_droid_sans_fallback_24x28

struct SSD1306_Device VolDisplay;
struct SSD1306_Device InpDisplay;

static uint8_t once = 0;

static esp_err_t DefaultBusInit( struct SSD1306_Device* DisplayHandle, const uint8_t I2CAddress, const uint8_t DispWidth, const uint8_t DispHeight, const int16_t ResetPin)
{
        if(once == 0)
        {
          assert( SSD1306_I2CMasterInitDefault( ) == true );
          once = 1;
        }
        assert( SSD1306_I2CMasterAttachDisplayDefault( DisplayHandle, DispWidth, DispHeight, I2CAddress, ResetPin ) == true );

    return ESP_OK;
}


static void vol_to_bar(uint8_t volume)
{
  uint8_t xMin = 3, yMin = 3, yMax = 59;
  uint8_t bar_width = 5;

  for(int i = xMin, j = xMin+bar_width, count = 0; count <= volume; i+=bar_width, j+=bar_width, count++)
  {
    SSD1306_DrawBox( &VolDisplay, i+1, yMin+1, j-1, yMax-1, SSD_COLOR_WHITE, true );
  }
}

void volDispMute( void ) //manually write and update the display
{
  SSD1306_Clear( &VolDisplay, SSD_COLOR_BLACK );
  SSD1306_FontDrawAnchoredString( &VolDisplay, TextAnchor_Center, "MUTE", SSD_COLOR_WHITE );
  SSD1306_Update( &VolDisplay );
}

void volDispWrite(uint8_t volume) //manually write and update the display
{
  SSD1306_Clear( &VolDisplay, SSD_COLOR_BLACK );
  vol_to_bar(volume);
  SSD1306_Update( &VolDisplay );
}


esp_err_t initVolDisp(void)
{
	ESP_LOGI( LOG_TAG, "Starting Initialisation for Volume Display.\n" );
	if( DefaultBusInit( &VolDisplay, I2C_VOL_ADDRESS, VOL_DISP_WIDTH, VOL_DISP_HEIGHT, VOLRESETPIN ) == ESP_OK )
	{
    ESP_LOGI( LOG_TAG, "Volume Display Init Succesfull.\n" );
	  SSD1306_Clear( &VolDisplay, SSD_COLOR_BLACK );
	  SSD1306_SetFont( &VolDisplay, VOL_FONT );
    SSD1306_Update( &VolDisplay );
		SSD1306_FontDrawAnchoredString( &VolDisplay, TextAnchor_Center, "Volume", SSD_COLOR_WHITE );
    SSD1306_Update( &VolDisplay );
  }else{ return ESP_FAIL; }

 return ESP_OK;
}


void inpDispWrite(uint8_t input_num) //manually write and update the display
{
  const char* outs[] = {"0", "1", "2", "3", "4", "5", "6"};
  SSD1306_Clear( &InpDisplay, SSD_COLOR_BLACK );
  SSD1306_FontDrawAnchoredString( &InpDisplay, TextAnchor_Center, outs[input_num], SSD_COLOR_WHITE );
  SSD1306_Update( &InpDisplay );
}

esp_err_t initInpDispl( void ) //init function for input chooser display
{
	ESP_LOGI( LOG_TAG, "Starting Initialisation for Input chooser Display.\n" );
	if( DefaultBusInit( &InpDisplay, I2C_INP_ADDRESS, INP_DISP_WIDTH, INP_DISP_HEIGHT, INPRESETPIN ) == ESP_OK )
	{
    ESP_LOGI( LOG_TAG, "Input chooser Display Init Succesfull.\n" );
	  SSD1306_Clear( &InpDisplay, SSD_COLOR_BLACK );
	  SSD1306_SetFont( &InpDisplay, INP_FONT );
    SSD1306_Update( &InpDisplay );
		SSD1306_FontDrawAnchoredString( &InpDisplay, TextAnchor_Center, "Input", SSD_COLOR_WHITE );
    SSD1306_Update( &InpDisplay );
  }else{ return ESP_FAIL; }

 return ESP_OK;
}
