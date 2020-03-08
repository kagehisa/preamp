#ifndef DISPLAY_H_
#define DISPLAY_H_

#include "esp_err.h"

//init function for volume display
esp_err_t init_volDisp( void );

//write volume value in the form of a bar graph and update the display
void volDispWrite(uint8_t volume);

//write "MUTE" and update the display
void volDispMute( void );


//init function for input chooser display
esp_err_t initInpDispl( void );

//manually write and update the display
void inpDispWrite(uint8_t input_num);

#endif
