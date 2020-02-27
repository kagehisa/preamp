# PreAmp
An esp32 based preamp volume and output control unit.
This project is based on the [muffsy stereo relay input selector](https://hackaday.io/project/46280-muffsy-stereo-relay-input-selector)
and the [Clone Note](http://www.buildanamp.com) LDR volume control. 

This readme is far from complete and also serves as a kind of log for me.

### TODO list software
- add SSD1306 diplay driver to the project
- check if a bitfield or union would suffice for volume_adg_val

### TODO list Hardware
- I2C line pullup before PCA9306 or after?
- check if seperate I2C ports are needed for display and volume control

## Hardware information

### Pinout
- Rotary encoder 1 (SW - GPIO13, DT - GPIO14, CLK - GPIO12)
- Rotary encoder 2 (SW - GPIO25, DT - GPIO26, CLK - GPIO27)
- I2C port for volume control (CLK - GPIO19, SDL - GPIO18)

### I2C Adresses
- ADG728 U1 0x4C
- ADG728 U2 0x4D
- ADG728 U3 0x4E

### Hardware used
- [ADG728 CMOS Switch](https://www.analog.com/media/en/technical-documentation/data-sheets/ADG728_729.pdf)
- [PCA9306 I2C Voltage Level Translator](http://www.ti.com/lit/ds/symlink/pca9306.pdf)
- ESP32 NODEMCU Module [DevkitC](https://docs.espressif.com/projects/esp-idf/en/latest/hw-reference/get-started-devkitc-v2.html), [ESP32 Datasheet](https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf), [Board schematic](https://dl.espressif.com/dl/schematics/ESP32-Core-Board-V2_sch.pdf)

