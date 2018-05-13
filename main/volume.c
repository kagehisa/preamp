/*
   This code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "msg_stuff.h"

#include "volume.h"

#define I2C_MASTER_TX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE  0                /*!< I2C master do not need buffer */

#define WRITE_BIT                          I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                           I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                            0x0              /*!< I2C ack value */
#define NACK_VAL                           0x1              /*!< I2C nack value */


typedef union
  {
    uint32_t volume;
    uint8_t regs[4];
  } adg_regs;

uint8_t ux_addr[3] = {U1_ADDR, U2_ADDR, U3_ADDR};
/**
 * @brief code to read esp-i2c-slave
 *        We need to fill the buffer of esp slave device, then master can read them out.
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 */
static esp_err_t read_adg(uint8_t adg_addr, uint8_t* data)
{
    i2c_port_t i2c_num = I2C_MASTER_NUM;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( adg_addr << 1 ) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data_rd, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief code to write esp-i2c-slave
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */
static esp_err_t write_adg(uint8_t adg_addr, uint8_t data)
{
    i2c_port_t i2c_num = I2C_MASTER_NUM;	
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( adg_addr << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t set_volume(uint8_t vol)
{
  esp_err_t ret;
  
  if((vol > MAX_VOL || vol < MIN_VOL) && vol != MUTE_VOL )
  {
   return 0x102; //volume out of bound
  }else{
    uint8_t shift = (vol == MUTE_VOL) ? 0 : (vol-1); //mute case
    uint32_t shift_op = (vol == 0) ? 0 : 1; //mute case

    adg_regs adg_registers;
  
    adg_registers.volume = (shift_op<<shift);

    for(i=0;i<3; i++)
    {
     ret = write_adg(ux_addr[i], adg_registers.regs[i])
     if (ret != ESP_OK){ return ret;  }
    }

  }
  




}


/**
 * @brief i2c master initialization
 */ 
	//TODO: nvs init...
void volume_init()
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_MASTER_RX_BUF_DISABLE,
                       I2C_MASTER_TX_BUF_DISABLE, 0);
}
