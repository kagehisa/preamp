/*
   This code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "msg_stuff.h"
#include "nv_acc.h"
#include "driver/i2c.h"
#include "volume.h"

#define I2C_MASTER_TX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE  0                /*!< I2C master do not need buffer */

#define UX_NUM		3 /* number of registers we need to address  */
#define REG_NUM	        4

#define WRITE_BIT                          I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                           I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                            0x0              /*!< I2C ack value */
#define NACK_VAL                           0x1              /*!< I2C nack value */

//TODO: sdkconfig
#define I2C_SCL_IO          19               /*!< gpio number for I2C master clock */
//TODO: sdkconfig
#define I2C_SDA_IO          18               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM             I2C_NUM_1        /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ         100000           /*!< I2C master clock frequency */


//TODO: sdkconfig
#define U1_ADDR                            0x4C             /*!< slave address for U1 */
#define U2_ADDR                            0x4D             /*!< slave address for U2 */
#define U3_ADDR                            0x4E             /*!< slave address for U3 */



typedef union
  {
    uint32_t volume;
    uint8_t regs[4];
  } adg_regs;

adg_regs adg_registers;

uint8_t ux_addr[3] = {U1_ADDR, U2_ADDR, U3_ADDR};

/* handle for accessing the nvm  */
nvs_handle volume_handle;

/* key string for the nvm values of this module  */
const char* vol_key = "volume";

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
    i2c_master_read_byte(cmd, data, NACK_VAL);
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
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t set_volume(uint8_t vol)
{
  esp_err_t ret;
  uint8_t i;
  
  if((vol > MAX_VOL || vol < MIN_VOL) && vol != MUTE_VOL )
  {
   return 0x102; //volume out of bound
  }else{
    uint8_t shift = (vol == MUTE_VOL) ? 0 : (vol-1); //mute case
    uint32_t shift_op = (vol == 0) ? 0 : 1; //mute case


  
    adg_registers.volume = (shift_op<<shift);

    for(i=0;i<UX_NUM; i++)
    {
     ret = write_adg(ux_addr[i], adg_registers.regs[i]);
     if (ret != ESP_OK){ return ret;  }
    }

    return ret;

  }
}

esp_err_t get_volume(uint8_t *vol)
{
  esp_err_t ret;
  uint8_t i; 
  uint32_t tmp;
  
  for(i=0;i<UX_NUM; i++)
  {
    ret = read_adg(ux_addr[i], adg_registers.regs+i);
    if (ret != ESP_OK){ return ret;  }
  }

  i=0;
  tmp = adg_registers.volume;

  do
  {
   tmp = (tmp >> 1);
   i++;
  }while(tmp > 0);
  
  *vol = tmp;
  
  return ret;
}

static void i2c_init( void )
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


static esp_err_t init_vol_state( void )
{
  esp_err_t ret = ESP_OK;

  ret = init_nv();
  
  if(ret == ESP_OK)
  {
   ret = open_nv(&volume_handle);

   if(ret == ESP_OK)
   {
    ret = read_blob_nv(volume_handle, (&adg_registers)->regs, REG_NUM, vol_key );

    if(ret != ESP_OK) //there is no entry yet
    {    
     ret = write_blob_nv(volume_handle, (&adg_registers)->regs, REG_NUM, vol_key);
    }
   }
  }

  return ret;
}

/**
 * module initialization
 */ 
void volume_init( void )
{
   esp_err_t ret = ESP_OK;
   uint8_t i;

   i2c_init();

   //read current register configuration
   //from adg, else muting.
   for(i=0; i<UX_NUM; i++)
   {
    ret = read_adg(ux_addr[i], adg_registers.regs+i);
    if(ret != ESP_OK){ adg_registers.regs[i] = 0; }
   }    

   //load latest persistent value or write one inital
   ret = init_vol_state();
   
}

/* write the current volume setting to the nvs
 * TODO: check if i2c needs to be added here as well
 * */
esp_err_t pers_volume( void )
{
  esp_err_t ret = ESP_OK;

  ret = write_blob_nv(volume_handle, (&adg_registers)->regs, REG_NUM, vol_key);

  return ret;
}



