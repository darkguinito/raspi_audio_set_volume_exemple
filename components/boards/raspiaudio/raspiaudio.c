//
// RaspiAudio Muse Luxe
//


#include "RaspiAudio.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <esp_log.h>
#include <driver/i2c.h>
#include <esp_system.h>
#include "audio_hal.h"


#include "esp_types.h"

#include "audio_hal.h"
#include "board.h" 

// #define I2C_MASTER_NUM            I2C_NUM_0 /*!< I2C port number for master dev */
#define I2C_MASTER_NUM            0 // 16        /*!< I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE 0         /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0         /*!< I2C master do not need buffer */
#define I2C_MASTER_FREQ_HZ        250000    /*!< I2C master clock frequency */

#define WRITE_BIT                 I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                  I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN              0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS             0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                   0x0              /*!< I2C ack value */
#define NACK_VAL                  0x1              /*!< I2C nack value */

#define min(a,b) (((a) < (b)) ? (a) : (b))
#define max(a,b) (((a) > (b)) ? (a) : (b))

#define AC_ASSERT(a, format, b, ...) \
    if ((a) != 0) { \
        ESP_LOGE(TAG, format, ##__VA_ARGS__); \
        return b;\
    }
/////////////////////////////////////////////////////////////////
//*********************** NeoPixels  ***************************
////////////////////////////////////////////////////////////////
#define NUM_LEDS             1
#define LED_RMT_TX_CHANNEL   0
#define LED_RMT_TX_GPIO      22


#define BITS_PER_LED_CMD 24 
#define LED_BUFFER_ITEMS ((NUM_LEDS * BITS_PER_LED_CMD))

#define GREEN   0xFF0000
#define RED 	  0x00FF00
#define BLUE  	0x0000FF
#define WHITE   0xFFFFFF
#define YELLOW  0xE0F060



#define maxVol     64
#define M          32
//maxVol-66

struct led_state {
    uint32_t leds[NUM_LEDS];
};

void ws2812_control_init(void);
void ws2812_write_leds(struct led_state new_state);
static const char TAG[] = "es8388";	

static i2c_config_t i2c_cfg;
static int u8_volume = 0;
static int i2c_port;
uint8_t  vauxd;
uint8_t  vsdd;

#define ES8388_ADDR 0x20
#define GLED GPIO_NUM_22

static void battery(void *data);


audio_hal_func_t AUDIO_CODEC_RASPIAUDIO_DEFAULT_HANDLE = {
    .audio_codec_initialize = raspiaudio_init,
    .audio_codec_deinitialize = raspiaudio_deinit,
    .audio_codec_ctrl = raspiaudio_ctrl,
    .audio_codec_config_iface = raspiaudio_config_iface,
    .audio_codec_set_mute = raspiaudio_set_mute,
    .audio_codec_set_volume = raspiaudio_set_volume,
    .audio_codec_get_volume = raspiaudio_get_volume,
};

esp_err_t raspiaudio_deinit(void) {
  // TODO

	// AC101_Write_Reg(CHIP_AUDIO_RS, 0x123);		//soft reset
	i2c_driver_delete(i2c_port);
  return ESP_OK;
}

esp_err_t raspiaudio_ctrl(audio_hal_codec_mode_t mode,
                       audio_hal_ctrl_t ctrl_state) {
  // TODO
  return ESP_OK;
}

esp_err_t raspiaudio_config_iface(audio_hal_codec_mode_t       mode, 
                                  audio_hal_codec_i2s_iface_t *iface) {
  // TODO
	esp_err_t res = 0;
	int bits = 0;
	int fmat = 0;
	int sample = 0;
	uint16_t regval = 0;
  
	switch(iface->bits)						//0x10
	{
	// case AUDIO_HAL_BIT_LENGTH_8BITS:
	// 	bits = BIT_LENGTH_8_BITS;
	// 	break;
	case AUDIO_HAL_BIT_LENGTH_16BITS:
		bits = BIT_LENGTH_16_BITS;
		break;
	case AUDIO_HAL_BIT_LENGTH_24BITS:
		bits = BIT_LENGTH_24_BITS;
		break;
	default:
		bits = BIT_LENGTH_16_BITS;
	}

	switch(iface->fmt)						//0x10
	{
	case AUDIO_HAL_I2S_NORMAL:
		fmat = 0x0;
		break;
	case AUDIO_HAL_I2S_LEFT:
		fmat = 0x01;
		break;
	case AUDIO_HAL_I2S_RIGHT:
		fmat = 0x02;
		break;
	case AUDIO_HAL_I2S_DSP:
		fmat = 0x03;
		break;
	default:
		fmat = 0x00;
		break;
	}

	switch(iface->samples)
	{
	default:
		sample = SIMPLE_RATE_44100;
	}
	regval = ES8388_Read_Reg(I2S1LCK_CTRL);
	regval &= 0xffc3;
	regval |= (iface->mode << 15);
	regval |= (bits << 4);
	regval |= (fmat << 2);
	res |= ES8388_Write_Reg(I2S1LCK_CTRL, regval);
	res |= ES8388_Write_Reg(I2S_SR_CTRL, sample);
	return res;
}

esp_err_t raspiaudio_set_volume(int vol) {
  esp_err_t ret = ESP_OK;
  // printf("Raw VOLUME=============>%u\n",vol);
  // uint8_t value1 = 0;
  // uint8_t value2 = 0;
  // float   tmp    = 0;

  // ret |= ES8388_Write_Reg(25, 0x00);

  // tmp = ((float)vol / 100.0) * 64;
  // printf("Percentage VOLUME=============>%f\n",tmp);
  // if (tmp > M) {
  //   value1 = (uint8_t)tmp - M;
  // }
  // else {
  //   value2 = (uint8_t)((M - tmp) * 3);
  // }

  // if (vol == 0) {
  //   ret |= ES8388_Write_Reg(25, 0x04);
  // }
  // printf("Raw VOLUME=============>value1 %u  value2 %u\n",value1,value2);
  // ret |= ES8388_Write_Reg(26, value2);
  // ret |= ES8388_Write_Reg(27, value2);
  // ret |= ES8388_Write_Reg(46, value1);
  // ret |= ES8388_Write_Reg(47, value1);
  // u8_volume = vol;
  return ESP_OK;
}

esp_err_t raspiaudio_get_volume(int *vol) {
  esp_err_t ret = ESP_OK;
  *vol = u8_volume;
  return ret;
}

esp_err_t raspiaudio_set_mute(bool enable) {
  esp_err_t ret = ESP_OK;
  uint8_t nmute = (enable) ? 0 : 1 ; 
  gpio_set_level(MU, nmute);
  return ret;
}

esp_err_t ma120x0_get_mute(bool *enabled) {
  esp_err_t ret = ESP_OK;

  *enabled = false;  // TODO read from register
  return ret;
}

static i2c_config_t i2c_config = { 
    .mode             = I2C_MODE_MASTER,
    .sda_io_num       = GPIO_NUM_21,
    .sda_pullup_en    = GPIO_PULLDOWN_DISABLE,
    .scl_io_num       = GPIO_NUM_22,
    .scl_pullup_en    = GPIO_PULLDOWN_DISABLE,
    .master.clk_speed = I2C_MASTER_FREQ_HZ,
};

#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE   0   /*!< I2C master do not need buffer */

#define IIC_PORT	I2C_MASTER_NUM

static int i2c_init()
{
  esp_err_t ret = ESP_OK;
  ret = i2c_param_config(IIC_PORT, &i2c_config);
  ret |= i2c_driver_install(IIC_PORT, i2c_config.mode, I2C_EXAMPLE_MASTER_RX_BUF_DISABLE, I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
  AC_ASSERT(ret, "i2c_init error", -1);
  return ret;
}
    
esp_err_t raspiaudio_init(audio_hal_codec_config_t *codec_cfg) {	

  esp_err_t ret = ESP_OK;
	if(i2c_init()) return -1;
  // provides MCLK
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
  WRITE_PERI_REG(PIN_CTRL, READ_PERI_REG(PIN_CTRL)& 0xFFFFFFF0);

  // reset
  ret |= ES8388_Write_Reg(0, 0x80);
  ret |= ES8388_Write_Reg(0, 0x00);

  // mute
  ret |= ES8388_Write_Reg(25, 0x04);
  ret |= ES8388_Write_Reg(1, 0x50);

  //powerup
  ret |= ES8388_Write_Reg(2, 0x00);

  // slave mode
  ret |= ES8388_Write_Reg(8, 0x00);

  // DAC powerdown
  ret |= ES8388_Write_Reg(4, 0xC0);

  // vmidsel/500k ADC/DAC idem
  ret |= ES8388_Write_Reg(0, 0x12);
  ret |= ES8388_Write_Reg(1, 0x00);

  // i2s 16 bits
  ret |= ES8388_Write_Reg(23, 0x18);

  // sample freq 256
  ret |= ES8388_Write_Reg(24, 0x02);

  // LIN2/RIN2 for mixer
  ret |= ES8388_Write_Reg(38, 0x09);

  // left DAC to left mixer
  ret |= ES8388_Write_Reg(39, 0x90);

  // right DAC to right mixer
  ret |= ES8388_Write_Reg(42, 0x90);

  // DACLRC ADCLRC idem
  ret |= ES8388_Write_Reg(43, 0x80);
  ret |= ES8388_Write_Reg(45, 0x00);

  // DAC volume max
  ret |= ES8388_Write_Reg(27, 0x00);
  ret |= ES8388_Write_Reg(26, 0x00);
  ret |= ES8388_Write_Reg(2 , 0xF0);
  ret |= ES8388_Write_Reg(2 , 0x00);
  ret |= ES8388_Write_Reg(29, 0x1C);

  // DAC power-up LOUT1/ROUT1 enabled
  ret |= ES8388_Write_Reg(4, 0x30);

  // unmute
  ret |= ES8388_Write_Reg(25, 0x00);

//   // amp validation
//   gpio_set_level(PW, 1);

// 	if (ret != ESP_OK) {
// 		ESP_LOGW(TAG, "I2C write failed %d", ret);
// 	}	

//   gpio_reset_pin(PW);
//   gpio_set_direction(PW, GPIO_MODE_OUTPUT);
//   gpio_set_level(PW, 1);

//   //VP
//   gpio_reset_pin(VP);
//   gpio_set_direction(VP, GPIO_MODE_INPUT);
//   gpio_set_pull_mode(VP, GPIO_PULLUP_ONLY);
// //
//   ////VM
//   gpio_reset_pin(VM);
//   gpio_set_direction(VM, GPIO_MODE_INPUT);
//   gpio_set_pull_mode(VM, GPIO_PULLUP_ONLY);

//   ////MU
//   gpio_reset_pin(MU);
//   gpio_set_direction(MU, GPIO_MODE_INPUT);
//   gpio_set_pull_mode(MU, GPIO_PULLUP_ONLY);

//   // AUX detect
//   gpio_reset_pin(AUXD);
//   gpio_set_direction(AUXD, GPIO_MODE_INPUT);
//   // gpio_set_pull_mode(AUXD, GPIO_PULLUP_ONLY);

//   // SD detect
//   gpio_reset_pin(SDD);
//   gpio_set_direction(SDD, GPIO_MODE_INPUT);
//   gpio_set_pull_mode(SDD, GPIO_PULLUP_ONLY);

//   // pause
//   gpio_reset_pin(MU);
//   gpio_set_direction(MU, GPIO_MODE_INPUT);
//   gpio_set_pull_mode(MU, GPIO_PULLUP_ONLY);

//   // Store SD detect and AUX detect initial values
//   vsdd = gpio_get_level(SDD);
//   vauxd = gpio_get_level(AUXD);

  printf("-------------------->>>> init ES8388\n");
  //raspiaudio_set_volume(50);
	return (ret == ESP_OK);
}
// esp_err_t hal_i2c_master_mem_read(i2c_port_t i2c_num, uint8_t DevAddr,uint8_t MemAddr,uint8_t* data_rd, size_t size)
// {
// 	 if (size == 0) {
//         return ESP_OK;
//     }
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();//a cmd list
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, ( DevAddr << 1 ) | I2C_MASTER_WRITE, ACK_CHECK_EN);
//     i2c_master_write_byte(cmd, MemAddr, ACK_CHECK_EN);
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, ( DevAddr << 1 ) | I2C_MASTER_READ, ACK_CHECK_EN);
//     if (size > 1) {
//         i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
//     }
//     i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
//     i2c_master_stop(cmd);
//     esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
//     i2c_cmd_link_delete(cmd);
//     return ret;
// }

// esp_err_t hal_i2c_master_mem_write(i2c_port_t i2c_num, uint8_t DevAddr,uint8_t MemAddr,uint8_t* data_wr, size_t size)
// {
// 	 if (size == 0) {
//         return ESP_OK;
//     }
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();//a cmd list
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, ( DevAddr << 1 ) | I2C_MASTER_WRITE, ACK_CHECK_EN);
//     i2c_master_write_byte(cmd, MemAddr, ACK_CHECK_EN);
//     i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
//     i2c_master_stop(cmd);
//     esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
//     i2c_cmd_link_delete(cmd);
//     return ret;
// }

// void ES8388_Write_Reg(uint8_t reg, uint8_t val)
// {
//   uint8_t buf[2];
//   buf[0] = reg;
//   buf[1] = val;
//   hal_i2c_master_mem_write((i2c_port_t)0, ES8388_ADDR, buf[0], buf + 1, 1);
// }


esp_err_t ES8388_Write_Reg(uint8_t reg, uint16_t val)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  esp_err_t ret =0;
	uint8_t send_buff[4];

	send_buff[0] = (ES8388_ADDR << 1);
	send_buff[1] = reg;
	send_buff[2] = (val>>8) & 0xff;
	send_buff[3] = val & 0xff;

  ret |= i2c_master_start(cmd);
  ret |= i2c_master_write(cmd, send_buff, 4, ACK_CHECK_EN);
  ret |= i2c_master_stop(cmd);
  ret |= i2c_master_cmd_begin(IIC_PORT, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

uint16_t ES8388_Read_Reg(uint8_t reg) {
	uint16_t val = 0;
	uint8_t data_rd[2];
  uint8_t size = 2;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( ES8388_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( ES8388_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);		//check or not
  i2c_master_read(cmd, data_rd, size, ACK_VAL);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(IIC_PORT, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

	val=(data_rd[0]<<8)+data_rd[1];
	return val;
}
