#ifndef _RASPIAUDIO_H_
#define _RASPIAUDIO_H_

#include <stdint.h>
#include <esp_err.h>
#include "board.h"


#define AUXD         GPIO_NUM_27     // AUX In detect 27
#define GAIN         GPIO_NUM_23     //
#define MU           GPIO_NUM_12      // short => mute/unmute  long => stop (deep sleep)
#define PW           GPIO_NUM_21 
#define SDD          GPIO_NUM_34     // Sd detect
#define VM           GPIO_NUM_32     // Vol-
#define VP           GPIO_NUM_19     // Vol+ 

#define I2CN (i2c_port_t)0
#define I2S_SR_CTRL (i2s_port_t)0
#define I2S1LCK_CTRL (i2s_port_t)0
#define I2SR (i2s_port_t)0


typedef enum{
	SIMPLE_RATE_8000	= 0x0000,
	SIMPLE_RATE_11052	= 0x1000,
	SIMPLE_RATE_12000	= 0x2000,
	SIMPLE_RATE_16000	= 0x3000,
	SIMPLE_RATE_22050	= 0x4000,
	SIMPLE_RATE_24000	= 0x5000,
	SIMPLE_RATE_32000	= 0x6000,
	SIMPLE_RATE_44100	= 0x7000,
	SIMPLE_RATE_48000	= 0x8000,
	SIMPLE_RATE_96000	= 0x9000,
	SIMPLE_RATE_192000	= 0xa000,
}ac_adda_fs_i2s1_t;

typedef enum {
    BIT_LENGTH_8_BITS = 0x00,
    BIT_LENGTH_16_BITS = 0x01,
    BIT_LENGTH_20_BITS = 0x02,
    BIT_LENGTH_24_BITS = 0x03,
} ac_bits_length_t;

void setup_raspiaudio(void);
esp_err_t ES8388_Write_Reg(uint8_t reg, uint16_t val);
uint16_t ES8388_Read_Reg(uint8_t reg);

esp_err_t raspiaudio_init(audio_hal_codec_config_t *codec_cfg);
esp_err_t raspiaudio_deinit(void);
esp_err_t raspiaudio_set_volume(int vol);
esp_err_t raspiaudio_get_volume(int *value);
esp_err_t raspiaudio_set_mute(bool enable);
esp_err_t raspiaudio_get_mute(bool *enabled);
esp_err_t raspiaudio_ctrl(audio_hal_codec_mode_t, audio_hal_ctrl_t);
esp_err_t raspiaudio_config_iface(audio_hal_codec_mode_t , audio_hal_codec_i2s_iface_t *);

void i2c_master_init(void);

esp_err_t ma_write_byte(uint8_t i2c_addr, uint8_t prot, uint16_t address,
                        uint8_t value);
esp_err_t ma_write(uint8_t i2c_addr, uint8_t prot, uint16_t address,
                   uint8_t *wbuf, uint8_t n);

esp_err_t ma_read_byte(uint8_t i2c_addr, uint8_t prot, uint16_t address, uint8_t *data);
esp_err_t ma_read(uint8_t i2c_addr, uint8_t prot, uint16_t address,
                  uint8_t *rbuf, uint8_t n);

#endif /* _RASPIAUDIO_H_  */
