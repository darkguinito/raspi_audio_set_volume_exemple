#ifndef _RASPIAUDIO_H_
#define _RASPIAUDIO_H_

#include <stdint.h>
#include <esp_err.h>
#include "board.h"


#define VP           GPIO_NUM_19     // Vol+ 
#define GAIN         GPIO_NUM_23     //
#define AUXD         GPIO_NUM_27     // AUX In detect 27
#define VM           GPIO_NUM_32     // Vol-
#define SDD          GPIO_NUM_34     // Sd detect

void setup_raspiaudio(void);
void ES8388_Write_Reg(uint8_t reg, uint8_t val);

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
