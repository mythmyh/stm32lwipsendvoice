/*
 * audio_player.h
 *
 *  Created on: Jun 9, 2020
 *      Author: admin
 */

#ifndef INC_AUDIO_PLAYER_H_
#define INC_AUDIO_PLAYER_H_

#include "main.h"

void Audio_Player_Init(void);
void Audio_Player_Start(void);
void Audio_Player_Pause(void);
void Audio_Player_Resume(void);
void Audio_Player_Stop(void);

void WM8978_HPvol_Set(uint8_t voll,uint8_t volr);
void WM8978_SPKvol_Set(uint8_t volx);
void WM8978_SPKvol_Set(uint8_t volx);
void WM8978_ADDA_Cfg(uint8_t dacen,uint8_t adcen);
void WM8978_Input_Cfg(uint8_t micen,uint8_t lineinen,uint8_t auxen);
void WM8978_Output_Cfg(uint8_t dacen,uint8_t bpsen);
void WM8978_MIC_Gain(uint8_t gain);
void WM8978_I2S_Cfg(uint8_t fmt,uint8_t len);
#endif /* INC_AUDIO_PLAYER_H_ */
