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
//void Audio_Player_Start(char * filename []);
void Audio_Player_Pause(void);
void Audio_Player_Resume(void);
void Audio_Player_Stop(void);
void Audio_Set_Volume(int num);
void Audio_Player_Start(void);

#endif /* INC_AUDIO_PLAYER_H_ */
