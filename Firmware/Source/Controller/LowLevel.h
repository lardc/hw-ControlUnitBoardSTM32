#ifndef __LOWLEVEL_H
#define __LOWLEVEL_H

// Include
#include "stdinc.h"

// Functions
void LL_ToggleBoardLED();
void LL_Fan(bool State);
void LL_LampGreen(bool State);
void LL_LampRed(bool State);
void LL_SwitchPC(bool IgnoreFlag);
bool LL_ExternalButton();

#endif //__LOWLEVEL_H
