// Header
#include "DebugActions.h"

// Include
#include "LowLevel.h"
#include "Board.h"
#include "DataTable.h"
#include "Delay.h"

// Functions
void DBGACT_Fan()
{
	LL_Fan(true);
	DELAY_US(1000000);
	LL_Fan(false);
}
//-----------------------------------------------

void DBGACT_LampGreen()
{
	LL_LampGreen(true);
	DELAY_US(500000);
	LL_LampGreen(false);
}
//-----------------------------------------------

void DBGACT_LampRed()
{
	LL_LampRed(true);
	DELAY_US(500000);
	LL_LampRed(false);
}
//-----------------------------------------------

void DBGACT_SwitchPC()
{
	LL_SwitchPC(true);
}
//-----------------------------------------------
