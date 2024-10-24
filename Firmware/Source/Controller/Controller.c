﻿// ----------------------------------------
// Controller logic
// ----------------------------------------

// Header
#include "Controller.h"
//
// Includes
#include "SysConfig.h"
#include "Board.h"
#include "DeviceObjectDictionary.h"
#include "DataTable.h"
#include "DeviceProfile.h"
#include "Interrupts.h"
#include "BCCIxParams.h"
#include "DebugActions.h"
#include "LowLevel.h"

// Variables
volatile Int64U CONTROL_TimeCounter = 0;
Boolean CycleActive = false;

// Forward functions
Boolean CONTROL_DispatchAction(Int16U ActionID, pInt16U UserError);
void CONTROL_Idle();
void CONTROL_UpdateWatchDog();
void CONTROL_Init();

// Functions
void CONTROL_Init()
{
	// Init data table
	EPROMServiceConfig EPROMService = {(FUNC_EPROM_WriteValues)&NFLASH_WriteDT, (FUNC_EPROM_ReadValues)&NFLASH_ReadDT};
	DT_Init(EPROMService, false);
	DT_SaveFirmwareInfo(CAN_SLAVE_NID, CAN_MASTER_NID);
	
	// Device profile initialization
	DEVPROFILE_Init(&CONTROL_DispatchAction, &CycleActive);
	DEVPROFILE_ResetControlSection();
	DataTable[REG_MME_CODE] = MME_CODE;
}
// ----------------------------------------

void CONTROL_Idle()
{
	// Управление индикаторами индикатором
	LL_LampGreen(DataTable[REG_LAMP_GREEN]);
	LL_LampRed(DataTable[REG_LAMP_RED]);

	// Управление вентилятором
	LL_Fan(DataTable[REG_INT_FAN]);

	// Состояние внешней кнопки
	DataTable[REG_EXT_BUTTON] = LL_ExternalButton() ? 0 : 1;

	DEVPROFILE_ProcessRequests();
	CONTROL_UpdateWatchDog();
}
// ----------------------------------------

Boolean CONTROL_DispatchAction(Int16U ActionID, pInt16U UserError)
{
	switch(ActionID)
	{
		case ACT_DIAG_INT_FAN:
			DBGACT_Fan();
			break;

		case ACT_DIAG_GREEN_LED:
			DBGACT_LampGreen();
			break;

		case ACT_DIAG_RED_LED:
			DBGACT_LampRed();
			break;

		case ACT_DIAG_PC_SWITCH:
			DBGACT_SwitchPC();
			break;

		default:
			return false;
	}

	return true;
}
// ----------------------------------------

void CONTROL_UpdateWatchDog()
{
	if(BOOT_LOADER_VARIABLE != BOOT_LOADER_REQUEST)
		IWDG_Refresh();
}
// ----------------------------------------
