// ----------------------------------------
// Device profile
// ----------------------------------------

// Header
#include "DeviceProfile.h"
//
// Includes
#include "SysConfig.h"
#include "Global.h"
#include "DataTable.h"
#include "Controller.h"
#include "Constraints.h"
#include "ZwNCAN.h"
#include "ZwUSART.h"

// Types
//
typedef struct __FEPState
{
	Int16U Size;
	Int16U ReadCounter;
	Int16U WriteCounter;
	Int16U LastReadCounter;
	pInt16U pDataCounter;
	float* Data;
} FEPState, *pFEPState;
//
typedef struct __EPState
{
	Int16U Size;
	Int16U ReadCounter;
	Int16U WriteCounter;
	Int16U LastReadCounter;
	pInt16U pDataCounter;
	pInt16U Data;
} EPState, *pEPState;
//
typedef struct __EPStates
{
	EPState EPs[EP_COUNT];
	EPState WriteEPs[EP_WRITE_COUNT];
	FEPState FEPs[FEP_COUNT];
} EPStates, *pEPStates;

// Variables
//
SCCI_Interface DEVICE_USB_UART1_Interface, DEVICE_USB_UART2_Interface;
BCCI_Interface DEVICE_CAN_Interface;
BCCIM_Interface MASTER_DEVICE_CAN_Interface;
//
static SCCI_IOConfig USB_UART1_IOConfig, USB_UART2_IOConfig;
static BCCI_IOConfig CAN_IOConfig;
static xCCI_ServiceConfig X_ServiceConfig;
static EPStates DummyEPState, USB_UART1_EPState, USB_UART2_EPState, CAN_EPState;
static xCCI_FUNC_CallbackAction ControllerDispatchFunction;
//
static Boolean* MaskChangesFlag;

// Forward functions
//
static Boolean DEVPROFILE_Validate16(Int16U Address, Int16U Data);
static Boolean DEVPROFILE_ValidateFloat(Int16U Address, float Data, float* LowLimit, float* HighLimit);
static Boolean DEVPROFILE_DispatchAction(Int16U ActionID, pInt16U UserError);
static void DEVPROFILE_FillWRPartDefault();

// Functions
//
void DEVPROFILE_Init(xCCI_FUNC_CallbackAction SpecializedDispatch, Boolean* MaskChanges)
{
	// Save values
	ControllerDispatchFunction = SpecializedDispatch;
	MaskChangesFlag = MaskChanges;
	
	// Init interface
	USB_UART1_IOConfig.IO_SendArray16 = &USART1_SendArray16;
	USB_UART1_IOConfig.IO_ReceiveArray16 = &USART1_ReceiveArray16;
	USB_UART1_IOConfig.IO_GetBytesToReceive = &USART1_GetBytesToReceive;
	USB_UART1_IOConfig.IO_ReceiveByte = &USART1_ReceiveChar;

	USB_UART2_IOConfig.IO_SendArray16 = &USART2_SendArray16;
	USB_UART2_IOConfig.IO_ReceiveArray16 = &USART2_ReceiveArray16;
	USB_UART2_IOConfig.IO_GetBytesToReceive = &USART2_GetBytesToReceive;
	USB_UART2_IOConfig.IO_ReceiveByte = &USART2_ReceiveChar;

	CAN_IOConfig.IO_SendMessage = &NCAN_SendMessage;
	CAN_IOConfig.IO_SendMessageEx = &NCAN_SendMessageEx;
	CAN_IOConfig.IO_GetMessage = &NCAN_GetMessage;
	CAN_IOConfig.IO_IsMessageReceived = &NCAN_IsMessageReceived;
	CAN_IOConfig.IO_ConfigMailbox = &NCAN_ConfigMailbox;
	
	// Init service
	X_ServiceConfig.UserActionCallback = &DEVPROFILE_DispatchAction;
	X_ServiceConfig.ValidateCallback16 = &DEVPROFILE_Validate16;
	X_ServiceConfig.ValidateCallbackFloat = &DEVPROFILE_ValidateFloat;
	
	// Init interface driver
	SCCI_Init(&DEVICE_USB_UART1_Interface, &USB_UART1_IOConfig, &X_ServiceConfig, (pInt16U)DataTable, DATA_TABLE_SIZE,
			xCCI_TIMEOUT_TICKS, &DummyEPState);
	SCCI_Init(&DEVICE_USB_UART2_Interface, &USB_UART2_IOConfig, &X_ServiceConfig, (pInt16U)DataTable, DATA_TABLE_SIZE,
			xCCI_TIMEOUT_TICKS, &DummyEPState);
	BCCI_Init(&DEVICE_CAN_Interface, &CAN_IOConfig, &X_ServiceConfig, (pInt16U)DataTable, DATA_TABLE_SIZE, &DummyEPState);
	BCCIM_Init(&MASTER_DEVICE_CAN_Interface, &CAN_IOConfig, xCCI_TIMEOUT_TICKS, &CONTROL_TimeCounter);

	// Set write protection
	SCCI_AddProtectedArea(&DEVICE_USB_UART1_Interface, DATA_TABLE_WP_START, DATA_TABLE_SIZE - 1);
	SCCI_AddProtectedArea(&DEVICE_USB_UART2_Interface, DATA_TABLE_WP_START, DATA_TABLE_SIZE - 1);
	BCCI_AddProtectedArea(&DEVICE_CAN_Interface, DATA_TABLE_WP_START, DATA_TABLE_SIZE - 1);
}
// ----------------------------------------

void DEVPROFILE_InitEPService(pInt16U Indexes, pInt16U Sizes, pInt16U* Counters, pInt16U* Datas)
{
	Int16U i;

	for(i = 0; i < EP_COUNT; ++i)
	{
		USB_UART1_EPState.EPs[i].Size = Sizes[i];
		USB_UART1_EPState.EPs[i].pDataCounter = Counters[i];
		USB_UART1_EPState.EPs[i].Data = Datas[i];

		USB_UART2_EPState.EPs[i].Size = Sizes[i];
		USB_UART2_EPState.EPs[i].pDataCounter = Counters[i];
		USB_UART2_EPState.EPs[i].Data = Datas[i];

		CAN_EPState.EPs[i].Size = Sizes[i];
		CAN_EPState.EPs[i].pDataCounter = Counters[i];
		CAN_EPState.EPs[i].Data = Datas[i];

		USB_UART1_EPState.EPs[i].ReadCounter = USB_UART1_EPState.EPs[i].LastReadCounter = 0;
		USB_UART2_EPState.EPs[i].ReadCounter = USB_UART2_EPState.EPs[i].LastReadCounter = 0;
		CAN_EPState.EPs[i].ReadCounter = CAN_EPState.EPs[i].LastReadCounter = 0;

		SCCI_RegisterReadEndpoint16(&DEVICE_USB_UART1_Interface, Indexes[i], &DEVPROFILE_CallbackReadX);
		SCCI_RegisterReadEndpoint16(&DEVICE_USB_UART2_Interface, Indexes[i], &DEVPROFILE_CallbackReadX);
		BCCI_RegisterReadEndpoint16(&DEVICE_CAN_Interface, Indexes[i], &DEVPROFILE_CallbackReadX);
	}
}
// ----------------------------------------

Int16U DEVPROFILE_CallbackReadX(Int16U Endpoint, pInt16U* Buffer, Boolean Streamed, Boolean RepeatLastTransmission,
		void* EPStateAddress, Int16U MaxNonStreamSize)
{
	Int16U pLen;
	pEPState epState;
	pEPStates epStates = (pEPStates)EPStateAddress;

	// Validate pointer
	if(!epStates)
		return 0;

	// Get endpoint
	epState = &epStates->EPs[Endpoint - 1];

	// Handle transmission repeat
	if(RepeatLastTransmission)
		epState->ReadCounter = epState->LastReadCounter;

	// Write possible content reference
	*Buffer = epState->Data + epState->ReadCounter;

	// Calculate content length
	if(*(epState->pDataCounter) < epState->ReadCounter)
		pLen = 0;
	else
		pLen = *(epState->pDataCounter) - epState->ReadCounter;

	if(!Streamed)
		pLen = (pLen > MaxNonStreamSize) ? MaxNonStreamSize : pLen;

	// Update content state
	epState->LastReadCounter = epState->ReadCounter;
	epState->ReadCounter += pLen;

	return pLen;
}
// ----------------------------------------

void DEVPROFILE_ProcessRequests()
{
	// Handle interface requests
	SCCI_Process(&DEVICE_USB_UART1_Interface, CONTROL_TimeCounter, *MaskChangesFlag);
	SCCI_Process(&DEVICE_USB_UART2_Interface, CONTROL_TimeCounter, *MaskChangesFlag);

	// Handle interface requests
	BCCI_Process(&DEVICE_CAN_Interface, *MaskChangesFlag);
}
// ----------------------------------------

void DEVPROFILE_ResetEPReadState()
{
	Int16U i;

	for(i = 0; i < EP_COUNT; ++i)
	{
		USB_UART1_EPState.EPs[i].ReadCounter = 0;
		USB_UART2_EPState.EPs[i].ReadCounter = 0;
		CAN_EPState.EPs[i].ReadCounter = 0;
		USB_UART1_EPState.EPs[i].LastReadCounter = 0;
		USB_UART2_EPState.EPs[i].LastReadCounter = 0;
		CAN_EPState.EPs[i].LastReadCounter = 0;
	}

	for(i = 0; i < FEP_COUNT; ++i)
	{
		USB_UART1_EPState.FEPs[i].ReadCounter = 0;
		USB_UART2_EPState.FEPs[i].ReadCounter = 0;
		CAN_EPState.FEPs[i].ReadCounter = 0;
		USB_UART1_EPState.FEPs[i].LastReadCounter = 0;
		USB_UART2_EPState.FEPs[i].LastReadCounter = 0;
		CAN_EPState.FEPs[i].LastReadCounter = 0;
	}
}
// ----------------------------------------

void DEVPROFILE_ResetScopes(Int16U ResetPosition)
{
	Int16U i;

	for(i = 0; i < EP_COUNT; ++i)
	{
		*(USB_UART1_EPState.EPs[i].pDataCounter) = ResetPosition;
		*(USB_UART2_EPState.EPs[i].pDataCounter) = ResetPosition;
		*(CAN_EPState.EPs[i].pDataCounter) = ResetPosition;

		MemZero16(USB_UART1_EPState.EPs[i].Data, USB_UART1_EPState.EPs[i].Size);
		MemZero16(USB_UART2_EPState.EPs[i].Data, USB_UART2_EPState.EPs[i].Size);
		MemZero16(CAN_EPState.EPs[i].Data, CAN_EPState.EPs[i].Size);
	}

	for(i = 0; i < FEP_COUNT; ++i)
	{
		*(USB_UART1_EPState.FEPs[i].pDataCounter) = ResetPosition;
		*(USB_UART2_EPState.FEPs[i].pDataCounter) = ResetPosition;
		*(CAN_EPState.FEPs[i].pDataCounter) = ResetPosition;

		MemZero16((pInt16U)USB_UART1_EPState.FEPs[i].Data, USB_UART1_EPState.FEPs[i].Size * 2);
		MemZero16((pInt16U)USB_UART2_EPState.FEPs[i].Data, USB_UART2_EPState.FEPs[i].Size * 2);
		MemZero16((pInt16U)CAN_EPState.FEPs[i].Data, CAN_EPState.FEPs[i].Size * 2);
	}
}
// ----------------------------------------

void DEVPROFILE_ResetControlSection()
{
	DT_ResetWRPart(&DEVPROFILE_FillWRPartDefault);
}
// ----------------------------------------

static void DEVPROFILE_FillWRPartDefault()
{
	for(Int16U i = DATA_TABLE_WR_START; i < DATA_TABLE_WP_START; ++i)
		DataTable[i] = Constraint[i].Default;
}
// ----------------------------------------

void DEVPROFILE_FillNVPartDefault(void)
{
	for(Int16U i = 0; i < DATA_TABLE_NV_SIZE; ++i)
		DataTable[i] = Constraint[i].Default;
}
// ----------------------------------------

static Boolean DEVPROFILE_Validate16(Int16U Address, Int16U Data)
{
	if(Address < DATA_TABLE_WP_START)
		return (Constraint[Address].Min <= Data) && (Data <= Constraint[Address].Max);
	else
		return FALSE;
}
// ----------------------------------------

static Boolean DEVPROFILE_ValidateFloat(Int16U Address, float Data, float* LowLimit, float* HighLimit)
{
	if(LowLimit && HighLimit)
	{
		*LowLimit = Constraint[Address].Min;
		*HighLimit = Constraint[Address].Max;

		return TRUE;
	}
	else if(Address < DATA_TABLE_WP_START)
		return (Constraint[Address].Min <= Data) && (Data <= Constraint[Address].Max);
	else
		return FALSE;
}
// ----------------------------------------

static Boolean DEVPROFILE_DispatchAction(Int16U ActionID, pInt16U UserError)
{
	switch (ActionID)
	{
		case ACT_SAVE_TO_ROM:
			DT_SaveNVPartToEPROM();
			break;

		case ACT_RESTORE_FROM_ROM:
			DT_RestoreNVPartFromEPROM();
			break;

		case ACT_RESET_TO_DEFAULT:
			DT_ResetNVPart(&DEVPROFILE_FillNVPartDefault);
			break;

		case ACT_BOOT_LOADER_REQUEST:
			BOOT_LOADER_VARIABLE = BOOT_LOADER_REQUEST;
			break;

		default:
			return (ControllerDispatchFunction) ? ControllerDispatchFunction(ActionID, UserError) : FALSE;
	}
	
	return TRUE;
}
// ----------------------------------------
