#include "Controller.h"
#include "InitConfig.h"
#include "SysConfig.h"

int main()
{
	__disable_irq();
	SCB->VTOR = (uint32_t)BOOT_LOADER_MAIN_PR_ADDR;
	__enable_irq();
	
	// ��������� ��������� ������� ������������
	INITCFG_ConfigSystemClock();
	
	// ��������� ������
	INITCFG_ConfigIO();
	
	// ��������� CAN
	INITCFG_ConfigCAN();
	
	// ��������� UART
	INITCFG_ConfigUART();
	
	// ��������� ���������� ��������
	INITCFG_ConfigTimer7();
	
	// ��������� ����������� �������
	INITCFG_ConfigWatchDog();
	
	// ������� ����� ������������� 2
	INITCFG_Sync2Proxy();

	// ������������� ������ �����������
	CONTROL_Init();
	
	// ������� ����
	while(TRUE)
		CONTROL_Idle();
	
	return 0;
}