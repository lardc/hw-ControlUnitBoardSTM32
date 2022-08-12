#ifndef __DC_VOLTAGE_BOARD_DICT_H
#define __DC_VOLTAGE_BOARD_DICT_H

// �������
#define DCV_ACT_START_PROCESS				100	// ������ �������� ������������ �����������
#define DCV_ACT_STOP_PROCESS				101	// ��������� �������� ������������ �����������

// ��������
#define DCV_REG_OUTPUT_LINE					128	// ����� ������ ��������� ����������
#define DCV_REG_OUTPUT_TYPE					129	// ��� ������
#define DCV_REG_OUTPUT_MODE					130	// ����� ������
#define DCV_REG_PULSE_LENGTH				131	// ������������ ��������� ��������

#define DCV_REG_CURRENT_SETPOINT			135	// �������� ������� ����
#define DCV_REG_CURRENT_SETPOINT_32			136
#define DCV_REG_VOLTAGE_SETPOINT			137	// �������� ������� ����������
#define DCV_REG_VOLTAGE_SETPOINT_32			138

#define DCV_REG_CURRENT_RESULT				200	// �������� ��������� ����
#define DCV_REG_CURRENT_RESULT_32			201
#define DCV_REG_VOLTAGE_RESULT				202	// �������� ��������� ����������
#define DCV_REG_VOLTAGE_RESULT_32			203

#define DCV_REG_VOLTAGE_READY				205	// ���� ������ �� ������� ����������

typedef enum __DCV_OutputLine
{
	DCV_BUS_LV = 1,
	DCV_CTRL = 2,
	DCV_PS1 = 3,
	DCV_PS2 = 4
} DCV_OutputLine;

typedef enum __DCV_OutputType
{
	DCV_Voltage = 1,
	DCV_Current = 2
} DCV_OutputType;

typedef enum __DCV_OutputMode
{
	DCV_Pulse = 1,
	DCV_Continuous = 2
} DCV_OutputMode;

#endif // __DC_VOLTAGE_BOARD_DICT_H
