// ----------------------------------------
// Logic controller
// ----------------------------------------

/**
 * @file Controller.h
 * @brief Файл Controller.h
 * @details Детали:
*/ 

#ifndef __CONTROLLER_H
#define __CONTROLLER_H

// Include
#include "stdinc.h"

// Variables
extern volatile Int64U CONTROL_TimeCounter;

// Functions
/**
 * @fn void CONTROL_Init()
 * @brief Функция инициализации
*/
void CONTROL_Init();
void CONTROL_Idle();

#endif // __CONTROLLER_H
