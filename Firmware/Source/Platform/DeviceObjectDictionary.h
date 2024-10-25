/** @file DeviceObjectDictionary.h
 * @brief Device object dictionary
*/

#ifndef __DEV_OBJ_DIC_H
#define __DEV_OBJ_DIC_H

// Commands
#define ACT_DIAG_INT_FAN			10	///< Проверка вентилятора
#define ACT_DIAG_GREEN_LED			11	///< Проверка зеленого индикатора
#define ACT_DIAG_RED_LED			12	///< Проверка красного индикатора
#define ACT_DIAG_PC_SWITCH			13	///< Включение ПК
//
#define ACT_BROADCAST_PING			100	///< Отправка широковещательного запроса в CAN сеть
#define ACT_SAVE_TO_ROM				200	///< Save parameters to EEPROM module
#define ACT_SAVE_TO_ROM				200	///< Сохранение пользовательских данных во FLASH процессора
#define ACT_RESTORE_FROM_ROM		201	///< Восстановление данных из FLASH
#define ACT_RESET_TO_DEFAULT		202	///< Сброс DataTable в состояние по умолчанию
//
#define ACT_BOOT_LOADER_REQUEST		320	///< Request reboot to bootloader

// Registers
#define REG_LAMP_GREEN				128	///< Управление зелёным индикатором
#define REG_LAMP_RED				129	///< Управление красным индикатором
#define REG_COMPATIBILITY_1			130	///< (Не используется, для совместимости)
#define REG_INT_FAN					131	///< Управление вентилятором
//
#define REG_DEV_STATE				192	///< Device state (Не используется, для совместимости)
#define REG_FAULT_REASON			193	///< Fault reason in the case DeviceState -> FAULT (Не используется, для совместимости)
#define REG_DISABLE_REASON			194	///< Fault reason in the case DeviceState -> DISABLED (Не используется, для совместимости)
#define REG_WARNING					195	///< Warning if present (Не используется, для совместимости)
//
#define REG_SENSOR_2				197	///< (Не используется, для совместимости)
#define REG_SENSOR_3				198	///< (Не используется, для совместимости)
#define REG_SENSOR_4				199	///< (Не используется, для совместимости)
#define REG_EXT_BUTTON				200	///< Состояние внешней кнопки
//
#define REG_OVERLAP_COUNT_REQ		210	///< Overlapping requests via SCCI interface
#define REG_OVERLAP_COUNT_RESP		211	///< Overlapping responses via SCCI interface
//
#define REG_MME_CODE				250	///< MME code number

#define REG_FWINFO_SLAVE_NID		256	///< Device CAN slave node ID
#define REG_FWINFO_MASTER_NID		257	///< Device CAN master node ID (if presented)
//
#define REG_FWINFO_STR_LEN			260	///< Length of the information string record
#define REG_FWINFO_STR_BEGIN		261	///< Begining of the information string record

// ENDPOINTS
//
#define EP16_CAN_Nodes				1

#endif // __DEV_OBJ_DIC_H
