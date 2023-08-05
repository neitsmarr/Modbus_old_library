/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EEPROM_H
#define __EEPROM_H

/*Includes*/
#include "MODBUS.h"

uint8_t EE_Init(uint32_t start_address);
uint8_t EE_ReadVariable(uint16_t VirtAddress, uint16_t *data);
uint8_t EE_WriteVariable(uint16_t VirtAddress, uint16_t Data);

#endif /* __EEPROM_H */

