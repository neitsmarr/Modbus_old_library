/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EEPROM_H
#define __EEPROM_H

/*Includes*/
#include "MODBUS.h"

uint8_t EEL_Init(uint32_t start_address);
uint8_t EEL_Read_Variable(uint16_t virtual_address, uint16_t *data);
uint8_t EEL_Write_Variable(uint16_t virtual_address, uint16_t data);

#endif /* __EEPROM_H */

