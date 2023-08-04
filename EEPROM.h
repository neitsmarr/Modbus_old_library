/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EEPROM_H
#define __EEPROM_H

/*Includes*/
#include "MODBUS.h"

/* Base address of the Flash sectors */
#ifdef __STM32F031x6_H
#define ADDR_FLASH_PAGE_15    ((uint32_t)0x08007800) /* Base @ of Page 30, 1 Kbyte */
#define ADDR_FLASH_PAGE_16    ((uint32_t)0x08007C00) /* Base @ of Page 31, 1 Kbyte */
#endif

#ifdef __STM32F051x8_H
#define ADDR_FLASH_PAGE_15    ((uint32_t)0x0800F800) /* Base @ of Page 62, 1 Kbyte */
#define ADDR_FLASH_PAGE_16    ((uint32_t)0x0800FC00) /* Base @ of Page 63, 1 Kbyte */
#endif

/* Define the size of the sectors to be used */
#define PAGE_SIZE               (uint32_t)FLASH_PAGE_SIZE  /* Page size */

/* EEPROM start address in Flash */
#define EEPROM_START_ADDRESS  ((uint32_t)ADDR_FLASH_PAGE_15) /* EEPROM emulation start address */

/* Pages 0 and 1 base and end addresses */
#define PAGE0_BASE_ADDRESS    ((uint32_t)(EEPROM_START_ADDRESS + 0x0000))
#define PAGE0_END_ADDRESS     ((uint32_t)(EEPROM_START_ADDRESS + (PAGE_SIZE - 1)))

#define PAGE1_BASE_ADDRESS    ((uint32_t)(EEPROM_START_ADDRESS + PAGE_SIZE))
#define PAGE1_END_ADDRESS     ((uint32_t)(EEPROM_START_ADDRESS + (2*PAGE_SIZE - 1)))

/* Used Flash pages for EEPROM emulation */
#define PAGE0                 ((uint16_t)0x0000)
#define PAGE1                 ((uint16_t)0x0001) /* Page nb between PAGE0_BASE_ADDRESS & PAGE1_BASE_ADDRESS*/

/* No valid page define */
#define NO_VALID_PAGE         ((uint16_t)0x00AB)

/* Page status definitions */
#define ERASED                ((uint16_t)0xFFFF)     /* Page is empty */
#define RECEIVE_DATA          ((uint16_t)0xEEEE)     /* Page is marked to receive data */
#define VALID_PAGE            ((uint16_t)0x0000)     /* Page containing valid data */

/* Valid pages in read and write defines */
#define READ_FROM_VALID_PAGE  ((uint8_t)0x00)
#define WRITE_IN_VALID_PAGE   ((uint8_t)0x01)

/* Page full define */
#define PAGE_FULL             ((uint8_t)0x80)

/* Variables' number */
#define NB_OF_VAR             ((uint8_t)0x14)

#ifndef EEPROM_EMULATED_SIZE
#define EEPROM_EMULATED_SIZE	H_REG_COUNT /*+ SPECIAL_REG_ADDR_END*/ //128 //(H_REG_ADDR_END+1)  // -> 128 half words = 256 bytes
#endif

#ifndef EEPROM_VIRTUAL_ADDRESS
#define EEPROM_VIRTUAL_ADDRESS 	0xA001  // -> Start address of block of all virtual addresses
#endif

#ifndef EEPROM_NO_WRITE_ADDRESS
#define EEPROM_NO_WRITE_ADDRESS 			0xA000  // -> Start address of block of all virtual addresses
#endif


#define	EE_NO_VALID_PAGE		4

uint16_t EE_Init(void);
uint16_t EE_ReadVariable(uint16_t VirtAddress);
uint16_t EE_WriteVariable(uint16_t VirtAddress, uint16_t Data);
uint16_t EE_FindValidPage(uint8_t Operation);

#endif /* __EEPROM_H */

