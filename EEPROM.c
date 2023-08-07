/*EEPROM.c*/

/*Includes*/
#include "EEPROM.h"
#include <stdlib.h>

/* Define the size of the sectors to be used */
#define PAGE_SIZE	((uint32_t)FLASH_PAGE_SIZE)  /* Page size */

/* Used Flash pages for EEPROM emulation */
enum	/* Page nb between PAGE0_BASE_ADDRESS & PAGE1_BASE_ADDRESS*/
{
	PAGE0 = ((uint16_t)0x0000),
	PAGE1 = ((uint16_t)0x0001)
};

/* No valid page define */
#define NO_VALID_PAGE         ((uint16_t)0x00AB)
#define MAX_DATA_STORED (PAGE_SIZE / 4 / 2)	//128 variables by default

/* Page status definitions */
enum
{
	ERASED = ((uint16_t)0xFFFF),	/* Page is empty */
	RECEIVE_DATA = ((uint16_t)0xEEEE),	/* Page is marked to receive data */
	VALID_PAGE = ((uint16_t)0x0000)	/* Page containing valid data */
};

//enum
//{
//	page_erased = ((uint16_t)0xFFFFFFFF),	/* Page is empty */
//	page_receive_data = ((uint16_t)0xFFFFEEEE),	/* Page is marked to receive data */
//	page_valid = ((uint16_t)0xEEEEEEEE)	/* Page containing valid data */
//};

//struct eeprom_handle
//{
//	void *active_page;
//	uint8_t free_space;
//	uint32_t page_0_address;
//	uint32_t page_1_address;
//};

/* Valid pages in read and write defines */
#define READ_FROM_VALID_PAGE  ((uint8_t)0x00)
#define WRITE_IN_VALID_PAGE   ((uint8_t)0x01)

/* Page full define */
#define PAGE_FULL             ((uint8_t)0x80)

#define EEPROM_EMULATED_SIZE	H_REG_COUNT /*+ SPECIAL_REG_ADDR_END*/ //128 //(H_REG_ADDR_END+1)  // -> 128 half words = 256 bytes
#define EEPROM_VIRTUAL_ADDRESS 	0xA001  // -> Start address of block of all virtual addresses
#define EEPROM_NO_WRITE_ADDRESS	0xA000  // -> Start address of block of all virtual addresses


uint32_t eeprom_start_address;
uint8_t free_space;	//TODO stop here 06.08.2023

/* Pages 0 and 1 base and end addresses */
uint32_t page_0_base_address;
uint32_t page_1_base_address;

//struct eeprom_handle eeprom_handle;
uint32_t active_page_address;

/*Extern variables*/
extern const struct structHRVA RegVirtAddr[H_REG_COUNT];

static HAL_StatusTypeDef Format(void);
static uint16_t Verify_Page_Full_Write_Variable(uint16_t VirtAddress, uint16_t Data);
static uint8_t Page_Transfer(void);
static uint8_t Verify_Page_Fully_Erased(uint32_t address, uint8_t *flg_erased);


/**
 * @brief  Restore the pages to a known good state in case of page's status
 *   corruption after a power loss.
 * @param  None.
 * @retval - Flash error code: on write Flash error
 *         - FLASH_COMPLETE: on success
 */
uint8_t EEL_Init(uint32_t start_address)
{
	uint16_t page_status_0, page_status_1;
	uint16_t varidx = 0;
	uint16_t data;
	uint32_t page_error = 0;
	FLASH_EraseInitTypeDef erase_init;
	uint8_t flg_erased;
	uint32_t valid_page_address, receive_page_address, erased_page_address;
	HAL_StatusTypeDef status = HAL_ERROR;


	eeprom_start_address = start_address;
	page_0_base_address = eeprom_start_address;
	page_1_base_address = eeprom_start_address + PAGE_SIZE;

	/* Unlock the Flash */
	while(status != HAL_OK)
	{
		status = HAL_FLASH_Unlock();
	}

	/* Get Page0 status */
	page_status_0 = (*(volatile uint16_t*)page_0_base_address);
	/* Get Page1 status */
	page_status_1 = (*(volatile uint16_t*)page_1_base_address);

	/* Fill EraseInit structure*/
	erase_init.TypeErase   = FLASH_TYPEERASE_PAGES;
	erase_init.NbPages     = 1;

	switch(page_status_0 ^ page_status_1)
	{
	case VALID_PAGE^ERASED:
	if(page_status_0 == VALID_PAGE)
	{
		valid_page_address = page_0_base_address;
		erased_page_address = page_1_base_address;
	}
	else
	{
		valid_page_address = page_1_base_address;
		erased_page_address = page_0_base_address;
	}

	Verify_Page_Fully_Erased(erased_page_address, &flg_erased);
	if(!flg_erased)
	{
		erase_init.PageAddress = erased_page_address;
		status = HAL_FLASHEx_Erase(&erase_init, &page_error);
		/* If erase operation was failed, a Flash error code is returned */
		if (status != HAL_OK)
		{
			return status;
		}
	}

	active_page_address = valid_page_address;
	break;

	case VALID_PAGE^RECEIVE_DATA:
	if(page_status_0 == VALID_PAGE)
	{
		valid_page_address = page_0_base_address;
		receive_page_address = page_1_base_address;
	}
	else
	{
		valid_page_address = page_1_base_address;
		receive_page_address = page_0_base_address;
	}

	/* Transfer data from Page1 to Page0 */	//TODO add checking for already transfered
	for (varidx = EEPROM_VIRTUAL_ADDRESS; varidx < EEPROM_VIRTUAL_ADDRESS+EEPROM_EMULATED_SIZE; varidx++)
	{
		/* Read the last variables' updates */
		status = EEL_Read_Variable(varidx, &data);
		/* In case variable corresponding to the virtual address was found */
		if (status == HAL_OK)
		{
			/* Transfer the variable to the Page0 */
			status = Verify_Page_Full_Write_Variable(varidx, data);
			/* If program operation was failed, a Flash error code is returned */
			if (status != HAL_OK)
			{
				return status;
			}
		}
	}
	/* Mark Page X as valid */
	status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, receive_page_address, VALID_PAGE);
	/* If program operation was failed, a Flash error code is returned */
	if (status != HAL_OK)
	{
		return status;
	}
	/* Erase Page X */
	Verify_Page_Fully_Erased(page_1_base_address, &flg_erased);
	if(!flg_erased)
	{
		erase_init.PageAddress = valid_page_address;
		status = HAL_FLASHEx_Erase(&erase_init, &page_error);
		/* If erase operation was failed, a Flash error code is returned */
		if (status != HAL_OK)
		{
			return status;
		}
	}
	active_page_address = receive_page_address;
	break;

	case RECEIVE_DATA^ERASED:
	if(page_status_0 == RECEIVE_DATA)
	{
		receive_page_address = page_0_base_address;
		erased_page_address = page_1_base_address;
	}
	else
	{
		receive_page_address = page_1_base_address;
		erased_page_address = page_0_base_address;
	}

	/* Erase Page1 */
	Verify_Page_Fully_Erased(erased_page_address, &flg_erased);
	if(!flg_erased)
	{
		erase_init.PageAddress = erased_page_address;
		status = HAL_FLASHEx_Erase(&erase_init, &page_error);
		/* If erase operation was failed, a Flash error code is returned */
		if (status != HAL_OK)
		{
			return status;
		}
	}
	/* Mark Page0 as valid */
	status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, receive_page_address, VALID_PAGE);
	/* If program operation was failed, a Flash error code is returned */
	if (status != HAL_OK)
	{
		return status;
	}

	active_page_address = receive_page_address;
	break;

	default:
		status = Format();
		/* If erase/program operation was failed, a Flash error code is returned */
		if (status != HAL_OK)
		{
			return status;
		}

		active_page_address = eeprom_start_address;
		break;
	}

	return HAL_OK;
}


/**
 * @brief  Verify if specified page is fully erased.
 * @param  Address: page address
 *   This parameter can be one of the following values:
 *     @arg PAGE0_BASE_ADDRESS: Page0 base address
 *     @arg PAGE1_BASE_ADDRESS: Page1 base address
 * @retval page fully erased status:
 *           - 0: if Page not erased
 *           - 1: if Page erased
 */
static uint8_t Verify_Page_Fully_Erased(uint32_t address, uint8_t *flg_erased)
{
	volatile uint32_t *ptr_addr_compare;
	uint8_t temp_flg_erased = 1;

	volatile uint32_t *ptr_address = (volatile uint32_t*)address;

	ptr_addr_compare = ptr_address + PAGE_SIZE/4;

	/* Check each active page address starting from end */
	for(;ptr_address<ptr_addr_compare; ptr_address+=4)
	{
		if(*ptr_address != 0xFFFFFFFFU)
		{
			temp_flg_erased = 0;
			break;
		}
	}

	*flg_erased = temp_flg_erased;
	return HAL_OK;
}

/**
 * @brief  Returns the last stored variable data, if found, which correspond to
 *   the passed virtual address
 * @param  VirtAddress: Variable virtual address
 * @param  Data: Global variable contains the read variable value
 * @retval Success or error status:
 *           - 0: if variable was found
 *           - 1: if the variable was not found
 *           - NO_VALID_PAGE: if no valid page was found.
 */
uint8_t EEL_Read_Variable(uint16_t VirtAddress, uint16_t *data)
{
	uint16_t addressvalue, value;
	uint32_t address = eeprom_start_address, PageStartAddress = eeprom_start_address;
	uint16_t result = 1;
	uint32_t word;

	/* Get the valid Page start Address */
	PageStartAddress = active_page_address;

	/* Get the valid Page end Address */
	address = active_page_address + PAGE_SIZE - 4;

	/* Check each active page address starting from end */
	while(address > PageStartAddress)
	{
		word = *(volatile uint32_t*)address;

		addressvalue = word>>16;

		if(addressvalue == VirtAddress)
		{
			value = word & 0xFFFF;
			*data = value;
			result = HAL_OK;
			break;
		}

		address -= 4;
	}

	/* Return readstatus value: (0: variable exist, 1: variable doesn't exist) */
	return result;
}

/**
 * @brief  Writes/upadtes variable data in EEPROM.
 * @param  VirtAddress: Variable virtual address
 * @param  Data: 16 bit data to be written
 * @retval Success or error status:
 *           - FLASH_COMPLETE: on success
 *           - PAGE_FULL: if valid page is full
 *           - NO_VALID_PAGE: if no valid page was found
 *           - Flash error code: on write Flash error
 */
uint8_t EEL_Write_Variable(uint16_t VirtAddress, uint16_t data)
{
	uint8_t status = 0;
	uint16_t old_data;

	if(VirtAddress != EEPROM_NO_WRITE_ADDRESS)
	{
		status = EEL_Read_Variable(VirtAddress, &old_data);
		if(data != old_data || status != 0)		//TODO optimize this workaround
		{
			/* In case the EEPROM active page is full */
			if (free_space == 0)
			{
				/* Perform Page transfer */
				status = Page_Transfer();
			}

			/* Write the variable virtual address and value in the EEPROM */
			status = Verify_Page_Full_Write_Variable(VirtAddress, data);
		}
	}
	/* Return last operation status */
	return status;
}

/**
 * @brief  Erases PAGE and PAGE1 and writes VALID_PAGE header to PAGE
 * @param  None
 * @retval Status of the last operation (Flash write or erase) done during
 *         EEPROM formating
 */
static HAL_StatusTypeDef Format(void)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint32_t page_error;
	FLASH_EraseInitTypeDef erase_init;
	uint8_t flg_erased;
	uint32_t page_address;

	for(uint8_t i=0; i<2; i++)
	{
		page_address = eeprom_start_address + PAGE_SIZE * i;
		/* Erase Page X */
		Verify_Page_Fully_Erased(page_address, &flg_erased);
		if(!flg_erased)
		{
			erase_init.TypeErase   = FLASH_TYPEERASE_PAGES;
			erase_init.PageAddress = page_address;
			erase_init.NbPages     = 1;

			status = HAL_FLASHEx_Erase(&erase_init, &page_error);
			/* If erase operation was failed, a Flash error code is returned */
			if (status != HAL_OK)
			{
				return status;
			}
		}
	}

	/* Set Page0 as valid page: Write VALID_PAGE at Page0 base address */
	status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, eeprom_start_address, VALID_PAGE);

	/* If program operation was failed, a Flash error code is returned */
	return status;
}

/**
 * @brief  Verify if active page is full and Writes variable in EEPROM.
 * @param  VirtAddress: 16 bit virtual address of the variable
 * @param  Data: 16 bit data to be written as variable value
 * @retval Success or error status:
 *           - FLASH_COMPLETE: on success
 *           - PAGE_FULL: if valid page is full
 *           - NO_VALID_PAGE: if no valid page was found
 *           - Flash error code: on write Flash error
 */
static uint16_t Verify_Page_Full_Write_Variable(uint16_t VirtAddress, uint16_t Data)
{
	HAL_StatusTypeDef status;
	uint32_t address, page_end_address;

	/* Get the valid Page start address */
	address = active_page_address;

	/* Get the valid Page end address */
	page_end_address = active_page_address + PAGE_SIZE;

	/* Check each active page address starting from beginning */
	while (address < page_end_address)
	{
		if ((*(volatile uint32_t*)address) == 0xFFFFFFFF)
		{
			status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, Data);
			status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address + 2, VirtAddress);
			free_space--;
			return status;
		}

		address += 4;
	}

	return HAL_OK;
}

/**
 * @brief  Transfers last updated variables data from the full Page to
 *   an empty one.
 * @param  VirtAddress: 16 bit virtual address of the variable
 * @param  Data: 16 bit data to be written as variable value
 * @retval Success or error status:
 *           - FLASH_COMPLETE: on success
 *           - PAGE_FULL: if valid page is full
 *           - NO_VALID_PAGE: if no valid page was found
 *           - Flash error code: on write Flash error
 */
static uint8_t Page_Transfer(void)	//TODO add checking for already transfered variables
{
	uint32_t new_page_address, old_page_address, current_address;
	uint32_t page_error = 0;
	FLASH_EraseInitTypeDef erase_init;
	HAL_StatusTypeDef status;
	uint8_t buf_data[256] = {0};	//'hash' table
	uint8_t virtual_address;

	if (active_page_address == page_0_base_address)
	{
		new_page_address = page_1_base_address;
		old_page_address = page_0_base_address;
	}
	else
	{
		new_page_address = page_0_base_address;
		old_page_address = page_1_base_address;
	}

	status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, new_page_address, RECEIVE_DATA);
	new_page_address +=4;
	if (status != HAL_OK)
	{
		return status;
	}

	current_address = old_page_address + PAGE_SIZE - 4;
	while(current_address > old_page_address)
	{
		virtual_address = *(volatile uint8_t*)current_address;

		if(buf_data[virtual_address] == 0)
		{
			buf_data[virtual_address] = 1;

			*(uint32_t*)new_page_address = *(uint32_t*)old_page_address;
		}

		current_address -= 4;
	}

	erase_init.TypeErase   = FLASH_TYPEERASE_PAGES;
	erase_init.PageAddress = old_page_address;
	erase_init.NbPages     = 1;

	status = HAL_FLASHEx_Erase(&erase_init, &page_error);
	if (status != HAL_OK)
	{
		return status;
	}

	status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, new_page_address, VALID_PAGE);
	if (status != HAL_OK)
	{
		return status;
	}

	active_page_address = new_page_address;
	return HAL_OK;
}
