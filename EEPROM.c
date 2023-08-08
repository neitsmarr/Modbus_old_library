/*EEPROM.c*/

/*Includes*/
#include "EEPROM.h"

/* Define the size of the sectors to be used */
#define PAGE_SIZE	((uint32_t)FLASH_PAGE_SIZE)  /* Page size */

/* Page status definitions */
enum
{
	ERASED = ((uint16_t)0xFFFF),	/* Page is empty */
	RECEIVE_DATA = ((uint16_t)0xEEEE),	/* Page is marked to receive data */
	VALID_PAGE = ((uint16_t)0x0000)	/* Page containing valid data */
};

enum page_status_t
{
	page_status_erased,	// = ((uint16_t)0xFFFFFFFF),	/* Page is empty */
	page_status_receive,	// = ((uint16_t)0xFFFFEEEE),	/* Page is marked to receive data */
	page_status_valid	// = ((uint16_t)0xEEEEEEEE)	/* Page containing valid data */
};

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

#define EEPROM_NO_WRITE_ADDRESS	0xFF

uint32_t eeprom_start_address;
uint32_t active_page_free_space;	//in records (32 bit)

/* Pages 0 and 1 base and end addresses */
uint32_t page_0_base_address;
uint32_t page_1_base_address;

//struct eeprom_handle eeprom_handle;
uint32_t active_page_address;

static HAL_StatusTypeDef Format(void);
static uint16_t _Write_Variable(uint8_t virtual_address, uint16_t data);
static uint8_t Page_Transfer(void);
static uint8_t Calculate_Free_Space(uint32_t page_address, uint32_t *free_space);
static uint8_t Set_Page_Status(uint32_t page_address, uint8_t status);
static uint8_t Get_Page_Status(uint32_t page_address, uint8_t *status);

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
	uint32_t page_error = 0;
	FLASH_EraseInitTypeDef erase_init;
	uint32_t free_space;
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

	Calculate_Free_Space(erased_page_address, &free_space);	//stop here 07.08.2023 check calculation of free_space

	if(free_space != PAGE_SIZE / 4)
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

	active_page_address = valid_page_address;
	/* Transfer data from Page to Page */	//TODO add checking for already transfered
	Page_Transfer();

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

	Calculate_Free_Space(erased_page_address, &free_space);

	if(free_space != PAGE_SIZE/4)
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

	Calculate_Free_Space(active_page_address, &active_page_free_space);

	if (active_page_free_space == 0)
	{
		Page_Transfer();
	}


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
uint8_t EEL_Read_Variable(uint8_t virtual_address, uint16_t *data)
{
	uint8_t addressvalue;
	uint16_t value;
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

		addressvalue = word & 0xFF;

		if(addressvalue == virtual_address)
		{
			value = word>>16;
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
uint8_t EEL_Write_Variable(uint8_t virtual_address, uint16_t data)
{
	uint8_t status = 0;
	uint16_t old_data;

	if(virtual_address != EEPROM_NO_WRITE_ADDRESS)
	{
		status = EEL_Read_Variable(virtual_address, &old_data);

		if(data != old_data || status != 0)		//TODO optimize this workaround
		{
			/* In case the EEPROM active page is full */
			if (active_page_free_space == 0)
			{
				/* Perform Page transfer */
				status = Page_Transfer();
			}

			/* Write the variable virtual address and value in the EEPROM */
			status = _Write_Variable(virtual_address, data);
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
	uint32_t page_address;
	uint32_t free_space;

	for(uint8_t i=0; i<2; i++)
	{
		page_address = eeprom_start_address + PAGE_SIZE * i;
		/* Erase Page X */
		Calculate_Free_Space(page_address, &free_space);

//		Verify_Page_Fully_Erased(page_address, &flg_erased);

		if(free_space != PAGE_SIZE/4)
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

	active_page_free_space = PAGE_SIZE / 4 - 1;

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
static uint16_t _Write_Variable(uint8_t virtual_address, uint16_t data)
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
			status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, virtual_address);
			status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address + 2, data);
			active_page_free_space--;
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
	uint32_t new_page_address, old_page_address, from_address, to_address;
	uint32_t page_error = 0;
	FLASH_EraseInitTypeDef erase_init;
	HAL_StatusTypeDef status;
	uint8_t buf_data[256] = {0};	//'hash' table
	uint8_t virtual_address;
	uint32_t word;

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

	if(*(uint16_t*)new_page_address != RECEIVE_DATA)
	{
		status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, new_page_address, RECEIVE_DATA);

		if (status != HAL_OK)
		{
			return status;
		}
	}

	to_address = new_page_address + 4;

	from_address = old_page_address + PAGE_SIZE - 4;
	while(from_address > old_page_address)
	{
		virtual_address = *(volatile uint8_t*)from_address;

		if(buf_data[virtual_address] == 0)
		{
			buf_data[virtual_address] = 1;

			word = *(volatile uint32_t*)from_address;

			HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, to_address, word&0xFFFF);
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, to_address+2, word>>16);

			to_address += 4;
		}

		from_address -= 4;
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
	Calculate_Free_Space(new_page_address, &active_page_free_space);

	return HAL_OK;
}

static uint8_t Calculate_Free_Space(uint32_t page_address, uint32_t *free_space)
{
	uint32_t address;

	/* Get the valid Page start address */
	address = page_address;

	/* Check each active page address starting from beginning */
	while (address < page_address+PAGE_SIZE)
	{
		if ((*(volatile uint32_t*)address) == 0xFFFFFFFF)
		{
			break;
		}

		address += 4;
	}

	*free_space = (PAGE_SIZE - (address - page_address)) / 4;

	return HAL_OK;
}

static uint8_t Set_Page_Status(uint32_t page_address, uint8_t status)
{
	switch(status)
	{
	case page_status_receive:
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, page_address, 0xEEEE);
		break;
	case page_status_valid:
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, page_address+2, 0xEEEE);
		break;
	}

	return HAL_OK;
}

static uint8_t Get_Page_Status(uint32_t page_address, uint8_t *status)
{
	uint32_t word;

	word = *(volatile uint32_t*)page_address;

	switch(word)
	{
	case 0xFFFFFFFF:
		*status = page_status_erased;
		break;
	case 0xFFFFEEEE:
		*status = page_status_receive;
		break;
	case 0xEEEEEEEE:
		*status = page_status_valid;
		break;
	}

	return HAL_OK;
}

