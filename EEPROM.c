/*EEPROM.c*/

/*Includes*/
#include "EEPROM.h"

/* Define the size of the sectors to be used */
#define PAGE_SIZE	((uint32_t)FLASH_PAGE_SIZE)  /* Page size */

/* Page status definitions */
enum page_status_t
{
	page_status_erased		= 1<<0,
	page_status_receive		= 1<<1,
	page_status_active		= 1<<2,
	page_status_invalid		= 1<<3
};

//struct eeprom_handle
//{
//	void *active_page;
//	uint8_t free_space;
//	uint32_t page_0_address;
//	uint32_t page_1_address;
//};

uint32_t eeprom_start_address;
uint32_t page_0_start_address;
uint32_t page_1_start_address;
uint32_t active_page_address;
uint32_t active_page_free_space;	//in records (32 bit)


//struct eeprom_handle eeprom_handle;


static HAL_StatusTypeDef Format(void);
static HAL_StatusTypeDef Add_Record(uint8_t identifier, uint16_t data);
static HAL_StatusTypeDef Page_Transfer(void);
static HAL_StatusTypeDef Calculate_Free_Space(uint32_t page_address, uint32_t *free_space);
static HAL_StatusTypeDef Set_Page_Status(uint32_t page_address, uint8_t status);
static HAL_StatusTypeDef Get_Page_Status(uint32_t page_address, uint8_t *status);
static HAL_StatusTypeDef Calculate_CRC(uint8_t *data, uint8_t length);

/**
 * @brief  Restore the pages to a known good state in case of page's status
 *   corruption after a power loss.
 * @param  None.
 * @retval - Flash error code: on write Flash error
 *         - FLASH_COMPLETE: on success
 */
uint8_t FEE_Init(uint32_t start_address)
{
	uint8_t page_status_0, page_status_1;
	uint32_t page_error = 0;
	FLASH_EraseInitTypeDef erase_init;
	uint32_t free_space;
	uint32_t valid_page_address, receive_page_address, erased_page_address;
	HAL_StatusTypeDef status = HAL_ERROR;

	eeprom_start_address = start_address;
	page_0_start_address = eeprom_start_address;
	page_1_start_address = eeprom_start_address + PAGE_SIZE;

	while(status != HAL_OK)
	{
		status = HAL_FLASH_Unlock();
	}

	Get_Page_Status(page_0_start_address, &page_status_0);
	Get_Page_Status(page_1_start_address, &page_status_1);


	erase_init.TypeErase   = FLASH_TYPEERASE_PAGES;
	erase_init.NbPages     = 1;

	switch(page_status_0 & page_status_1)
	{
	case page_status_active | page_status_erased:
	if(page_status_0 == page_status_active)
	{
		valid_page_address = page_0_start_address;
		erased_page_address = page_1_start_address;
	}
	else
	{
		valid_page_address = page_1_start_address;
		erased_page_address = page_0_start_address;
	}

	Calculate_Free_Space(erased_page_address, &free_space);	//stop here 07.08.2023 check calculation of free_space

	if(free_space != PAGE_SIZE / 4)
	{
		erase_init.PageAddress = erased_page_address;
		status = HAL_FLASHEx_Erase(&erase_init, &page_error);
		if (status != HAL_OK)
		{
			return status;
		}
	}

	active_page_address = valid_page_address;
	break;

	case page_status_active | page_status_receive:
	if(page_status_0 == page_status_active)
	{
		valid_page_address = page_0_start_address;
		receive_page_address = page_1_start_address;
	}
	else
	{
		valid_page_address = page_1_start_address;
		receive_page_address = page_0_start_address;
	}

	active_page_address = valid_page_address;
	Page_Transfer();

	active_page_address = receive_page_address;
	break;

	case page_status_receive | page_status_erased:
	if(page_status_0 == page_status_receive)
	{
		receive_page_address = page_0_start_address;
		erased_page_address = page_1_start_address;
	}
	else
	{
		receive_page_address = page_1_start_address;
		erased_page_address = page_0_start_address;
	}

	Calculate_Free_Space(erased_page_address, &free_space);

	if(free_space != PAGE_SIZE/4)
	{
		erase_init.PageAddress = erased_page_address;
		status = HAL_FLASHEx_Erase(&erase_init, &page_error);
		if (status != HAL_OK)
		{
			return status;
		}
	}

	Set_Page_Status(receive_page_address, page_status_active);
	if (status != HAL_OK)
	{
		return status;
	}

	active_page_address = receive_page_address;
	break;

	default:
		status = Format();
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
uint8_t FEE_Read_Data(uint8_t identifier, uint16_t *data)
{
	uint8_t addressvalue;
	uint16_t value;
	uint32_t address, PageStartAddress;
	uint16_t result = 1;
	uint32_t word;

	PageStartAddress = active_page_address;
	address = active_page_address + PAGE_SIZE - 4;

	while(address > PageStartAddress)
	{
		word = *(volatile uint32_t*)address;

		addressvalue = (word>>8) & 0xFF;

		if(addressvalue == identifier)
		{
			value = word>>16;
			*data = value;
			result = HAL_OK;
			break;
		}

		address -= 4;
	}

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
uint8_t FEE_Write_Data(uint8_t identifier, uint16_t data)
{
	enum {empty_record = 0xFF};

	uint8_t status = 0;
	uint16_t old_data;

	if(identifier != empty_record)
	{
		status = FEE_Read_Data(identifier, &old_data);

		if(data != old_data || status != 0)		//TODO optimize this workaround
		{
			if (active_page_free_space == 0)
			{
				status = Page_Transfer();
			}

			status = Add_Record(identifier, data);
		}
	}

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

		Calculate_Free_Space(page_address, &free_space);

		if(free_space != PAGE_SIZE/4)
		{
			erase_init.TypeErase   = FLASH_TYPEERASE_PAGES;
			erase_init.PageAddress = page_address;
			erase_init.NbPages     = 1;

			status = HAL_FLASHEx_Erase(&erase_init, &page_error);
			if (status != HAL_OK)
			{
				return status;
			}
		}
	}

	status = Set_Page_Status(eeprom_start_address, page_status_active);

	active_page_free_space = PAGE_SIZE / 4 - 1;

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
static HAL_StatusTypeDef Add_Record(uint8_t identifier, uint16_t data)
{
	HAL_StatusTypeDef status;
	uint32_t address, page_end_address;
	uint16_t shifted_identifier;

	shifted_identifier = (uint16_t)identifier<<8;

	address = active_page_address;

	page_end_address = active_page_address + PAGE_SIZE;

	while (address < page_end_address)
	{
		if ((*(volatile uint32_t*)address) == 0xFFFFFFFF)
		{
			status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, shifted_identifier);
			status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address + 2, data);
			active_page_free_space--;
			return status;
		}

		address += 4;
	}

	return HAL_OK;
}

static HAL_StatusTypeDef Get_Record(uint8_t identifier, uint16_t *data)
{
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
static HAL_StatusTypeDef Page_Transfer(void)	//TODO add checking for already transfered variables
{
	uint32_t new_page_address, old_page_address, from_address, to_address;
	uint32_t page_error = 0;
	FLASH_EraseInitTypeDef erase_init;
	HAL_StatusTypeDef status;
	uint8_t buf_data[256] = {0};	//'hash' table
	uint8_t identifier;
	uint32_t word;
	uint8_t page_status;

	if (active_page_address == page_0_start_address)
	{
		new_page_address = page_1_start_address;
		old_page_address = page_0_start_address;
	}
	else
	{
		new_page_address = page_0_start_address;
		old_page_address = page_1_start_address;
	}

	Get_Page_Status(new_page_address, &page_status);
	if(page_status != page_status_receive)
	{
		status = Set_Page_Status(new_page_address, page_status_receive);

		if (status != HAL_OK)
		{
			return status;
		}
	}

	to_address = new_page_address + 4;

	from_address = old_page_address + PAGE_SIZE - 4;
	while(from_address > old_page_address)
	{
		identifier = *((volatile uint8_t*)from_address+1);

		if(buf_data[identifier] == 0)
		{
			buf_data[identifier] = 1;

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

	status = Set_Page_Status(new_page_address, page_status_active);
	if (status != HAL_OK)
	{
		return status;
	}

	active_page_address = new_page_address;
	Calculate_Free_Space(new_page_address, &active_page_free_space);

	return HAL_OK;
}

static HAL_StatusTypeDef Calculate_Free_Space(uint32_t page_address, uint32_t *free_space)
{
	uint32_t address;

	address = page_address;

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

static HAL_StatusTypeDef Set_Page_Status(uint32_t page_address, uint8_t status)
{
	switch(status)
	{
	case page_status_receive:
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, page_address, 0x0000);
		break;
	case page_status_active:
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, page_address+2, 0x0000);
		break;
	}

	return HAL_OK;
}

static HAL_StatusTypeDef Get_Page_Status(uint32_t page_address, uint8_t *status)
{
	uint32_t word;

	word = *(volatile uint32_t*)page_address;

	switch(word)
	{
	case 0xFFFFFFFF:
		*status = page_status_erased;
		break;
	case 0xFFFF0000:
		*status = page_status_receive;
		break;
	case 0x00000000:
		*status = page_status_active;
		break;
	}

	return HAL_OK;
}

static HAL_StatusTypeDef Calculate_CRC(uint8_t *data, uint8_t length)
{
	return HAL_OK;
}

