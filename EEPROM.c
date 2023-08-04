/*EEPROM.c*/

/*Includes*/
#include "EEPROM.h"


/*Extern variables*/
extern const struct structHRVA RegVirtAddr[H_REG_COUNT];


static HAL_StatusTypeDef EE_Format(void);
//uint16_t EE_FindValidPage(uint8_t Operation);
static uint16_t EE_VerifyPageFullWriteVariable(uint16_t VirtAddress, uint16_t Data);
static uint16_t EE_PageTransfer(uint16_t VirtAddress, uint16_t Data);
static uint16_t EE_VerifyPageFullyErased(uint32_t Address);

/**
  * @brief  Restore the pages to a known good state in case of page's status
  *   corruption after a power loss.
  * @param  None.
  * @retval - Flash error code: on write Flash error
  *         - FLASH_COMPLETE: on success
  */
uint16_t EE_Init(void)
{
  uint16_t pagestatus0 = 6, pagestatus1 = 6;
  uint16_t varidx = 0;
  uint16_t eepromstatus = 0, readstatus = 0;
  HAL_StatusTypeDef  flashstatus;
  uint32_t page_error = 0;
  FLASH_EraseInitTypeDef s_eraseinit;

  /* Unlock the Flash */
  HAL_StatusTypeDef flash_ok = HAL_ERROR;
  while(flash_ok != HAL_OK)
  {
	  flash_ok = HAL_FLASH_Unlock();
  }

  /* Get Page0 status */
  pagestatus0 = (*(__IO uint16_t*)PAGE0_BASE_ADDRESS);
  /* Get Page1 status */
  pagestatus1 = (*(__IO uint16_t*)PAGE1_BASE_ADDRESS);

  /* Fill EraseInit structure*/
  s_eraseinit.TypeErase   = FLASH_TYPEERASE_PAGES;
  s_eraseinit.PageAddress = PAGE0_BASE_ADDRESS;
  s_eraseinit.NbPages     = 1;

  /* Check for invalid header states and repair if necessary */
  switch (pagestatus0)
  {
	case ERASED:
	  if (pagestatus1 == VALID_PAGE) /* Page0 erased, Page1 valid */
	  {
		  /* Erase Page0 */
		if(!EE_VerifyPageFullyErased(PAGE0_BASE_ADDRESS))
		{
		  flashstatus = HAL_FLASHEx_Erase(&s_eraseinit, &page_error);
		  /* If erase operation was failed, a Flash error code is returned */
		  if (flashstatus != HAL_OK)
		  {
			return flashstatus;
		  }
		}
	  }
	  else if (pagestatus1 == RECEIVE_DATA) /* Page0 erased, Page1 receive */
	  {
		/* Erase Page0 */
		if(!EE_VerifyPageFullyErased(PAGE0_BASE_ADDRESS))
		{
		  flashstatus = HAL_FLASHEx_Erase(&s_eraseinit, &page_error);
		  /* If erase operation was failed, a Flash error code is returned */
		  if (flashstatus != HAL_OK)
		  {
			return flashstatus;
		  }
		}
		/* Mark Page1 as valid */
		flashstatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, PAGE1_BASE_ADDRESS, VALID_PAGE);
		/* If program operation was failed, a Flash error code is returned */
		if (flashstatus != HAL_OK)
		{
		  return flashstatus;
		}
	  }
	  else /* First EEPROM access (Page0&1 are erased) or invalid state -> format EEPROM */
	  {
		/* Erase both Page0 and Page1 and set Page0 as valid page */
		flashstatus = EE_Format();
		/* If erase/program operation was failed, a Flash error code is returned */
		if (flashstatus != HAL_OK)
		{
		  return flashstatus;
		}
	  }
	  break;

	case RECEIVE_DATA:
	  if (pagestatus1 == VALID_PAGE) /* Page0 receive, Page1 valid */
	  {
		  /* Transfer data from Page1 to Page0 */
		  for (varidx = EEPROM_VIRTUAL_ADDRESS; varidx < EEPROM_VIRTUAL_ADDRESS+EEPROM_EMULATED_SIZE; varidx++)
		  {
			  /* Read the last variables' updates */
			  readstatus = EE_ReadVariable(varidx);
			  /* In case variable corresponding to the virtual address was found */
			  if (readstatus >= 0)
			  {
				/* Transfer the variable to the Page0 */
				eepromstatus = EE_VerifyPageFullWriteVariable(varidx, readstatus);
				/* If program operation was failed, a Flash error code is returned */
				if (eepromstatus != HAL_OK)
				{
				  return eepromstatus;
				}
			  }
		  }
		/* Mark Page0 as valid */
		flashstatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, PAGE0_BASE_ADDRESS, VALID_PAGE);
		/* If program operation was failed, a Flash error code is returned */
		if (flashstatus != HAL_OK)
		{
		  return flashstatus;
		}
		s_eraseinit.TypeErase   = FLASH_TYPEERASE_PAGES;
		s_eraseinit.PageAddress = PAGE1_BASE_ADDRESS;
		s_eraseinit.NbPages     = 1;
		/* Erase Page1 */
		if(!EE_VerifyPageFullyErased(PAGE1_BASE_ADDRESS))
		{
		  flashstatus = HAL_FLASHEx_Erase(&s_eraseinit, &page_error);
		  /* If erase operation was failed, a Flash error code is returned */
		  if (flashstatus != HAL_OK)
		  {
			return flashstatus;
		  }
		}
	  }
	  else if (pagestatus1 == ERASED) /* Page0 receive, Page1 erased */
	  {
		s_eraseinit.TypeErase   = FLASH_TYPEERASE_PAGES;
		s_eraseinit.PageAddress = PAGE1_BASE_ADDRESS;
		s_eraseinit.NbPages     = 1;
		/* Erase Page1 */
		if(!EE_VerifyPageFullyErased(PAGE1_BASE_ADDRESS))
		{
		  flashstatus = HAL_FLASHEx_Erase(&s_eraseinit, &page_error);
		  /* If erase operation was failed, a Flash error code is returned */
		  if (flashstatus != HAL_OK)
		  {
			return flashstatus;
		  }
		}
		/* Mark Page0 as valid */
		flashstatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, PAGE0_BASE_ADDRESS, VALID_PAGE);
		/* If program operation was failed, a Flash error code is returned */
		if (flashstatus != HAL_OK)
		{
		  return flashstatus;
		}
	  }
	  else /* Invalid state -> format eeprom */
	  {
		/* Erase both Page0 and Page1 and set Page0 as valid page */
		flashstatus = EE_Format();
		/* If erase/program operation was failed, a Flash error code is returned */
		if (flashstatus != HAL_OK)
		{
		  return flashstatus;
		}
	  }
	  break;

	case VALID_PAGE:
	  if (pagestatus1 == VALID_PAGE) /* Invalid state -> format eeprom */
	  {
		/* Erase both Page0 and Page1 and set Page0 as valid page */
		flashstatus = EE_Format();
		/* If erase/program operation was failed, a Flash error code is returned */
		if (flashstatus != HAL_OK)
		{
		  return flashstatus;
		}
	  }
	  else if (pagestatus1 == ERASED) /* Page0 valid, Page1 erased */
	  {
		s_eraseinit.TypeErase   = FLASH_TYPEERASE_PAGES;
		s_eraseinit.PageAddress = PAGE1_BASE_ADDRESS;
		s_eraseinit.NbPages     = 1;
		/* Erase Page1 */
		if(!EE_VerifyPageFullyErased(PAGE1_BASE_ADDRESS))
		{
		  flashstatus = HAL_FLASHEx_Erase(&s_eraseinit, &page_error);
		  /* If erase operation was failed, a Flash error code is returned */
		  if (flashstatus != HAL_OK)
		  {
			return flashstatus;
		  }
		}
	  }
	  else /* Page0 valid, Page1 receive */
	  {
		  /* Transfer data from Page0 to Page1 */
		  for (varidx = EEPROM_VIRTUAL_ADDRESS; varidx < EEPROM_VIRTUAL_ADDRESS+EEPROM_EMULATED_SIZE; varidx++)
		  {
			  /* Read the last variables' updates */
			  readstatus = EE_ReadVariable(varidx);
			  /* In case variable corresponding to the virtual address was found */
			  if (readstatus >= 0)
			  {
				/* Transfer the variable to the Page1 */
				eepromstatus = EE_VerifyPageFullWriteVariable(varidx, readstatus);
				/* If program operation was failed, a Flash error code is returned */
				if (eepromstatus != HAL_OK)
				{
				  return eepromstatus;
				}
			  }
		  }
		/* Mark Page1 as valid */
		flashstatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, PAGE1_BASE_ADDRESS, VALID_PAGE);
		/* If program operation was failed, a Flash error code is returned */
		if (flashstatus != HAL_OK)
		{
		  return flashstatus;
		}
		s_eraseinit.TypeErase   = FLASH_TYPEERASE_PAGES;
		s_eraseinit.PageAddress = PAGE0_BASE_ADDRESS;
		s_eraseinit.NbPages     = 1;
		/* Erase Page0 */
		if(!EE_VerifyPageFullyErased(PAGE0_BASE_ADDRESS))
		{
		  flashstatus = HAL_FLASHEx_Erase(&s_eraseinit, &page_error);
		  /* If erase operation was failed, a Flash error code is returned */
		  if (flashstatus != HAL_OK)
		  {
			return flashstatus;
		  }
		}
	  }
	  break;

	default:  /* Any other state -> format eeprom */
	  /* Erase both Page0 and Page1 and set Page0 as valid page */
	  flashstatus = EE_Format();
	  /* If erase/program operation was failed, a Flash error code is returned */
	  if (flashstatus != HAL_OK)
	  {
		return flashstatus;
	  }
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
uint16_t EE_VerifyPageFullyErased(uint32_t Address)
{
	uint32_t readstatus = 1;
	uint16_t addressvalue = 0x5555;
	uint32_t addr_compare = 0;

	if (Address == PAGE0_BASE_ADDRESS)
	{
	  addr_compare = PAGE0_END_ADDRESS;
	} else if (Address == PAGE1_BASE_ADDRESS)
	{
	  addr_compare = PAGE1_END_ADDRESS;
	}
	/* Check each active page address starting from end */
	while (Address <= addr_compare)
  {
	/* Get the current location content to be compared with virtual address */
	addressvalue = (*(__IO uint16_t*)Address);

	/* Compare the read address with the virtual address */
	if (addressvalue != ERASED)
	{

	  /* In case variable value is read, reset readstatus flag */
	  readstatus = 0;

	  break;
	}
	/* Next address location */
	Address = Address + 4;
  }

  /* Return readstatus value: (0: Page not erased, 1: Page erased) */
  return readstatus;
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
uint16_t EE_ReadVariable(uint16_t VirtAddress)
{
 uint16_t validpage = PAGE0;
  uint16_t addressvalue = 0x5555;
  uint32_t address = EEPROM_START_ADDRESS, PageStartAddress = EEPROM_START_ADDRESS;
  uint16_t Data = 0;

  /* Get active Page for read operation */
  validpage = EE_FindValidPage(READ_FROM_VALID_PAGE);

  /* Check if there is no valid page */
  if (validpage == NO_VALID_PAGE)
  {
	//return  NO_VALID_PAGE;
	//  error_code = EE_NO_VALID_PAGE;
  }

  /* Get the valid Page start Address */
  PageStartAddress = (uint32_t)(EEPROM_START_ADDRESS + (uint32_t)(validpage * PAGE_SIZE));

  /* Get the valid Page end Address */
  address = (uint32_t)((EEPROM_START_ADDRESS - 2) + (uint32_t)((1 + validpage) * PAGE_SIZE));

  if(VirtAddress == EEPROM_NO_WRITE_ADDRESS)
  {
	  Data = 0;
  }
  else
  {
	/* Check each active page address starting from end */
	while (address > (PageStartAddress + 2))
	{
		/* Get the current location content to be compared with virtual address */
		addressvalue = (*(__IO uint16_t*)address);
		/* Compare the read address with the virtual address */
		if (addressvalue == VirtAddress)
		{
			/* Get content of Address-2 which is variable value */
			Data = (*(__IO uint16_t*)(address - 2));
			break;
		}
		else
		{
			/* Next address location */
			address = address - 4;
		}
	}
  }

  /* Return readstatus value: (0: variable exist, 1: variable doesn't exist) */
  return Data;
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
uint16_t EE_WriteVariable(uint16_t VirtAddress, uint16_t Data)
{
	uint16_t Status = 0;

	if(VirtAddress != EEPROM_NO_WRITE_ADDRESS)
	{
	/* Write the variable virtual address and value in the EEPROM */
	Status = EE_VerifyPageFullWriteVariable(VirtAddress, Data);

	/* In case the EEPROM active page is full */
	if (Status == PAGE_FULL)
	{
	/* Perform Page transfer */
	Status = EE_PageTransfer(VirtAddress, Data);
	}
	}
	/* Return last operation status */
	return Status;
}

/**
  * @brief  Erases PAGE and PAGE1 and writes VALID_PAGE header to PAGE
  * @param  None
  * @retval Status of the last operation (Flash write or erase) done during
  *         EEPROM formating
  */
static HAL_StatusTypeDef EE_Format(void)
{
	HAL_StatusTypeDef flashstatus = HAL_OK;
	uint32_t page_error = 0;
	FLASH_EraseInitTypeDef s_eraseinit;

  s_eraseinit.TypeErase   = FLASH_TYPEERASE_PAGES;
  s_eraseinit.PageAddress = PAGE0_BASE_ADDRESS;
  s_eraseinit.NbPages     = 1;
  /* Erase Page0 */
  if(!EE_VerifyPageFullyErased(PAGE0_BASE_ADDRESS))
  {
	flashstatus = HAL_FLASHEx_Erase(&s_eraseinit, &page_error);
	/* If erase operation was failed, a Flash error code is returned */
	if (flashstatus != HAL_OK)
	{
	  return flashstatus;
	}
  }
  /* Set Page0 as valid page: Write VALID_PAGE at Page0 base address */
  flashstatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, PAGE0_BASE_ADDRESS, VALID_PAGE);
  /* If program operation was failed, a Flash error code is returned */
  if (flashstatus != HAL_OK)
  {
	return flashstatus;
  }

  s_eraseinit.PageAddress = PAGE1_BASE_ADDRESS;
  /* Erase Page1 */
  if(!EE_VerifyPageFullyErased(PAGE1_BASE_ADDRESS))
  {
	flashstatus = HAL_FLASHEx_Erase(&s_eraseinit, &page_error);
	/* If erase operation was failed, a Flash error code is returned */
	if (flashstatus != HAL_OK)
	{
	  return flashstatus;
	}
  }

  return HAL_OK;
}

/**
  * @brief  Find valid Page for write or read operation
  * @param  Operation: operation to achieve on the valid page.
  *   This parameter can be one of the following values:
  *     @arg READ_FROM_VALID_PAGE: read operation from valid page
  *     @arg WRITE_IN_VALID_PAGE: write operation from valid page
  * @retval Valid page number (PAGE or PAGE1) or NO_VALID_PAGE in case
  *   of no valid page was found
  */
//static uint16_t EE_FindValidPage(uint8_t Operation)
uint16_t EE_FindValidPage(uint8_t Operation)
{
 	uint16_t pagestatus0 = 6, pagestatus1 = 6;

  /* Get Page0 actual status */
  pagestatus0 = (*(__IO uint16_t*)PAGE0_BASE_ADDRESS);

  /* Get Page1 actual status */
  pagestatus1 = (*(__IO uint16_t*)PAGE1_BASE_ADDRESS);

  /* Write or read operation */
  switch (Operation)
  {
	case WRITE_IN_VALID_PAGE:   /* ---- Write operation ---- */
	  if (pagestatus1 == VALID_PAGE)
	  {
		/* Page0 receiving data */
		if (pagestatus0 == RECEIVE_DATA)
		{
		  return PAGE0;         /* Page0 valid */
		}
		else
		{
		  return PAGE1;         /* Page1 valid */
		}
	  }
	  else if (pagestatus0 == VALID_PAGE)
	  {
		/* Page1 receiving data */
		if (pagestatus1 == RECEIVE_DATA)
		{
		  return PAGE1;         /* Page1 valid */
		}
		else
		{
		  return PAGE0;         /* Page0 valid */
		}
	  }
	  else
	  {
		return NO_VALID_PAGE;   /* No valid Page */
	  }

	case READ_FROM_VALID_PAGE:  /* ---- Read operation ---- */
	  if (pagestatus0 == VALID_PAGE)
	  {
		return PAGE0;           /* Page0 valid */
	  }
	  else if (pagestatus1 == VALID_PAGE)
	  {
		return PAGE1;           /* Page1 valid */
	  }
	  else
	  {
		return NO_VALID_PAGE ;  /* No valid Page */
	  }

	default:
	  return PAGE0;             /* Page0 valid */
  }
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
static uint16_t EE_VerifyPageFullWriteVariable(uint16_t VirtAddress, uint16_t Data)
{
 HAL_StatusTypeDef flashstatus = HAL_OK;
  uint16_t validpage = PAGE0;
  uint32_t address = EEPROM_START_ADDRESS, pageendaddress = EEPROM_START_ADDRESS+PAGE_SIZE;

  /* Get valid Page for write operation */
  validpage = EE_FindValidPage(WRITE_IN_VALID_PAGE);

  /* Check if there is no valid page */
  if (validpage == NO_VALID_PAGE)
  {
	return  NO_VALID_PAGE;
  }

  /* Get the valid Page start address */
  address = (uint32_t)(EEPROM_START_ADDRESS + (uint32_t)(validpage * PAGE_SIZE));

  /* Get the valid Page end address */
  pageendaddress = (uint32_t)((EEPROM_START_ADDRESS - 1) + (uint32_t)((validpage + 1) * PAGE_SIZE));

  /* Check each active page address starting from begining */
  while (address < pageendaddress)
  {
	/* Verify if address and address+2 contents are 0xFFFFFFFF */
	if ((*(__IO uint32_t*)address) == 0xFFFFFFFF)
	{
	  /* Set variable data */
	  flashstatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, Data);
	  /* If program operation was failed, a Flash error code is returned */
	  if (flashstatus != HAL_OK)
	  {
		return flashstatus;
	  }
	  /* Set variable virtual address */
	  flashstatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address + 2, VirtAddress);
	  /* Return program operation status */
	  return flashstatus;
	}
	else
	{
	  /* Next address location */
	  address = address + 4;
	}
  }

  /* Return PAGE_FULL in case the valid page is full */
  return PAGE_FULL;
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
static uint16_t EE_PageTransfer(uint16_t VirtAddress, uint16_t Data)
{
	HAL_StatusTypeDef flashstatus = HAL_OK;
	uint32_t newpageaddress = EEPROM_START_ADDRESS;
	uint32_t oldpageid = 0;
	uint16_t validpage = PAGE0; // varidx = 0;
	uint16_t eepromstatus = 0, readstatus = 0;
	uint32_t page_error = 0;
	FLASH_EraseInitTypeDef s_eraseinit;

  int32_t i;
  /* Get active Page for read operation */
  validpage = EE_FindValidPage(READ_FROM_VALID_PAGE);

  if (validpage == PAGE1)       /* Page1 valid */
  {
	/* New page address where variable will be moved to */
	newpageaddress = PAGE0_BASE_ADDRESS;

	/* Old page ID where variable will be taken from */
	oldpageid = PAGE1_BASE_ADDRESS;
  }
  else if (validpage == PAGE0)  /* Page0 valid */
  {
	/* New page address  where variable will be moved to */
	newpageaddress = PAGE1_BASE_ADDRESS;

	/* Old page ID where variable will be taken from */
	oldpageid = PAGE0_BASE_ADDRESS;
  }
  else
  {
	return NO_VALID_PAGE;       /* No valid Page */
  }

  /* Set the new Page status to RECEIVE_DATA status */
  flashstatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, newpageaddress, RECEIVE_DATA);
  /* If program operation was failed, a Flash error code is returned */
  if (flashstatus != HAL_OK)
  {
	return flashstatus;
  }

  /* Write the variable passed as parameter in the new active page */
  eepromstatus = EE_VerifyPageFullWriteVariable(VirtAddress, Data);
  /* If program operation was failed, a Flash error code is returned */
  if (eepromstatus != HAL_OK)
  {
	return eepromstatus;
  }

//  /* Transfer process: transfer variables from old to the new active page */
//    for (varidx = EEPROM_VIRTUAL_ADDRESS; varidx < EEPROM_VIRTUAL_ADDRESS + EEPROM_EMULATED_SIZE; varidx++)
//    {
//      if (varidx != VirtAddress)  /* Check each variable except the one passed as parameter */
//      {
//        /* Read the other last variable updates */
//        readstatus = EE_ReadVariable(varidx);
//        /* In case variable corresponding to the virtual address was found */
//        if (readstatus >= 0)//tododo actually this check is always true
//        {
//          /* Transfer the variable to the new active page */
//      	eepromstatus = EE_VerifyPageFullWriteVariable(varidx, readstatus);
//          /* If program operation was failed, a Flash error code is returned */
//          if (eepromstatus != HAL_OK)
//          {
//            return eepromstatus;
//          }
//        }
//      }
//    }
  /* Transfer process: transfer variables from old to the new active page */
  for(i=0; i<EEPROM_EMULATED_SIZE; i++)
  {
	  if(i < H_REG_COUNT)//first transfer the holding registers
	  {		//skip the register that are not in use      //skip the passed parameter
		  if((RegVirtAddr[i].virtualAddress != EEPROM_NO_WRITE_ADDRESS)&&(RegVirtAddr[i].virtualAddress != VirtAddress))
		  {
			  readstatus = EE_ReadVariable(RegVirtAddr[i].virtualAddress);
			  eepromstatus = EE_VerifyPageFullWriteVariable(RegVirtAddr[i].virtualAddress, readstatus);
		  }
	  }
//	  else//transfer other parameters
//	  {		//skip the passed parameter
//		  //if(VirtAddress != 0xB000+i-H_REG_ADDR_END)
//		  // Modified by Maya
//		  uint16_t i_special = i-H_REG_ADDR_END;
//		  if((SpecialRegistersAddr[i_special].virtualAddress != EEPROM_NO_WRITE_ADDRESS)&&(SpecialRegistersAddr[i_special].virtualAddress != VirtAddress))
//		 {
////			  readstatus = EE_ReadVariable(0xB000+i-H_REG_ADDR_END);
////			  eepromstatus = EE_VerifyPageFullWriteVariable(0xB000+i-H_REG_ADDR_END, readstatus);
//			  readstatus = EE_ReadVariable(SpecialRegistersAddr[i_special].virtualAddress);
//			  eepromstatus = EE_VerifyPageFullWriteVariable(SpecialRegistersAddr[i_special].virtualAddress, readstatus);
//		  }
//	  }
  }





  s_eraseinit.TypeErase   = FLASH_TYPEERASE_PAGES;
  s_eraseinit.PageAddress = oldpageid;
  s_eraseinit.NbPages     = 1;

  /* Erase the old Page: Set old Page status to ERASED status */
  flashstatus = HAL_FLASHEx_Erase(&s_eraseinit, &page_error);
  /* If erase operation was failed, a Flash error code is returned */
  if (flashstatus != HAL_OK)
  {
	return flashstatus;
  }

  /* Set new Page status to VALID_PAGE status */
  flashstatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, newpageaddress, VALID_PAGE);
  /* If program operation was failed, a Flash error code is returned */
  if (flashstatus != HAL_OK)
  {
	return flashstatus;
  }



  /* Return last operation flash status */
  return flashstatus;
}

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
