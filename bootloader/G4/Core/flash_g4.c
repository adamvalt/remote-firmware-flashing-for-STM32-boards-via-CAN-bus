/**
 * @file flash.c
 * @author Juraj Martiček
 * @brief Writing and reading from STM32g4 flash
 * @date 2023-03-27
 *
 * @copyright TU Brno Racing
 *
 */

#include "flash_g4.h"
#include <stdint.h>

#define BANK1_ADDR (0x08000000)
#define BANK2_ADDR (0x08000000 + FLASH_BANK_SIZE)

void flash_init(flash_t *flash)
{
	if (FLASH->OPTR & FLASH_OPTR_DBANK)
	{
		// Dual bank mode
		flash->banks_num = 2;
		flash->pages_num = FLASH_PAGE_NB;
		flash->page_size = FLASH_PAGE_SIZE;
		flash->data_width = 64;
	}
	else
	{
		// Single bank mode
		flash->banks_num = 1;
		flash->pages_num = FLASH_PAGE_NB;
		flash->page_size = FLASH_PAGE_SIZE_128_BITS;
		flash->data_width = 128;
	}
}

static uint32_t GetPage(flash_t *flash, uint32_t Address, uint32_t BankStartAddress)
{
	// TODO: test second bank
	for (int indx = 0; indx < flash->pages_num; indx++)
	{
		if ((Address < (BankStartAddress + (flash->page_size * (indx + 1)))) && (Address >= (BankStartAddress + flash->page_size * indx)))
		{
			return indx;
		}
	}
	return 0;
}

uint32_t Flash_Erase(flash_t *flash, uint32_t StartPageAddress, uint16_t length) {
	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PAGEError;

	uint32_t bankStartAddress = BANK1_ADDR;
	if(flash->banks_num == 2) {
		if(StartPageAddress < BANK2_ADDR) {
			bankStartAddress = BANK1_ADDR;
		} else {
			bankStartAddress = BANK2_ADDR;
		}
	}

	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	/* Erase the user Flash area*/
	uint32_t StartPage = GetPage(flash, StartPageAddress, bankStartAddress);
	uint32_t EndPageAdress = StartPageAddress + length * (flash->data_width / 8);
	uint32_t EndPage = GetPage(flash, EndPageAdress, bankStartAddress);

	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
 	EraseInitStruct.Banks = bankStartAddress == BANK1_ADDR ? FLASH_BANK_1 : FLASH_BANK_2;
	EraseInitStruct.Page = StartPage;
	EraseInitStruct.NbPages = (EndPage - StartPage) + 1;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
	{
		/*Error occurred while page erase.*/
		return HAL_FLASH_GetError();
	}

	HAL_FLASH_Lock();
	return HAL_OK;
}

uint32_t Flash_Write_Data_DWORD(flash_t *flash, uint32_t StartPageAddress, uint64_t *Data, size_t length)
{
	HAL_FLASH_Unlock();
	/* Program the user Flash area dword by dword*/
	for(size_t i = 0; i < length; i++) {
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, StartPageAddress, Data[i]) != HAL_OK)
		{
			/* Error occurred while writing data in Flash memory*/
			return HAL_FLASH_GetError();
		}
		StartPageAddress += 8; // use StartPageAddress += 2 for half word and 4 for word
	}

	/* Lock the Flash to disable the flash control register access (recommended
	   to protect the FLASH memory against possible unwanted operation) *********/
	HAL_FLASH_Lock();

	return HAL_OK;
}

void Flash_Read_Data(uint32_t StartPageAddress, uint64_t *RxBuf, size_t length)
{
	for(int i = 0; i < length; i++)
    {
		RxBuf[i] = *(__IO uint64_t *)StartPageAddress;
        StartPageAddress += 8;
    }

}
