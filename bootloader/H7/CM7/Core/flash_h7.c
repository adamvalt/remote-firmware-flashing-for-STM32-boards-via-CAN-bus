/**
 * @file flash_h7.c
 * @author Matej Kaňuch (2023)
 * @author Mário Harvan (2023)
 * @brief Writing and reading from STM32h7 flash
 * @date 2023-02-11
 *
 * @copyright TU Brno Racing
 *
 */

#include "flash_h7.h"

static uint32_t GetSector(uint32_t Address)
{
    uint32_t sector = 0;

    if((Address < ADR_SEC_1) && (Address >= ADR_SEC_0))
    {
        sector = FLASH_SECTOR_0;
    }
    else if((Address < ADR_SEC_2) && (Address >= ADR_SEC_1))
    {
        sector = FLASH_SECTOR_1;
    }
    else if((Address < ADR_SEC_3) && (Address >= ADR_SEC_2))
    {
        sector = FLASH_SECTOR_2;
    }
    else if((Address < ADR_SEC_4) && (Address >= ADR_SEC_3))
    {
        sector = FLASH_SECTOR_3;
    }
    else if((Address < ADR_SEC_5) && (Address >= ADR_SEC_4))
    {
        sector = FLASH_SECTOR_4;
    }
    else if((Address < ADR_SEC_6) && (Address >= ADR_SEC_5))
    {
        sector = FLASH_SECTOR_5;
    }
    else if((Address < ADR_SEC_7) && (Address >= ADR_SEC_6))
    {
        sector = FLASH_SECTOR_6;
    }
    else if((Address < ADR_SEC_8) && (Address >= ADR_SEC_7))
    {
        sector = FLASH_SECTOR_7;
    }

    return sector;
}

uint32_t Flash_Erase(uint32_t StartSectorAddress, uint16_t numberofwords){

    static FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SECTORError;

    /* Get the number of sector to erase from 1st sector */

    uint32_t StartSector = GetSector(StartSectorAddress);
    uint32_t EndSectorAddress = StartSectorAddress + numberofwords*4;
    uint32_t EndSector = GetSector(EndSectorAddress);

    /* Fill EraseInit structure*/
    EraseInitStruct.Banks         = FLASH_BANK_1;
    EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector        = StartSector;
    EraseInitStruct.NbSectors     = (EndSector - StartSector) + 1;

    /* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
       you have to make sure that these data are rewritten before they are accessed during code
       execution. If this cannot be done safely, it is recommended to flush the caches by setting the
       DCRST and ICRST bits in the FLASH_CR register. */
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
    {
        return HAL_FLASH_GetError();
    }

    FLASH_WaitForLastOperation(4000, FLASH_BANK_1);

    return HAL_OK;
}

#include "stdio.h"
uint32_t Flash_Write_Data(uint32_t StartSectorAddress, uint32_t *Data, uint16_t numberofwords)
{
    int sofar=0;


    /* Unlock the Flash to enable the flash control register access *************/
    HAL_FLASH_Unlock();

    /* Erase the user Flash area */

    if (Flash_Erase(StartSectorAddress, numberofwords) != HAL_OK){
        /* Error occurred while erasing the user Flash area */
        return HAL_FLASH_GetError ();
    }

    /* Program the user Flash area word by word
      (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

    while (sofar<numberofwords)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, StartSectorAddress, (uint32_t)(Data + sofar * 8)) == HAL_OK)
        {
            StartSectorAddress += 32;  // use StartPageAddress += 2 for half word and 8 for double word
            sofar++;
        }
        else
        {
            /* Error occurred while writing data in Flash memory*/
            return HAL_FLASH_GetError();
        }
    }

    /* Lock the Flash to disable the flash control register access (recommended
       to protect the FLASH memory against possible unwanted operation) *********/
    HAL_FLASH_Lock();

    return 0;
}

void Flash_Read_Data (uint32_t StartSectorAddress, uint32_t *RxBuf, uint16_t numberofwords)
{
    for(int i = 0; i < numberofwords; i++)
    {
        RxBuf[i] = *(__IO uint32_t *)StartSectorAddress;
        StartSectorAddress += 8;
    }
}