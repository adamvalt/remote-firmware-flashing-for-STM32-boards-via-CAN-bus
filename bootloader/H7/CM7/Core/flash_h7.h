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

#ifndef VCU_STM32_FLASH_H7_H
#define VCU_STM32_FLASH_H7_H

#include "stm32h7xx_hal.h"

/** @brief Flash sector addresses */
#define ADR_SEC_0 0x08000000
#define ADR_SEC_1 0x08020000
#define ADR_SEC_2 0x08040000
#define ADR_SEC_3 0x08060000
#define ADR_SEC_4 0x08080000
#define ADR_SEC_5 0x080A0000
#define ADR_SEC_6 0x080C0000
#define ADR_SEC_7 0x080D0000
#define ADR_SEC_8 0x08100000

/** @brief Size of a flash sector in bytes */
#define SECTOR_SIZE (ADR_SEC_3 - ADR_SEC_2)

/**
 * @brief Get the starting address of the sector
 *
 * @param Address Any flash address
 * @return uint32_t
 */
static uint32_t GetSector(uint32_t Address);

/**
 * @brief Write data from *Data buffer
 *
 * The whole sector memory will be erased every time you call this function !
 *
 * @param StartSectorAddress Any sector address
 * @param Data
 * @param numberofwords 1 WORD == 32 bytes
 * @return uint32_t ERR_OK if everything is ok
 */
uint32_t Flash_Write_Data(uint32_t StartSectorAddress, uint32_t *Data, uint16_t numberofwords);

/**
 * @brief Read data from flash and copy it to the RxBuf
 *
 * @param StartSectorAddress Any sector address
 * @param RxBuf Buffer for data
 * @param numberofwords 1 WORD == 8 bytes
 */
void Flash_Read_Data(uint32_t StartSectorAddress, uint32_t *RxBuf, uint16_t numberofwords);

/**
 * Erases a section of flash memory.
 * @param StartSectorAddress Starting address of the section to erase
 * @param numberofwords Number of words to erase
 * @return ERR_OK if successful
 */
uint32_t Flash_Erase(uint32_t StartSectorAddress, uint16_t numberofwords);

#endif //VCU_STM32_FLASH_H7_H
