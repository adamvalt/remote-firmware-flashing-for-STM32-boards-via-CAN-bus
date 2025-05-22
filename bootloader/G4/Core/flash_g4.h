/**
 * @file flash.h
 * @brief Writing and reading from STM32g4 flash
 * @date 2023-03-27
 *
 * @copyright TU Brno Racing
 *
 */

#ifndef FLASH_H
#define FLASH_H

#include "stm32g4xx_hal.h"

/** @brief Structure containing flash memory configuration */
typedef struct {
    uint32_t size;
    uint8_t banks_num; // 1 or 2
    uint8_t pages_num; // 128 or 64
    uint8_t data_width; // 64 or 128 bits
    uint32_t page_size; // 2 or 4 kB
} flash_t;

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
 * @param length DWORD == 8 bytes
 * @return uint32_t ERR_OK if everything is ok
 */
uint32_t Flash_Write_Data_DWORD(flash_t *flash, uint32_t StartPageAddress, uint64_t *Data, size_t length);

/**
 * @brief Erase a section of flash memory.
 *
 * @param flash Flash configuration structure
 * @param StartPageAddress Starting address of the section to erase
 * @param length Number of pages to erase
 * @return uint32_t ERR_OK if successful
 */
uint32_t Flash_Erase(flash_t *flash, uint32_t StartPageAddress, uint16_t length);

/**
 * @brief Read data from flash and copy it to the RxBuf
 *
 * @param StartSectorAddress Any sector address
 * @param RxBuf Buffer for data
 * @param length DWORD == 8 bytes
 */
void Flash_Read_Data(uint32_t StartPageAddress, uint64_t *RxBuf, size_t length);

/**
 * Initializes flash configuration structure.
 * @param flash Pointer to flash configuration structure
 */
void flash_init(flash_t *flash);

#endif
