//
// Created by Mario Harvan on 28/05/2023.
//
/*
 * Eeprom interface for use with CO_storageEeprom, PIC32 specific
 *
 * @file        CO_eepromPIC32.c
 * @author      Janez Paternoster
 * @copyright   2021 Janez Paternoster
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "storage/CO_eeprom.h"
#include "301/crc16-ccitt.h"
#include "Canopen_EEPROM.h"
#include <stdlib.h>

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
#include "w25qxx.h"

#ifndef CO_STOR_EE_SIZE
#define CO_STOR_EE_SIZE (65536 * 32) // 0x4000 for 128kbits or 0x8000 for 25LC256
#endif
#ifndef CO_STOR_EE_PAGE_SIZE
#define CO_STOR_EE_PAGE_SIZE 4096
#endif

// Distribution of data in EEPROM/FLASH
#define CO_STOR_EE_AUTOSAVE_SECTION 0
// Second section for autosave duplicated data
#define CO_STOR_EE_AUTOSAVE_SPARE_SECTION (CO_STOR_EE_SIZE / 8)
#define CO_STOR_EE_SAVE_ON_COMMAND_SECTION (CO_STOR_EE_SIZE / 4)
#define CO_STOR_EE_FIRMWARE_SECTION (CO_STOR_EE_SIZE / 2)

/*
 * Eeprom is configured so, that first eight of memory is used for auto
 * storage variables. Second eight is used as a copy of first eight, to provide
 * data integrity even if MCU is reset while writing to flash. Second quarter is
 * reserved for store on command variables. Second half of flash is reserved for
 * storing new firmware.
 */
static size_t eepromAddrNextAuto = CO_STOR_EE_AUTOSAVE_SECTION;
static size_t eepromAddrNextProt = CO_STOR_EE_SAVE_ON_COMMAND_SECTION;
static volatile uint32_t dummy;


CO_storage_entry_t * CO_storage_init_storage_entries(OD_t *OD, CO_storage_module_t *storageModule, int *storageEntriesCnt)
{
    *storageEntriesCnt = 0;
    for(int i = 0; i < OD->size; i++)
    {
        if(OD->list[i].index >= AUTOSAVE_OD_INDEX_START && OD->list[i].index < COMMAND_SAVE_OD_INDEX_END)
        {
            *storageEntriesCnt += 1;
        }
    }
    CO_storage_entry_t *storage_entries = malloc(sizeof(CO_storage_entry_t) * (*storageEntriesCnt));
    int storage_entries_index = 0;
    for(int i = 0; i < OD->size; i++)
    {
        if(OD->list[i].index < AUTOSAVE_OD_INDEX_START || OD->list[i].index >= COMMAND_SAVE_OD_INDEX_END)
        {
            continue;
        }
        void *address;
        size_t len;
        switch (OD->list[i].odObjectType) {
            case ODT_VAR:
                address = ((OD_obj_var_t *) OD->list[i].odObject)->dataOrig;
                len = ((OD_obj_var_t *) OD->list[i].odObject)->dataLength;
                break;
            case ODT_ARR:
                address = ((OD_obj_array_t *) OD->list[i].odObject)->dataOrig;
                len = ((OD_obj_array_t *) OD->list[i].odObject)->dataElementLength * OD->list[i].subEntriesCount - 1;
                break;
            case ODT_REC:
                address = ((OD_obj_record_t *) OD->list[i].odObject)->dataOrig;
                len = ((OD_obj_record_t *) OD->list[i].odObject)->dataLength;
                break;
        }
        int attributes = 0;
        if(OD->list[i].index >= AUTOSAVE_OD_INDEX_START && OD->list[i].index < COMMAND_SAVE_OD_INDEX_START)
        {
            attributes = CO_storage_auto | CO_storage_cmd | CO_storage_restore;
        }
        else if(OD->list[i].index >= COMMAND_SAVE_OD_INDEX_START && OD->list[i].index < COMMAND_SAVE_OD_INDEX_END)
        {
            attributes = CO_storage_cmd | CO_storage_restore;
        }
        storage_entries[storage_entries_index].addr = address;
        storage_entries[storage_entries_index].len = len;
        storage_entries[storage_entries_index].attr = attributes;
        storage_entries[storage_entries_index].subIndexOD = 2;
        storage_entries[storage_entries_index].storageModule = storageModule;
        storage_entries_index++;
    }
    return storage_entries;
}

void CO_storage_rx_cplt_callback(CO_t * CO)
{
    CO_storage_module_t *stor = (CO_storage_module_t *) CO->storage->storageModule;
    CO_autosave_module_t *autosave = stor->autosaveModule;
    autosave->rx_complete = true;
    W25qxx_CHIP_Select(stor->flahsConfig, 1);
}

void CO_storage_tx_cplt_callback(CO_t * CO)
{
    CO_storage_module_t *stor = (CO_storage_module_t *) CO->storage->storageModule;
    CO_autosave_module_t *autosave = stor->autosaveModule;
    autosave->tx_complete = true;
    W25qxx_CHIP_Select(stor->flahsConfig, 1);
}

CO_storage_module_t * CO_flash_config_init(void *flashConfig)
{
    CO_storage_module_t *stor = malloc(sizeof(CO_storage_module_t));
    stor->flahsConfig = malloc(sizeof(w25qxx_conf_t));
    w25qxx_conf_t * config = stor->flahsConfig;
    config->hspi = ((w25qxx_conf_t *) flashConfig)->hspi;
    config->huart = ((w25qxx_conf_t *) flashConfig)->huart;
    config->gpio = ((w25qxx_conf_t *) flashConfig)->gpio;
    config->pin = ((w25qxx_conf_t *) flashConfig)->pin;
    stor->firstWrite = true;
    return stor;
}

void CO_periodic_write_init(void *storageModule)
{
    CO_storage_module_t *stor = (CO_storage_module_t *) storageModule;

    stor->autosaveModule = malloc(sizeof(CO_autosave_module_t));

    CO_autosave_module_t *autosave = stor->autosaveModule;
    autosave->tx_complete = false;
    autosave->rx_complete = false;
    autosave->current_state = AUTOSAVE_WAIT;
    autosave->write_command = false;
}

void CO_periodic_write_task(void *storageModule)
{
    CO_storage_module_t *stor = (CO_storage_module_t *) storageModule;
    CO_autosave_module_t *autosave = stor->autosaveModule;
    w25qxx_conf_t *flashConfig = (w25qxx_conf_t *)stor->flahsConfig;
    autosave_states next_state = autosave->current_state;

    static int writeBytesCnt = 0;
    static int writeSectorCnt = 0;
    switch (autosave->current_state) {
        case AUTOSAVE_WAIT:
            if(autosave->write_command)
            {
                next_state = AUTOSAVE_ERASE_BLOCK;
            }
            else
            {
                next_state = AUTOSAVE_WAIT;
            }
            break;
        case AUTOSAVE_ERASE_BLOCK:
            W25qxx_EraseSectorIT(stor->flahsConfig, autosave->address);
            next_state = AUTOSAVE_ERASE_BLOCK_WAIT_TX;
            break;
        case AUTOSAVE_ERASE_BLOCK_WAIT_TX:
            if(W25qxx_IsWriteInProgress(stor->flahsConfig))
            {
                next_state = AUTOSAVE_ERASE_BLOCK_WAIT_TX;
            }
            else
            {
                next_state = AUTOSAVE_WRITE_BLOCK;
            }
            break;
        case AUTOSAVE_WRITE_BLOCK:
            W25qxx_WritePageIT(stor->flahsConfig, autosave->write_buffer + writeBytesCnt, 256, autosave->address + writeBytesCnt);
            next_state = AUTOSAVE_WRITE_BLOCK_WAIT_TX;
            break;
        case AUTOSAVE_WRITE_BLOCK_WAIT_TX:
            if(autosave->tx_complete)
            {
                next_state = AUTOSAVE_WAIT_FOR_WRITE_END;
                writeBytesCnt += flashConfig->w25qxx_info.PageSize;
                autosave->tx_complete = false;
            }
            else
            {
                next_state = AUTOSAVE_WRITE_BLOCK_WAIT_TX;
            }
            break;
        case AUTOSAVE_WAIT_FOR_WRITE_END:
            if(W25qxx_IsWriteInProgress(stor->flahsConfig))
            {
                next_state = AUTOSAVE_WAIT_FOR_WRITE_END;
            }
            else
            {
                if(writeBytesCnt < flashConfig->w25qxx_info.SectorSize)
                {
                    next_state = AUTOSAVE_WRITE_BLOCK;
                }
                else
                {
                    writeSectorCnt++;
                    writeBytesCnt = 0;
                    if(writeSectorCnt < 2)
                    {
                        autosave->address += CO_STOR_EE_AUTOSAVE_SPARE_SECTION;
                        next_state = AUTOSAVE_ERASE_BLOCK;
                    }
                    else
                    {
                        writeSectorCnt = 0;
                        next_state = AUTOSAVE_WAIT;
                        autosave->write_command = false;
                    }
                }
            }
            break;
        default:
            break;
    }
    autosave->current_state = next_state;
}


/******************************************************************************/
bool_t CO_eeprom_init(void *storageModule) {
    eepromAddrNextAuto = 0;
    eepromAddrNextProt = CO_STOR_EE_SAVE_ON_COMMAND_SECTION;
    void *flashConfig = ((CO_storage_module_t *) storageModule)->flahsConfig;

    return W25qxx_Init(flashConfig);
}


/******************************************************************************/
size_t CO_eeprom_getAddr(void *storageModule, bool_t isAuto,
                         size_t len, bool_t *overflow)
{
    size_t addr;
    if (isAuto) {
        /* auto storage is processed byte by byte, no alignment necessary */
        addr = eepromAddrNextAuto;
        eepromAddrNextAuto += len;
        // This is limitation of current flash implementation. We can periodically write to one buffer only.
        // It needs to be reworked a bit to be able to write to multiple pages.
        if (eepromAddrNextAuto > CO_STOR_EE_PAGE_SIZE - 1) {
            *overflow = true;
        }
    }
    else {
        /* addresses for storage on command must be page aligned */
        addr = eepromAddrNextProt;
        size_t lenAligned = len & (~(CO_STOR_EE_PAGE_SIZE - 1));
        if (lenAligned < len) {
            lenAligned += CO_STOR_EE_PAGE_SIZE;
        }
        eepromAddrNextProt += lenAligned;
        if (eepromAddrNextProt > CO_STOR_EE_FIRMWARE_SECTION) {
            *overflow = true;
        }
    }
    return addr;
}


/******************************************************************************/
void CO_eeprom_readBlock(void *storageModule, uint8_t *data,
                         size_t eepromAddr, size_t len)
{
    void *flashConfig = ((CO_storage_module_t *) storageModule)->flahsConfig;
    uint8_t tmpBuf[4096];
    W25qxx_ReadSector(flashConfig, tmpBuf, eepromAddr / CO_STOR_EE_PAGE_SIZE, 0, CO_STOR_EE_PAGE_SIZE);
    if(eepromAddr <= CO_STOR_EE_AUTOSAVE_SPARE_SECTION)
    {
        if(tmpBuf[4095] != 0)
        {
            uint32_t secondaryAddress = CO_STOR_EE_AUTOSAVE_SPARE_SECTION + eepromAddr;
            W25qxx_ReadSector(flashConfig, tmpBuf, secondaryAddress / CO_STOR_EE_PAGE_SIZE, 0, CO_STOR_EE_PAGE_SIZE);
        }
    }
    memcpy(data, tmpBuf + (eepromAddr % 4096), len);
}


/******************************************************************************/
bool_t CO_eeprom_writeBlock(void *storageModule, uint8_t *data, size_t eepromAddr, size_t len)
{
    void *flashConfig = ((CO_storage_module_t *) storageModule)->flahsConfig;
    uint8_t buff[CO_STOR_EE_PAGE_SIZE] = {0};
    W25qxx_ReadSector(flashConfig, buff, eepromAddr / CO_STOR_EE_PAGE_SIZE, 0, CO_STOR_EE_PAGE_SIZE);
    for (int i = 0; i < len; i++) {
        buff[(eepromAddr % CO_STOR_EE_PAGE_SIZE) + i] = data[i];
    }
    W25qxx_EraseSector(flashConfig, eepromAddr / CO_STOR_EE_PAGE_SIZE);
    W25qxx_WriteSector(flashConfig, buff, eepromAddr / CO_STOR_EE_PAGE_SIZE, 0, CO_STOR_EE_PAGE_SIZE);
    if(eepromAddr <= CO_STOR_EE_AUTOSAVE_SPARE_SECTION)
    {
        uint32_t secondaryAddress = CO_STOR_EE_AUTOSAVE_SPARE_SECTION + eepromAddr;
        W25qxx_EraseSector(flashConfig, secondaryAddress / CO_STOR_EE_PAGE_SIZE);
        W25qxx_WriteSector(flashConfig, buff, secondaryAddress / CO_STOR_EE_PAGE_SIZE, 0, CO_STOR_EE_PAGE_SIZE);
    }
    return true;
}


/******************************************************************************/
uint16_t CO_eeprom_getCrcBlock(void *storageModule,
                               size_t eepromAddr, size_t len)
{
    uint8_t buff[CO_STOR_EE_PAGE_SIZE] = {0};
    void *flashConfig = ((CO_storage_module_t *) storageModule)->flahsConfig;

    W25qxx_ReadSector(flashConfig, buff, eepromAddr / CO_STOR_EE_PAGE_SIZE, eepromAddr % CO_STOR_EE_PAGE_SIZE, len);
    return crc16_ccitt(buff, len, 0);
}


/******************************************************************************/
bool_t CO_eeprom_updateByte(void *storageModule, uint8_t data,
                            size_t eepromAddr)
{
    CO_storage_module_t *stor = (CO_storage_module_t *) storageModule;
    CO_autosave_module_t *autosave = stor->autosaveModule;

    // Write at address 0 means that all data are written to buffer and we will write it to memory
    // This implementation works only for one page of data
    if(eepromAddr == 0)
    {
        if(stor->firstWrite)
        {
            stor->firstWrite = false;
        }
        else
        {
            autosave->write_command = true;
            autosave->address = eepromAddr;

            // Last byte of page is used to validate correct write operation. If it's 0, data in sector are correct
            stor->dataBuff[CO_STOR_EE_PAGE_SIZE - 1] = 0;
            memcpy(autosave->write_buffer, stor->dataBuff, CO_STOR_EE_PAGE_SIZE);
        }
    }
    stor->dataBuff[eepromAddr % CO_STOR_EE_PAGE_SIZE] = data;
    return true;
}

void CO_storageEeprom_auto_process(CO_storage_t *storage, bool_t saveAll) {
    if (storage == NULL || !storage->enabled) {
        return;
    }

    static uint8_t write_buffer[CO_STOR_EE_PAGE_SIZE] __ALIGNED(8);
    static size_t buffer_pos = 0;
    static size_t current_entry = 0;
    static size_t current_offset = 0;

    // Process one entry at a time
    if (current_entry < storage->entriesCount) {
        CO_storage_entry_t *entry = &storage->entries[current_entry];
        
        if (entry->attr & CO_storage_auto) {
            if (saveAll) {
                // Batch write for saveAll
                size_t remaining = entry->len;
                size_t src_offset = 0;
                
                while (remaining > 0) {
                    size_t to_copy = MIN(CO_STOR_EE_PAGE_SIZE - buffer_pos, remaining);
                    memcpy(write_buffer + buffer_pos, 
                          (uint8_t*)entry->addr + src_offset, 
                          to_copy);
                    
                    buffer_pos += to_copy;
                    src_offset += to_copy;
                    remaining -= to_copy;
                    
                    if (buffer_pos >= CO_STOR_EE_PAGE_SIZE) {
                        // Write full page
                        CO_eeprom_writeBlock(entry->storageModule, 
                                           write_buffer,
                                           entry->eepromAddr + src_offset - buffer_pos,
                                           CO_STOR_EE_PAGE_SIZE);
                        buffer_pos = 0;
                    }
                }
                
                // Write remaining data
                if (buffer_pos > 0) {
                    CO_eeprom_writeBlock(entry->storageModule,
                                       write_buffer,
                                       entry->eepromAddr + entry->len - buffer_pos,
                                       buffer_pos);
                    buffer_pos = 0;
                }
            } else {
                // Single byte update with buffering
                uint8_t dataByte = ((uint8_t*)entry->addr)[entry->offset];
                write_buffer[buffer_pos++] = dataByte;
                
                if (buffer_pos >= CO_STOR_EE_PAGE_SIZE || 
                    entry->offset == entry->len - 1) {
                    // Write buffered data
                    CO_eeprom_writeBlock(entry->storageModule,
                                       write_buffer,
                                       entry->eepromAddr + entry->offset - buffer_pos + 1,
                                       buffer_pos);
                    buffer_pos = 0;
                }
                
                if (++entry->offset >= entry->len) {
                    entry->offset = 0;
                    current_entry++;
                }
            }
        } else {
            current_entry++;
        }
    } else {
        current_entry = 0;
    }
}

#endif