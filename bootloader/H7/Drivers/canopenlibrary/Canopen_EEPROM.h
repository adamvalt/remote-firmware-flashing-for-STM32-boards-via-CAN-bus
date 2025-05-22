//
// Created by Mario Harvan on 03/06/2023.
//

#ifndef BENCHTEST_G4_CANOPEN_EEPROM_H
#define BENCHTEST_G4_CANOPEN_EEPROM_H

#include "301/CO_ODinterface.h"
#include "CANopen.h"

/* States for automatic saving procedure */
typedef enum
{
    AUTOSAVE_WAIT,
    AUTOSAVE_ERASE_BLOCK,
    AUTOSAVE_ERASE_BLOCK_WAIT_TX,
    AUTOSAVE_WRITE_BLOCK,
    AUTOSAVE_WRITE_BLOCK_WAIT_TX,
    AUTOSAVE_WAIT_FOR_WRITE_END
} autosave_states;

/* Automatic saving structure for autosave functionality */
typedef struct
{
    autosave_states current_state;
    uint8_t write_buffer[4096];
    uint32_t address;
    bool write_command;
    bool rx_complete;
    bool tx_complete;
} CO_autosave_module_t;

/**
 * @brief Find all OD entries that are saved to persistent storage, eg. ID's 0x2000-0x2200
 * @param OD
 * @param storageModule with flash driver
 * @param storageEntriesCnt pointer which will be filled with entries count
 * @return storage entries array, with all entries that are stored to persistent storage
 */
CO_storage_entry_t * CO_storage_init_storage_entries(OD_t *OD, CO_storage_module_t *storageModule, int *storageEntriesCnt);

/**
 * @brief Flash receive complete operation callback.
 * Call this function inside RX callback
 * @param CO CANOpen instance which has storage configured
 */
void CO_storage_rx_cplt_callback(CO_t * CO);

/**
 * @brief Flash transmit complete operation callback.
 * Call this function inside TX callback
 * @param CO CANOpen instance which has storage configured
 */
void CO_storage_tx_cplt_callback(CO_t * CO);

/**
 * @brief Copy flash config to CANOpen storage module instance
 * @param flashConfig of w25qxx_conf_t type
 * @return CANOpen storage module
 */
CO_storage_module_t * CO_flash_config_init(void *flashConfig);

/**
 * @brief Initialize periodic writter
 * @param storageModule of CO_storage_module_t type
 */
void CO_periodic_write_init(void *storageModule);

/**
 * @brief Periodic writter task which handles non-blocking communication with flash
 * @param storageModule of type CO_storage_module_t
 */
void CO_periodic_write_task(void *storageModule);


#endif //BENCHTEST_G4_CANOPEN_EEPROM_H
