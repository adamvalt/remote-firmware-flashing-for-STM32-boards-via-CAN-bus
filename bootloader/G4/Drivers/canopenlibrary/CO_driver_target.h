/*
 * Device and application specific definitions for CANopenNode.
 *
 * @file        CO_driver_target.h
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


#ifndef CO_DRIVER_TARGET_H
#define CO_DRIVER_TARGET_H

/* This file contains device and application specific definitions.
 * It is included from CO_driver.h, which contains documentation
 * for common definitions below. */

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include "301/CO_driver.h"

#define CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE (0x1000) // manual adding of flag needed for declating the callback function

#ifdef CO_CONFIG_MASTER
#include "CO_driver_config_Master.h"
#endif

#ifdef CO_CONFIG_SLAVE
#include "CO_driver_config_Slave.h"
#endif

#if !defined (CO_CONFIG_MASTER) && !defined (CO_CONFIG_SLAVE)
    #error "You must add -DCO_CONFIG_SLAVE or -DCO_CONFIG_MASTER definition inside CMakeList!"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Stack configuration override default values.
 * For more information see file CO_config.h. */


/* Basic definitions. If big endian, CO_SWAP_xx macros must swap bytes. */
#define CO_LITTLE_ENDIAN
#define CO_SWAP_16(x) x
#define CO_SWAP_32(x) x
#define CO_SWAP_64(x) x
/* NULL is defined in stddef.h */
/* true and false are defined in stdbool.h */
/* int8_t to uint64_t are defined in stdint.h */
typedef uint_fast8_t            bool_t;
typedef float                   float32_t;
typedef double                  float64_t;

#define NMT_CONTROL \
            (CO_NMT_STARTUP_TO_OPERATIONAL \
          | CO_NMT_ERR_ON_ERR_REG  \
          | CO_NMT_ERR_FREE_TO_OPERATIONAL)

#define FIRST_HB_TIME 500
#define SDO_SRV_TIMEOUT_TIME 1000
#define SDO_CLI_TIMEOUT_TIME 500
#define SDO_CLI_BLOCK false
#define OD_STATUS_BITS NULL
#define FLOAT_DECIMAL_PLACES 100000.f
#define OD_DEFINITION
#define CO_CONFIG_EM_ERR_STATUS_BITS_COUNT (20*8)

typedef struct {
  uint32_t ident;
  uint8_t DLC;
  uint8_t data[8];
} CO_CANrxMsg_t;

/* Access to received CAN message */
#define CO_CANrxMsg_readIdent(msg) ((uint16_t) ((CO_CANrxMsg_t *)msg)->ident)
#define CO_CANrxMsg_readDLC(msg)   ((uint8_t) ((CO_CANrxMsg_t *)msg)->DLC)
#define CO_CANrxMsg_readData(msg)  ((uint8_t *) ((CO_CANrxMsg_t *)msg)->data)

/* Received message object */
typedef struct {
    uint16_t ident;
    uint16_t mask;
    void *object;
    void (*CANrx_callback)(void *object, void *message);
} CO_CANrx_t;

/* Transmit message object */
typedef struct {
    uint32_t ident;
    uint8_t DLC;
    uint8_t data[8];
    volatile bool_t bufferFull;
    volatile bool_t syncFlag;
} CO_CANtx_t;

/* CAN module object */
typedef struct {
    void *CANptr;
    CO_CANrx_t *rxArray;
    uint16_t rxSize;
    CO_CANtx_t *txArray;
    uint16_t txSize;
    uint16_t CANerrorStatus;
    volatile bool_t CANnormal;
    volatile bool_t useCANrxFilters;
    volatile bool_t bufferInhibitFlag;
    volatile bool_t firstCANtxMessage;
    volatile uint16_t CANtxCount;
    uint32_t errOld;
} CO_CANmodule_t;

/* Data storage object for one entry */
typedef struct {
    void *addr;
    size_t len;
    uint8_t subIndexOD;
    uint8_t attr;
    void *storageModule;
    uint16_t crc;
    size_t eepromAddrSignature;
    size_t eepromAddr;
    size_t offset;
} CO_storage_entry_t;

#define FLASH_BLOCK_SIZE 4096

#define AUTOSAVE_OD_INDEX_START 0x2000
#define COMMAND_SAVE_OD_INDEX_START 0x2100
#define COMMAND_SAVE_OD_INDEX_END 0x2200

// Period in seconds of writing storage entries to flash
#define CO_STORAGE_AUTOSAVE_PERIOD 30

/* Struct with flash driver config and autosave module */
typedef struct {
    void *flahsConfig;
    void *autosaveModule;
    uint8_t dataBuff[FLASH_BLOCK_SIZE];
    bool firstWrite;
} CO_storage_module_t;


/* (un)lock critical section in CO_CANsend() */
#define CO_LOCK_CAN_SEND(CAN_MODULE)
#define CO_UNLOCK_CAN_SEND(CAN_MODULE)

/* (un)lock critical section in CO_errorReport() or CO_errorReset() */
#define CO_LOCK_EMCY(CAN_MODULE)
#define CO_UNLOCK_EMCY(CAN_MODULE)

/* (un)lock critical section when accessing Object Dictionary */
#define CO_LOCK_OD(CAN_MODULE)
#define CO_UNLOCK_OD(CAN_MODULE)

/* Synchronization between CAN receive and message processing threads. */
#define CO_MemoryBarrier()
#define CO_FLAG_READ(rxNew) ((rxNew) != NULL)
#define CO_FLAG_SET(rxNew) {CO_MemoryBarrier(); rxNew = (void*)1L;}
#define CO_FLAG_CLEAR(rxNew) {CO_MemoryBarrier(); rxNew = NULL;}


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* CO_DRIVER_TARGET_H */
