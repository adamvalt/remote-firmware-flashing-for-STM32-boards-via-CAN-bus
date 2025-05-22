#include "Canopen_Main.h"
#include "CANopen.h"
#include "301/CO_HBconsumer.h"

#include "storage/CO_storageEeprom.h"
#include "Canopen_EEPROM.h"

#define CO_TPDO_DEFAULT_INDEX 0x1800
#define CO_RPDO_BASE_ADDRESS 0x1400

// Internal variables

CO_InstanceList_t COInstanceList[CO_MAX_INSTANCE_CNT];

CO_LegacyCanCallback_t COLegacyCallbackList[CO_MAX_INSTANCE_CNT];

uint8_t COInstanceCnt = 0;

uint8_t COLegacyCanCallbackCnt = 0;

// End of internal variables

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE

void CANOpenStorageInit(CO_t *CO, OD_t *OD, CO_config_t *ODConfig, void *flashConfig)
{
    CO_storage_t *storage = malloc(sizeof(CO_storage_t));
    CO_storage_module_t *flashStorageModule = CO_flash_config_init(flashConfig);
    int storageEntriesCnt = 0;

    CO_periodic_write_init(flashStorageModule);

    CO_storage_entry_t * storageEntries = CO_storage_init_storage_entries(OD, flashStorageModule, &storageEntriesCnt);

    uint32_t storageInitError;
    CO_storageEeprom_init(storage,
                          CO->CANmodule,
                          flashStorageModule,
                          ODConfig->ENTRY_H1010,
                          ODConfig->ENTRY_H1011,
                          storageEntries,
                          storageEntriesCnt,
                          &storageInitError);

    storage->storageModule = flashStorageModule;
    CO->storage = storage;
}
#endif

CO_t * CANOpenInit(void *CANPtr, CO_config_t *config, OD_t *OD, int canNodeId)
{
    CO_t *CO = NULL;
    CO_ReturnError_t err;
    CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
    uint32_t heapMemoryUsed;
    void *CANptr = CANPtr; /* CAN module address */
    uint16_t pendingBitRate = 1000;  /* read from dip switches or nonvolatile memory, configurable by LSS slave */

    CO = CO_new(config, &heapMemoryUsed);
    CO->CANmodule->CANnormal = false;

    /* initialize CANopen */
    err = CO_CANinit(CO, CANptr, pendingBitRate);

    uint32_t errInfo = 0;

    err = CO_CANopenInit(CO,                /* CANopen object */
                        NULL,              /* alternate NMT */
                        NULL,              /* alternate em */
                        OD,                /* Object dictionary */
                        OD_STATUS_BITS,    /* Optional OD_statusBits */
                        NMT_CONTROL,       /* CO_NMT_control_t */
                        FIRST_HB_TIME,     /* firstHBTime_ms */
                        SDO_SRV_TIMEOUT_TIME, /* SDOserverTimeoutTime_ms */
                        SDO_CLI_TIMEOUT_TIME, /* SDOclientTimeoutTime_ms */
                        SDO_CLI_BLOCK,     /* SDOclientBlockTransfer */
                        canNodeId,
                        &errInfo);
    err = CO_CANopenInitPDO(CO, CO->em, OD, canNodeId, &errInfo);

    CO_CANsetNormalMode(CO->CANmodule);
    COInstanceList[COInstanceCnt].CO = CO;
    COInstanceList[COInstanceCnt].lastProcessTime = 0;
    COInstanceList[COInstanceCnt].lastAutostoreTime = 0;
    COInstanceList[COInstanceCnt].lastSuccRxMsgTime = 0;
    COInstanceList[COInstanceCnt].lastSuccTxMsgTime = 0;
    COInstanceList[COInstanceCnt].txMessageCounter = 0;
    COInstanceList[COInstanceCnt].rxMessageCounter = 0;
    COInstanceList[COInstanceCnt].lastCanResetTime = 0;
    COInstanceList[COInstanceCnt].canErrCnt = 0;
    COInstanceList[COInstanceCnt++].rpdoTimeoutCallback = NULL;
    return CO;
}

void CANOpenProcess(CO_t * CO, uint32_t currentTick)
{
    for(uint8_t i = 0; i < COInstanceCnt; i++)
    {
      if(COInstanceList[i].CO == CO) {
          if (currentTick != COInstanceList[i].lastProcessTime) {
              bool communicationErr = CO_isError(CO->em, CO_EM_CAN_TX_OVERFLOW)
                                      || CO_isError(CO->em, CO_EM_CAN_RX_BUS_PASSIVE)
                                      || CO_isError(CO->em, CO_EM_CAN_TX_BUS_PASSIVE)
                                      || CO_isError(CO->em, CO_EM_CAN_TX_BUS_OFF);

              if(communicationErr)
              {
                  // Reset timeout, because when you reset interface too often it won't have enough
                  // time to startup and clear errors
                  if(currentTick - COInstanceList[i].lastCanResetTime > CO_CAN_RESET_TIMEOUT)
                  {
                      CO_CANmodule_disable(CO->CANmodule);
                      CO_CANsetNormalMode(CO->CANmodule);
                      CO->CANmodule->CANtxCount = 0;
                      for(uint16_t j = 0; j < CO->CNT_ALL_TX_MSGS; j++)
                      {
                          CO->CANtx[j].bufferFull = false;
                      }
                      COInstanceList[i].lastCanResetTime = currentTick;
                  }
              }
              CO_NMT_reset_cmd_t reset = CO_process(CO, false, (currentTick - COInstanceList[i].lastProcessTime) * 1000, NULL);
              if(reset != 0)
              {
                  CO_System_reset();
              }
#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
              // Calculate autosave period based on entries count
              int autosavePeriod = (1000 * CO_STORAGE_AUTOSAVE_PERIOD) / CO->storage->entriesCount;
              if(currentTick - COInstanceList[i].lastAutostoreTime > autosavePeriod)
              {
                  CO_storageEeprom_auto_process(CO->storage, false);
                  COInstanceList[i].lastAutostoreTime = currentTick;
              }
              CO_periodic_write_task(CO->storage->storageModule);
#endif
              CO_process_RPDO(CO, false, (currentTick - COInstanceList[i].lastProcessTime) * 1000, NULL);
              CO_process_TPDO(CO, false, (currentTick - COInstanceList[i].lastProcessTime) * 1000, NULL);

              if(CO_isError(CO->em, CO_EM_RPDO_TIME_OUT))
              {
                  COInstanceList[i].rpdoTimeoutCallback();
                  CO_errorReset(CO->em, CO_EM_RPDO_TIME_OUT, 0);
              }
              COInstanceList[i].lastProcessTime = currentTick;
          }
      }
    }
}

void CANOpenRegisterHBConsCallback(CO_t * CO, void (*callback)(uint8_t nodeId, uint8_t idx, CO_NMT_internalState_t NMTstate, void *object))
{
#if CO_CONFIG_HB_CONS
    CO_HBconsumer_initCallbackNmtChanged(CO->HBcons, 0, NULL, callback);
#endif
    return;
}

void CANOpenRegisterRPDOTimeoutCallback(CO_t * CO, OD_entry_t *RPDOEntry, uint32_t timeout, void (*callback)())
{
    if(RPDOEntry->index - CO_RPDO_BASE_ADDRESS < 0)
    {
        return;
    }
    else if(RPDOEntry->index - CO_RPDO_BASE_ADDRESS > CO->CNT_ALL_RX_MSGS)
    {
        return;
    }
    CO->RPDO[RPDOEntry->index - CO_RPDO_BASE_ADDRESS].timeoutTime_us = timeout * 1000;
    for(uint8_t i = 0; i < COInstanceCnt; i++)
    {
        if(COInstanceList[i].CO == CO)
        {
            COInstanceList[i].rpdoTimeoutCallback = callback;
            return;
        }
    }
}

//OD_ENTRY_H1801_transmitPDO_Communication_2
void CANOpenSendPDO(CO_t * CO, OD_entry_t *TPDOEntry)
{
  int TPDOindex = TPDOEntry->index - CO_TPDO_DEFAULT_INDEX;
  CO->TPDO[TPDOindex].sendRequest = true;
}

void CANOpenStorageRxCpltCallback(CO_t * CO)
{
    CO_storage_rx_cplt_callback(CO);
}

void CANOpenStorageTxCpltCallback(CO_t * CO)
{
    CO_storage_tx_cplt_callback(CO);
}

char * CANOpenGetErrorClassString(CO_EM_errorCode_t errorClass)
{
    if(errorClass == 0)
    {
        return "CO_NO_ERROR";
    }
    int errorClassIndex = errorClass - CO_EMC_ADDITIONAL_FUNC - 1;
    if(errorClassIndex >= 0 && errorClassIndex < sizeof(error_class_string))
    {
        return error_class_string[errorClassIndex];
    }
    else
    {
        return "CO_UNKNOWN_ERROR_CLASS";
    }
}

char * CANOpenGetErrorCodeString(CO_EM_errorStatusBits_t errorCode)
{
    if(errorCode == 0)
    {
        return "CO_NO_ERROR";
    }
    int errorCodeIndex = errorCode - CO_EM_MANUFACTURER_START - 1;
    if(errorCodeIndex >= 0 && errorCodeIndex < sizeof(error_codes_string))
    {
        return error_codes_string[errorCodeIndex];
    }
    else
    {
        return "CO_UNKOWN_ERROR_CODE";
    }
}

void CANOpenSetError(CO_t * CO, CO_EM_errorCode_t errorClass, CO_EM_errorStatusBits_t errorCode, uint32_t infoCode)
{
    CO_errorReport(CO->em, errorCode, errorClass, infoCode);
}

void CANOpenClearError(CO_t * CO, CO_EM_errorStatusBits_t errorCode, uint32_t infoCode)
{
    CO_errorReset(CO->em, errorCode, infoCode);
}

bool CANOpenIsError(CO_t * CO, CO_EM_errorStatusBits_t errorCode)
{
    return CO_isError(CO->em, errorCode);
}

#if ((CO_CONFIG_EM) & CO_CONFIG_EM_CONSUMER)
void CANOpenRegisterEmcyErrorCallback(CO_t * CO, void (*emcyErrorCallbackFuncPointer)(const uint16_t nodeId,
                                                                        const uint16_t errorCode,
                                                                        const uint8_t errorRegister,
                                                                        const uint8_t errorBit,
                                                                        const uint32_t infoCode)) {
    CO_EM_initCallbackRx(CO->em, emcyErrorCallbackFuncPointer);
}
#endif

CO_CANInterface_stats_t CANOpenGetInterfaceStats(CO_t * CO, uint32_t currentTick)
{
    for(uint8_t i = 0; i < COInstanceCnt; i++)
    {
        if(CO == COInstanceList[i].CO)
        {
            CO_CANInterface_stats_t stats = {
                    .rxMessageCounter = COInstanceList[i].rxMessageCounter,
                    .txMessageCounter = COInstanceList[i].txMessageCounter,
                    .timeSinceLastRxMsg = currentTick - COInstanceList[i].lastSuccRxMsgTime,
                    .timeSinceLastTxMsg = currentTick - COInstanceList[i].lastSuccTxMsgTime
            };
            return stats;
        }
    }
    CO_CANInterface_stats_t empty;
    return empty;
}

void CANOpenInitLegacyCanInterface(void *CANPtr, void (*callback)(CO_CANrxMsg_t rcvMsg))
{
    if(COLegacyCanCallbackCnt < CO_MAX_INSTANCE_CNT)
    {
        COLegacyCallbackList[COLegacyCanCallbackCnt].CANPtr = CANPtr;
        COLegacyCallbackList[COLegacyCanCallbackCnt].receiveMsgCallback = callback;
        COLegacyCallbackList[COLegacyCanCallbackCnt].txMsgBufferEnd = 0;
        COLegacyCallbackList[COLegacyCanCallbackCnt].txMsgBufferStart = 0;
        COLegacyCallbackList[COLegacyCanCallbackCnt].lastCanResetTime = 0;
        COLegacyCallbackList[COLegacyCanCallbackCnt].canErrCnt = 0;
        COLegacyCanCallbackCnt++;
        CO_legacyCanInit(CANPtr);
    }
}

void CANOpenSendLegacyCanMessage(void *CANptr, uint32_t ID, uint8_t DLC, uint8_t *data)
{
    CO_legacyCanSendMessage(CANptr, ID, DLC, data);
}

void CANOpenLegacyPeriodicTask(void *CANptr)
{
    CO_legacyCanExceptionHandler(CANptr);
}
