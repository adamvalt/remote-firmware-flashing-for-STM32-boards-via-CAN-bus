#ifndef CAN_OPEN_MAIN_H
#define CAN_OPEN_MAIN_H

#include "CANopenNode/CANopen.h"
#include "CO_driver_target.h"
#include "Canopen_error_codes.h"
#include "stdlib.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CO_MAX_INSTANCE_CNT 4
#define CO_LEGACY_CAN_TX_BUFFER_SIZE 32
#define CO_CAN_RESET_TIMEOUT 2000

typedef struct {
    CO_t * CO;
    uint32_t lastProcessTime;
    uint32_t lastAutostoreTime;
    void (*rpdoTimeoutCallback)();
    uint32_t txMessageCounter;
    uint32_t rxMessageCounter;
    uint32_t lastSuccTxMsgTime;
    uint32_t lastSuccRxMsgTime;
    uint32_t lastCanResetTime;
    uint32_t canErrCnt;
} CO_InstanceList_t;

typedef struct
{
    void * CANPtr;
    void (*receiveMsgCallback)(CO_CANrxMsg_t rcvMsg);
    CO_CANtx_t txBuffer[CO_LEGACY_CAN_TX_BUFFER_SIZE];
    uint32_t txMsgBufferStart;
    uint32_t txMsgBufferEnd;
    uint32_t lastCanResetTime;
    uint32_t canErrCnt;
} CO_LegacyCanCallback_t;

typedef struct
{
    uint32_t rxMessageCounter;
    uint32_t txMessageCounter;
    uint32_t timeSinceLastTxMsg;
    uint32_t timeSinceLastRxMsg;
} CO_CANInterface_stats_t;

/**
 * @brief Initialize CANOpen persistent storage
 * @param CO pointer to CANOpen instancce
 * @param OD pointer to Object dictionary
 * @param ODConfig pointer to object dictionary config generated with CO_CREATE_OD_CONFIG
 * @param flashConfig pointer to struct w25qxx_conf_t with configured flash IO
 */
void CANOpenStorageInit(CO_t *CO, OD_t *OD, CO_config_t *ODConfig, void *flashConfig);

/**
 * @brief Initialize CANOpen 
 * @param Pointer to Can HW interface
 * @param Pointer to Can configuration generated with CO_CREATE_OD_CONFIG macro
 * @param Pointer to CANOpen Object Dictionary
 * @param CANOpen ID of this node
 * @param Flash configuration, or null if you dont use storage
 * @return Pointer to CANopen instance
*/
CO_t * CANOpenInit(void *CANPtr, CO_config_t *config, OD_t *OD, int canNodeId);


/**
 * @brief Process all CANOpen protocols, needs to be called with 1ms period
 * @param Pointer to CANOpen instance, generated from CANOpenInit function
 * @param Current tick count from HAL_GetTick()
*/
void CANOpenProcess(CO_t * CO, uint32_t currentTick);

void CANOpenRegisterHBConsCallback(CO_t * CO, void (*callback)(uint8_t nodeId, uint8_t idx, CO_NMT_internalState_t NMTstate, void *object));

/**
 * @brief This function is not yet implemented
*/
void CANOpenRegisterRPDOCallback();

/**
 * @brief Set timeout of rpdo message with callback function
 * @param Pointer to CANOpen instance, generated from CANOpenInit function
 * @param TPDO Entry from Object Dictionary, for example: ODGen_ENTRY_H1400_receivePDO_Communication_1
 * @param Max acceptable timeout in ms
 * @param pointer to callback function
 */
void CANOpenRegisterRPDOTimeoutCallback(CO_t * CO, OD_entry_t *RPDOEntry, uint32_t timeout, void (*callback)());

/**
 * @brief Manualy send TPDO message
 * @param Pointer to CANOpen instance, generated from CANOpenInit function
 * @param TPDO Entry from Object Dictionary, for example: ODGen_ENTRY_H1800_transmitPDO_Communication_1
*/
void CANOpenSendPDO(CO_t * CO, OD_entry_t *TPDOEntry);

/**
 * @brief RX handler for flash asynchronous communication. Call inside RX completed callback
 * @param CO pointer to CANOpen instance with configured persistent storage
 */
void CANOpenStorageRxCpltCallback(CO_t * CO);

/**
 * @brief TX handler for flash asynchronous communication. Call inside RX completed callback
 * @param CO pointer to CANOpen instance with configured persistent storage
 */
void CANOpenStorageTxCpltCallback(CO_t * CO);

/**
 * @brief Get string from error class enum
 * @param errorClass enum
 * @return string from errorClass enum
 */
char * CANOpenGetErrorClassString(CO_EM_errorCode_t errorClass);

/**
 * @brief Get string from error code enum
 * @param errorCode enum
 * @return string from errorCode enum
 */
char * CANOpenGetErrorCodeString(CO_EM_errorStatusBits_t errorCode);

/**
 * @brief Set CANOpen error, which is send to CANOpen network
 * @param CO pointer to CANOpen instance
 * @param errorClass defined in Canopen_error_codes.h
 * @param errorCode defined in Canopen_error_codes.h
 * @param infoCode additional error information
 */
void CANOpenSetError(CO_t * CO, CO_EM_errorCode_t errorClass, CO_EM_errorStatusBits_t errorCode, uint32_t infoCode);

/**
 * @brief Clears CANOpen error
 * @param CO pointer to CANOpen instance
 * @param errorCode defined in Canopen_error_codes
 * @param infoCode additional error clear information
 */
void CANOpenClearError(CO_t * CO, CO_EM_errorStatusBits_t errorCode, uint32_t infoCode);

/**
 * @brief Check if error code is set
 * @param CO pointer to CANOpen instance
 * @param errorCode which is checked
 * @return true if error is set
 */
bool CANOpenIsError(CO_t * CO, CO_EM_errorStatusBits_t errorCode);

/**
 * @brief Register emergency error callback function
 * @param CO pointer to CANOpen instance
 * @param emcyErrorCallbackFuncPointer pointer to callback function
 */
void CANOpenRegisterEmcyErrorCallback(CO_t * CO, void (*emcyErrorCallbackFuncPointer)(const uint16_t nodeId,
                                                                                      const uint16_t errorCode,
                                                                                      const uint8_t errorRegister,
                                                                                      const uint8_t errorBit,
                                                                                      const uint32_t infoCode));

/**
 * @brief Generate interface statistics such as txCounter, rxCounter, time since last message...
 * @param CO pointer to CANOpen instance
 * @param currentTick time since MCU boot in MS
 * @return CO_CANInterface_stats_t struct with statistical data
 */
CO_CANInterface_stats_t CANOpenGetInterfaceStats(CO_t * CO, uint32_t currentTick);



/************************ Legacy CAN functions ***************************/


/**
 * @brief Init legacy Can interface, with callback function
 * @param Pointer to non CanOpen Can interface
 * @param Callback function which will be called from interrupt when new data is available
*/
void CANOpenInitLegacyCanInterface(void *CANPtr, void (*callback)(CO_CANrxMsg_t rcvMsg));

/**
 * @brief Send message through legacy can interface
 * @param CANptr pointer to can interface
 * @param ID of message
 * @param DLC length of message
 * @param data buffer
 */
void CANOpenSendLegacyCanMessage(void *CANptr, uint32_t ID, uint8_t DLC, uint8_t *data);

/**
 * @brief Periodic legacy can handler. Call in 100hz or 1000hz loop
 * @param CANptr pointer to can interface
 */
void CANOpenLegacyPeriodicTask(void *CANptr);

/**
 * @brief Macro for generating CANOpen configuration
 * @param name of generated Object Dictionary, for example: ODGen
 * @param NULL pointer of CO_config_t type
*/
#define CO_CREATE_OD_CONFIG(OD, configPtr) \
    configPtr = malloc(sizeof(CO_config_t)); \
    configPtr->CNT_NMT = OD##_CNT_NMT; \
    configPtr->CNT_HB_CONS = OD##_CNT_HB_CONS; \
    configPtr->CNT_ARR_1016 = OD##_CNT_ARR_1016; \
    configPtr->CNT_EM = OD##_CNT_EM; \
    configPtr->CNT_ARR_1003 = OD##_CNT_ARR_1003; \
    configPtr->CNT_SDO_SRV = OD##_CNT_SDO_SRV; \
    configPtr->CNT_SDO_CLI = OD##_CNT_SDO_CLI; \
    configPtr->CNT_TIME = OD##_CNT_TIME; \
    configPtr->CNT_SYNC = OD##_CNT_SYNC; \
    configPtr->CNT_RPDO = OD##_CNT_RPDO; \
    configPtr->CNT_TPDO = OD##_CNT_TPDO; \
    configPtr->CNT_LEDS = 0; \
    configPtr->CNT_GFC = 0; \
    configPtr->CNT_SRDO = 0; \
    configPtr->CNT_LSS_SLV = 0; \
    configPtr->CNT_LSS_MST = 0; \
    configPtr->CNT_GTWA = 0; \
    configPtr->CNT_TRACE = 0; \
    configPtr->ENTRY_H1001 = OD##_ENTRY_H1001_errorRegister; \
    configPtr->ENTRY_H1003 = OD##_ENTRY_H1003; \
    configPtr->ENTRY_H1005 = OD##_ENTRY_H1005; \
    configPtr->ENTRY_H1006 = OD##_ENTRY_H1006; \
    configPtr->ENTRY_H1007 = OD##_ENTRY_H1007; \
    configPtr->ENTRY_H1012 = OD##_ENTRY_H1012; \
    configPtr->ENTRY_H1014 = OD##_ENTRY_H1014; \
    configPtr->ENTRY_H1015 = OD##_ENTRY_H1015; \
    configPtr->ENTRY_H1016 = OD##_ENTRY_H1016; \
    configPtr->ENTRY_H1017 = OD##_ENTRY_H1017; \
    configPtr->ENTRY_H1019 = OD##_ENTRY_H1019; \
    configPtr->ENTRY_H1200 = OD##_ENTRY_H1200; \
    configPtr->ENTRY_H1400 = OD##_ENTRY_H1400; \
    configPtr->ENTRY_H1600 = OD##_ENTRY_H1600; \
    configPtr->ENTRY_H1800 = OD##_ENTRY_H1800; \
    configPtr->ENTRY_H1A00 = OD##_ENTRY_H1A00; \
    configPtr->ENTRY_H1010 = OD##_ENTRY_H1010; \
    configPtr->ENTRY_H1011 = OD##_ENTRY_H1011

#ifdef __cplusplus
}
#endif

#endif

