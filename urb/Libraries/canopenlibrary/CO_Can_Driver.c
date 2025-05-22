/*
 * CAN module object for generic microcontroller.
 *
 * This file is a template for other microcontrollers.
 *
 * @file        CO_driver.c
 * @ingroup     CO_driver
 * @author      Mario Harvan
 * @copyright   2022 - 2023 Mario Harvan
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

#if __has_include("stm32f4xx_hal.h") || __has_include("stm32f1xx_hal.h")

#include "301/CO_driver.h"
#include "CO_driver_target.h"
#include "CANopen.h"
#if __has_include("stm32f4xx_hal.h")
    #include "stm32f4xx_hal.h"
#elif __has_include("stm32f1xx_hal.h")
    #include "stm32f1xx_hal.h"
#endif
#if __has_include("stm32f4xx_hal_it.h")
  #include "stm32f4xx_hal_it.h"
#endif
#if __has_include("stm32f4xx_it.h")
  #include "stm32f4xx_it.h"
#endif
#include "Canopen_Main.h"


#define CANID_MASK                              0x07FF  /*!< CAN standard ID mask */
#define FLAG_RTR                                0x8000

extern CO_InstanceList_t COInstanceList[];
extern uint8_t COInstanceCnt;

extern CO_LegacyCanCallback_t COLegacyCallbackList[];
extern uint8_t COLegacyCanCallbackCnt;

/******************************************************************************/
void CO_CANsetConfigurationMode(void *CANptr){
  /* Put CAN module in configuration mode */
  HAL_CAN_Stop(CANptr);
}


/******************************************************************************/
void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule){
  /* Put CAN module in normal mode */
  HAL_CAN_Start(CANmodule->CANptr);
  CANmodule->CANnormal = true;
}


/******************************************************************************/
CO_ReturnError_t CO_CANmodule_init(
        CO_CANmodule_t         *CANmodule,
        void                   *CANptr,
        CO_CANrx_t              rxArray[],
        uint16_t                rxSize,
        CO_CANtx_t              txArray[],
        uint16_t                txSize,
        uint16_t                CANbitRate)
{
  uint16_t i;

  /* verify arguments */
  if(CANmodule==NULL || rxArray==NULL || txArray==NULL){
    return CO_ERROR_ILLEGAL_ARGUMENT;
  }

  /* Configure object variables */
  CANmodule->CANptr = CANptr;
  CANmodule->rxArray = rxArray;
  CANmodule->rxSize = rxSize;
  CANmodule->txArray = txArray;
  CANmodule->txSize = txSize;
  CANmodule->CANerrorStatus = 0;
  CANmodule->CANnormal = false;
  CANmodule->useCANrxFilters = rxSize > 28 ? false : true;
  CANmodule->bufferInhibitFlag = false;
  CANmodule->firstCANtxMessage = true;
  CANmodule->CANtxCount = 0U;
  CANmodule->errOld = 0U;

  for(i=0U; i<rxSize; i++){
    rxArray[i].ident = 0U;
    rxArray[i].mask = 0xFFFFU;
    rxArray[i].object = NULL;
    rxArray[i].CANrx_callback = NULL;
  }
  for(i=0U; i<txSize; i++){
    txArray[i].bufferFull = false;
  }
  
  if(CANmodule->useCANrxFilters == false)
  {
    CAN_FilterTypeDef config;
    config.FilterActivation = CAN_FILTER_ENABLE;
    config.FilterBank = 0;  // which filter bank to use from the assigned ones
    config.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    config.FilterIdLow = 0u;
    config.FilterMaskIdLow = 0u;
    config.FilterMode = CAN_FILTERMODE_IDMASK;
    config.FilterScale = CAN_FILTERSCALE_16BIT;
    config.SlaveStartFilterBank = 0;
    HAL_CAN_ConfigFilter(CANmodule->CANptr, &config);
  }

  /* Configure CAN module registers */


  /* Configure CAN timing */


  /* configure CAN interrupt registers */
  HAL_CAN_ActivateNotification(CANmodule->CANptr, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY);

  return CO_ERROR_NO;
}


/******************************************************************************/
void CO_CANmodule_disable(CO_CANmodule_t *CANmodule) {
  if (CANmodule != NULL) {
    /* turn off the module */
    HAL_CAN_Stop(CANmodule->CANptr);
  }
}

/******************************************************************************/
CO_ReturnError_t CO_CANrxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        uint16_t                mask,
        bool_t                  rtr,
        void                   *object,
        void                  (*CANrx_callback)(void *object, void *message))
{
  CO_ReturnError_t ret = CO_ERROR_NO;
  static CAN_FilterTypeDef config;
  static uint8_t filterCnt = 0;
  if((CANmodule!=NULL) && (object!=NULL) && (CANrx_callback!=NULL) && (index < CANmodule->rxSize)){
    /* buffer, which will be configured */
    CO_CANrx_t *buffer = &CANmodule->rxArray[index];

    /* Configure object variables */
    buffer->object = object;
    buffer->CANrx_callback = CANrx_callback;

    /* CAN identifier and CAN mask, bit aligned with CAN module. Different on different microcontrollers. */
    buffer->ident = ident & 0x07FFU;
    if(rtr){
      buffer->ident |= 0x0800U;
    }
    buffer->mask = (mask & 0x07FFU) | 0x0800U;

    /* Set CAN hardware module filter and mask. */
    if(CANmodule->useCANrxFilters){
        config.FilterActivation = CAN_FILTER_ENABLE;
        config.FilterBank = filterCnt / 2;  // which filter bank to use from the assigned ones
        config.FilterFIFOAssignment = CAN_FILTER_FIFO0;
        if(filterCnt % 2 == 0)
        {
            config.FilterIdLow = (buffer->ident & 0x7FF) << 5;
            config.FilterMaskIdLow = buffer->mask << 5;
            config.FilterIdHigh = 0;
            config.FilterMaskIdHigh = 0;
        }
        else 
        {
            config.FilterIdHigh = (buffer->ident & 0x7FF) << 5;
            config.FilterMaskIdHigh = buffer->mask << 5;
        }
        config.FilterMode = CAN_FILTERMODE_IDMASK;
        config.FilterScale = CAN_FILTERSCALE_16BIT;
        config.SlaveStartFilterBank = 0;
        HAL_CAN_ConfigFilter(CANmodule->CANptr, &config);
        filterCnt++;
    }
  }
  else{
    ret = CO_ERROR_ILLEGAL_ARGUMENT;
  }

  return ret;
}


/******************************************************************************/
CO_CANtx_t *CO_CANtxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        bool_t                  rtr,
        uint8_t                 noOfBytes,
        bool_t                  syncFlag)
{
  CO_CANtx_t *buffer = NULL;

  if((CANmodule != NULL) && (index < CANmodule->txSize)){
    /* get specific buffer */
    buffer = &CANmodule->txArray[index];

    /* CAN identifier, DLC and rtr, bit aligned with CAN module transmit buffer.
     * Microcontroller specific. */
    buffer->ident = ((uint32_t)ident & 0x07FFU)
                    | ((uint32_t)(((uint32_t)noOfBytes & 0xFU) << 12U))
                    | ((uint32_t)(rtr ? 0x8000U : 0U));
    buffer->DLC = noOfBytes;
    buffer->bufferFull = false;
    buffer->syncFlag = syncFlag;
  }

  return buffer;
}

bool sendCanMessage(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer){
  CANmodule->bufferInhibitFlag = buffer->syncFlag;
  /* copy message and txRequest */
  CAN_TxHeaderTypeDef   TxHeader;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.StdId = buffer->ident & CANID_MASK;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = buffer->DLC;
  uint32_t TxMailbox;

  /* Now add message to FIFO. Should not fail */
  return HAL_CAN_AddTxMessage(CANmodule->CANptr, &TxHeader, buffer->data, &TxMailbox) == HAL_OK;
}

/******************************************************************************/
CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer){
  CO_ReturnError_t err = CO_ERROR_NO;

  /* Verify overflow */
  if(buffer->bufferFull){
    if(!CANmodule->firstCANtxMessage){
      /* don't set error, if bootup message is still on buffers */
      CANmodule->CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
    }
    err = CO_ERROR_TX_OVERFLOW;
  }

  /* if CAN TX buffer is free, copy message to it */
  if(CANmodule->CANtxCount == 0){
    if(sendCanMessage(CANmodule, buffer)){
      CANmodule->bufferInhibitFlag = buffer->syncFlag;
    }
    else{
      buffer->bufferFull = true;
      CANmodule->CANtxCount++;
    }
  }
    /* if no buffer is free, message will be sent by interrupt */
  else{
    buffer->bufferFull = true;
    CANmodule->CANtxCount++;
  }

  return err;
}


/******************************************************************************/
void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule){
  uint32_t tpdoDeleted = 0U;

  CO_LOCK_CAN_SEND(CANmodule);
  /* Abort message from CAN module, if there is synchronous TPDO.
   * Take special care with this functionality. */
  if(/*messageIsOnCanBuffer && */CANmodule->bufferInhibitFlag){
    /* clear TXREQ */
    CANmodule->bufferInhibitFlag = false;
    tpdoDeleted = 1U;
  }
  /* delete also pending synchronous TPDOs in TX buffers */
  if(CANmodule->CANtxCount != 0U){
    uint16_t i;
    CO_CANtx_t *buffer = &CANmodule->txArray[0];
    for(i = CANmodule->txSize; i > 0U; i--){
      if(buffer->bufferFull){
        if(buffer->syncFlag){
          buffer->bufferFull = false;
          CANmodule->CANtxCount--;
          tpdoDeleted = 2U;
        }
      }
      buffer++;
    }
  }
  CO_UNLOCK_CAN_SEND(CANmodule);


  if(tpdoDeleted != 0U){
    CANmodule->CANerrorStatus |= CO_CAN_ERRTX_PDO_LATE;
  }
}

/******************************************************************************/
/* Get error counters from the module. If necessary, function may use
    * different way to determine errors. */

void CO_CANmodule_process(CO_CANmodule_t *CANmodule) {
    for(uint8_t i = 0; i < COInstanceCnt; i++)
    {
        if(CANmodule->CANptr == COInstanceList[i].CO->CANmodule->CANptr)
        {
            uint32_t err;
            err = HAL_CAN_GetError(CANmodule->CANptr);
            if(err != 0)
            {
                COInstanceList[i].canErrCnt++;
            }
            else
            {
                COInstanceList[i].canErrCnt = 0;
            }
            if (CANmodule->errOld != err) {
                uint16_t status = CANmodule->CANerrorStatus;

                CANmodule->errOld = err;

                if (COInstanceList[i].canErrCnt >= 256U) {
                    /* bus off */
                    status |= CO_CAN_ERRTX_BUS_OFF;
                }
                else {
                    /* recalculate CANerrorStatus, first clear some flags */
                    status &= 0xFFFF ^ (CO_CAN_ERRTX_BUS_OFF |
                                        CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE |
                                        CO_CAN_ERRTX_WARNING | CO_CAN_ERRTX_PASSIVE);

                    /* rx bus warning or passive */
                    if (COInstanceList[i].canErrCnt >= 128) {
                        status |= CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE;
                    } else if (COInstanceList[i].canErrCnt >= 96) {
                        status |= CO_CAN_ERRRX_WARNING;
                    }

                    /* tx bus warning or passive */
                    if (COInstanceList[i].canErrCnt >= 128) {
                        status |= CO_CAN_ERRTX_WARNING | CO_CAN_ERRTX_PASSIVE;
                    } else if (COInstanceList[i].canErrCnt >= 96) {
                        status |= CO_CAN_ERRTX_WARNING;
                    }

                    /* if not tx passive clear also overflow */
                    if ((status & CO_CAN_ERRTX_PASSIVE) == 0) {
                        status &= 0xFFFF ^ CO_CAN_ERRTX_OVERFLOW;
                    }
                }
                CANmodule->CANerrorStatus = status;
            }
        }
    }
}


/******************************************************************************/


/**
 * \brief           Read message from RX FIFO
 * \param           hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified FDCAN.
 * \param[in]       fifo: Fifo number to use for read
 * \param[in]       fifo_isrs: List of interrupts for respected FIFO
 */
static void canReadReceivedMsg(CAN_HandleTypeDef* hcan, uint32_t fifo) {
  static CAN_RxHeaderTypeDef rx_hdr;
  CO_CANrxMsg_t rcvMsg;
  CO_CANrx_t *buffer = NULL;              /* receive message buffer from CO_CANmodule_t object. */
  uint16_t index;                         /* index of received message */
  uint32_t rcvMsgIdent;                   /* identifier of the received message */
  uint8_t messageFound = 0;

  /* Read received message from FIFO */
  if (HAL_CAN_GetRxMessage(hcan, fifo, &rx_hdr, rcvMsg.data) != HAL_OK) {
    return;
  }

  /* Setup identifier (with RTR) and length */
  rcvMsg.ident = rx_hdr.StdId;
  rcvMsg.DLC = rx_hdr.DLC;
  rcvMsgIdent = rx_hdr.StdId;

  for(uint8_t i = 0; i < COInstanceCnt; i++)
  {
    if(COInstanceList[i].CO->CANmodule->CANptr == hcan) {
        /*
         * We are not using hardware filters, hence it is necessary
         * to manually match received message ID with all buffers
         */
        COInstanceList[i].rxMessageCounter++;
        COInstanceList[i].lastSuccRxMsgTime++;
        buffer = COInstanceList[i].CO->CANmodule->rxArray;
        for (index = COInstanceList[i].CO->CANmodule->rxSize; index > 0U; --index, ++buffer) {
          if (((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U) {
            messageFound = 1;
            break;
          }
        }
      break;
    }
  }
  for(uint8_t i = 0; i < COLegacyCanCallbackCnt; i++)
  {
      if(COLegacyCallbackList[i].CANPtr == hcan)
      {
          COLegacyCallbackList[i].receiveMsgCallback(rcvMsg);
          break;
      }
  }


  /* Call specific function, which will process the message */
  if (messageFound && buffer != NULL && buffer->CANrx_callback != NULL) {
    buffer->CANrx_callback(buffer->object, (void*) &rcvMsg);
  }
}

/**
 * \brief           Rx FIFO 0 callback.
 * \param[in]       hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified CAN.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    canReadReceivedMsg(hcan, CAN_RX_FIFO0);
}

/**
 * \brief           Rx FIFO 1 callback.
 * \param[in]       hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified CAN.
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    canReadReceivedMsg(hcan, CAN_RX_FIFO1);
}


/**
 * \brief           TX buffer has been well transmitted callback
 * \param[in]       hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified FDCAN.
 * \param[in]       BufferIndexes: Bits of successfully sent TX buffers
 */
void CanTransmissionComplete(CAN_HandleTypeDef *hcan)
{
    for(uint8_t j = 0; j < COInstanceCnt; j++)
    {
        if(COInstanceList[j].CO->CANmodule->CANptr == hcan){
            COInstanceList[j].CO->CANmodule->firstCANtxMessage = false; // First CAN message (bootup) was sent successfully
            COInstanceList[j].CO->CANmodule->bufferInhibitFlag = false; // Clear flag from previous message
            COInstanceList[j].lastSuccTxMsgTime = HAL_GetTick();
            COInstanceList[j].txMessageCounter++;
            if (COInstanceList[j].CO->CANmodule->CANtxCount > 0U) {     // Are there any new messages waiting to be send
                CO_CANtx_t *buffer = &COInstanceList[j].CO->CANmodule->txArray[0];  // Start with first buffer handle
                uint16_t i;

                // Try to send more buffers, process all empty ones
                //
                // This function is always called from interrupt,
                // however to make sure no preemption can happen, interrupts are anyway locked
                // (unless you can guarantee no higher priority interrupt will try to access to FDCAN instance and send data,
                //  then no need to lock interrupts..)


                for (i = COInstanceList[j].CO->CANmodule->txSize; i > 0U; --i, ++buffer) {
                    // Try to send message

                    if (buffer->bufferFull) {
                        if (sendCanMessage(COInstanceList[j].CO->CANmodule, buffer))
                        {
                            buffer->bufferFull = false;
                            COInstanceList[j].CO->CANmodule->CANtxCount--;
                            COInstanceList[j].CO->CANmodule->bufferInhibitFlag = buffer->syncFlag;
                        }
                        else{
                            return;
                        }
                    }
                }
                // Clear counter if no more messages
                if (i == 0U) {
                    COInstanceList[j].CO->CANmodule->CANtxCount = 0U;
                }
            }
            break;
        }
    }
    for(uint8_t i = 0; i < COLegacyCanCallbackCnt; i++)
    {
        if(COLegacyCallbackList[i].CANPtr == hcan)
        {
            CO_legacyTxCpltCallback(hcan);
        }
    }
}

void CO_System_reset()
{
    HAL_NVIC_SystemReset();
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
    CanTransmissionComplete(hcan);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {
    CanTransmissionComplete(hcan);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) {
    CanTransmissionComplete(hcan);
}

void CO_legacyCanInit(void *CANPtr)
{
    CAN_FilterTypeDef config;
    config.FilterActivation = CAN_FILTER_ENABLE;
    config.FilterBank = 0;  // which filter bank to use from the assigned ones
    config.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    config.FilterIdLow = 0u;
    config.FilterMaskIdLow = 0u;
    config.FilterMode = CAN_FILTERMODE_IDMASK;
    config.FilterScale = CAN_FILTERSCALE_16BIT;
    config.SlaveStartFilterBank = 0;
    HAL_CAN_ConfigFilter(CANPtr, &config);

    HAL_CAN_ActivateNotification(
            CANPtr, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY);
    HAL_StatusTypeDef stat = HAL_CAN_Start(CANPtr);
}

void CO_legacyCanExceptionHandler(void *CANPtr)
{
    for(uint8_t i = 0; i < COLegacyCanCallbackCnt; i++)
    {
        if(CANPtr == COLegacyCallbackList[i].CANPtr)
        {
            uint32_t err = HAL_CAN_GetError(CANPtr);
            if (err) {
                COLegacyCallbackList[i].canErrCnt++;
            }
            if (COLegacyCallbackList[i].canErrCnt > 100) {
                if(HAL_GetTick() - COLegacyCallbackList[i].lastCanResetTime > CO_CAN_RESET_TIMEOUT)
                {
                    HAL_CAN_Stop(CANPtr);
                    HAL_CAN_Start(CANPtr);
                    COLegacyCallbackList[i].canErrCnt = 0;
                    COLegacyCallbackList[i].lastCanResetTime = HAL_GetTick();
                }
            }
        }
    }
}

bool CO_legacyCanSendMessage(void *CANptr, uint32_t ID, uint8_t DLC, uint8_t *data)
{
    for(uint8_t i = 0; i < COLegacyCanCallbackCnt; i++) {
        if(CANptr == COLegacyCallbackList[i].CANPtr)
        {
            CAN_TxHeaderTypeDef TxHeader;
            TxHeader.IDE = CAN_ID_STD;
            TxHeader.StdId = ID;
            TxHeader.RTR = CAN_RTR_DATA;
            TxHeader.DLC = DLC;
            uint32_t TxMailbox;

            HAL_StatusTypeDef ret = HAL_CAN_AddTxMessage(CANptr, &TxHeader, data, &TxMailbox);

            if(ret == HAL_OK)
            {
                return true;
            }
            COLegacyCallbackList[i].txBuffer[COLegacyCallbackList[i].txMsgBufferStart].syncFlag = 0;
            COLegacyCallbackList[i].txBuffer[COLegacyCallbackList[i].txMsgBufferStart].ident = ID;
            COLegacyCallbackList[i].txBuffer[COLegacyCallbackList[i].txMsgBufferStart].DLC = DLC;
            memcpy(COLegacyCallbackList[i].txBuffer[COLegacyCallbackList[i].txMsgBufferStart].data, data, DLC);

            uint32_t cnt = COLegacyCallbackList[i].txMsgBufferEnd <= COLegacyCallbackList[i].txMsgBufferStart ?
                           COLegacyCallbackList[i].txMsgBufferStart - COLegacyCallbackList[i].txMsgBufferEnd :
                           COLegacyCallbackList[i].txMsgBufferStart + (CO_LEGACY_CAN_TX_BUFFER_SIZE - COLegacyCallbackList[i].txMsgBufferEnd);
            // Check if buffer has empty space
            if(cnt + 1 < CO_LEGACY_CAN_TX_BUFFER_SIZE)
            {
                COLegacyCallbackList[i].txMsgBufferStart = (COLegacyCallbackList[i].txMsgBufferStart + 1) % CO_LEGACY_CAN_TX_BUFFER_SIZE;
                return true;
            }
            return false;
        }
    }
    return false;
}

void CO_legacyTxCpltCallback(void *CANPtr)
{
    for(uint8_t i = 0; i < COLegacyCanCallbackCnt; i++) {
        if(COLegacyCallbackList[i].CANPtr == CANPtr)
        {
            uint32_t cnt = COLegacyCallbackList[i].txMsgBufferEnd <= COLegacyCallbackList[i].txMsgBufferStart ?
                           COLegacyCallbackList[i].txMsgBufferStart - COLegacyCallbackList[i].txMsgBufferEnd :
                           COLegacyCallbackList[i].txMsgBufferStart +
                           (CO_LEGACY_CAN_TX_BUFFER_SIZE - COLegacyCallbackList[i].txMsgBufferEnd);

            for (int j = 0; j < cnt; j++) {
                uint32_t index = COLegacyCallbackList[i].txMsgBufferEnd;

                CAN_TxHeaderTypeDef TxHeader;
                TxHeader.IDE = CAN_ID_STD;
                TxHeader.StdId = COLegacyCallbackList[i].txBuffer[index].ident & 0x07FF;;
                TxHeader.RTR = CAN_RTR_DATA;
                TxHeader.DLC = COLegacyCallbackList[i].txBuffer[index].DLC;
                uint32_t TxMailbox;

                HAL_StatusTypeDef ret = HAL_CAN_AddTxMessage(CANPtr, &TxHeader, COLegacyCallbackList[i].txBuffer[index].data, &TxMailbox);

                if (ret != HAL_OK) {
                    return;
                }
                COLegacyCallbackList[i].txMsgBufferEnd = (COLegacyCallbackList[i].txMsgBufferEnd + 1) % CO_LEGACY_CAN_TX_BUFFER_SIZE;
            }
        }
    }
}

#endif