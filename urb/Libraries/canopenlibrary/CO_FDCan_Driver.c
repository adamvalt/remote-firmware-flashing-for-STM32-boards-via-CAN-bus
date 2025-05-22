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

#if __has_include("stm32g4xx_hal_fdcan.h") || __has_include("stm32h7xx_hal_fdcan.h")

#include "301/CO_driver.h"
#include "CO_driver_target.h"
#include "fdcan.h"
#include "CANopen.h"
#include "Canopen_Main.h"

#if __has_include("stm32g4xx_hal_fdcan.h")
    #include "stm32g4xx_hal_fdcan.h"
    #include "stm32g4xx_hal.h"
    #include "stm32g4xx_it.h"
#endif
#if __has_include("stm32h7xx_hal_fdcan.h")
    #include "stm32h7xx_hal_fdcan.h"
    #include "stm32h7xx_hal.h"
    #include "stm32h7xx_it.h"
#endif

#define CANID_MASK                              0x07FF  /*!< CAN standard ID mask */
#define FLAG_RTR                                0x8000

extern CO_InstanceList_t COInstanceList[];
extern uint8_t COInstanceCnt;

extern CO_LegacyCanCallback_t COLegacyCallbackList[];
extern uint8_t COLegacyCanCallbackCnt;

/******************************************************************************/
void CO_CANsetConfigurationMode(void *CANptr){
  /* Put CAN module in configuration mode */
  HAL_FDCAN_Stop(CANptr);
}


/******************************************************************************/
void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule){
  /* Put CAN module in normal mode */
  HAL_FDCAN_Start(CANmodule->CANptr);
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


  /* Configure CAN module registers */


  /* Configure CAN timing */


  /* Configure CAN module hardware filters */
  if(CANmodule->useCANrxFilters){
    /* CAN module filters are used, they will be configured with */
    /* CO_CANrxBufferInit() functions, called by separate CANopen */
    /* init functions. */
    /* Configure all masks so, that received message must match filter */
    HAL_FDCAN_ConfigGlobalFilter(CANmodule->CANptr,
                                 FDCAN_REJECT, FDCAN_REJECT,
                                 FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  }
  else{
    /* CAN module filters are not used, all messages with standard 11-bit */
    /* identifier will be received */
    /* Configure mask 0 so, that all messages with standard identifier are accepted */
    HAL_FDCAN_ConfigGlobalFilter(CANmodule->CANptr,
                                  FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT,
                                  FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  }


  /* configure CAN interrupt registers */
  HAL_FDCAN_ActivateNotification(CANmodule->CANptr,
                                 0
                                 | FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE
                                 | FDCAN_IT_TX_COMPLETE | FDCAN_IT_TX_FIFO_EMPTY
                                 | FDCAN_IT_BUS_OFF
                                 | FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR
                                 | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING, 0xFFFFFFFF);

  return CO_ERROR_NO;
}


/******************************************************************************/
void CO_CANmodule_disable(CO_CANmodule_t *CANmodule) {
  if (CANmodule != NULL) {
    /* turn off the module */
    HAL_FDCAN_Stop(CANmodule->CANptr);
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
      FDCAN_FilterTypeDef filter;
      filter.IdType = FDCAN_STANDARD_ID;
      filter.FilterType = FDCAN_FILTER_MASK;
      filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
      filter.FilterID1 = buffer->ident;
      filter.FilterID2 = buffer->mask;
      filter.FilterIndex = index;
      HAL_FDCAN_ConfigFilter(CANmodule->CANptr, &filter);
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

bool sendCanMessage(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer)
{
  /* if CAN TX buffer is free, copy message to it */

    CANmodule->bufferInhibitFlag = buffer->syncFlag;
    /* copy message and txRequest */
    FDCAN_TxHeaderTypeDef tx_hdr;
    tx_hdr.Identifier = buffer->ident & CANID_MASK;
    tx_hdr.TxFrameType = FDCAN_DATA_FRAME;//(buffer->ident & FLAG_RTR) ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
    tx_hdr.IdType = FDCAN_STANDARD_ID;
    tx_hdr.FDFormat = FDCAN_CLASSIC_CAN;
    tx_hdr.BitRateSwitch = FDCAN_BRS_OFF;
    tx_hdr.MessageMarker = 0;
    tx_hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    switch (buffer->DLC) {
      case 0: tx_hdr.DataLength = FDCAN_DLC_BYTES_0; break;
      case 1: tx_hdr.DataLength = FDCAN_DLC_BYTES_1; break;
      case 2: tx_hdr.DataLength = FDCAN_DLC_BYTES_2; break;
      case 3: tx_hdr.DataLength = FDCAN_DLC_BYTES_3; break;
      case 4: tx_hdr.DataLength = FDCAN_DLC_BYTES_4; break;
      case 5: tx_hdr.DataLength = FDCAN_DLC_BYTES_5; break;
      case 6: tx_hdr.DataLength = FDCAN_DLC_BYTES_6; break;
      case 7: tx_hdr.DataLength = FDCAN_DLC_BYTES_7; break;
      case 8: tx_hdr.DataLength = FDCAN_DLC_BYTES_8; break;
      default: /* Hard error... */ break;
    }

    /* Now add message to FIFO. Should not fail */
    return HAL_FDCAN_AddMessageToTxFifoQ(CANmodule->CANptr, &tx_hdr, buffer->data) == HAL_OK;
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

  CO_LOCK_CAN_SEND(CANmodule);
  if(CANmodule->CANtxCount == 0) {
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
  CO_UNLOCK_CAN_SEND(CANmodule);

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
static uint16_t rxErrors=0, txErrors=0, overflow=0;

void CO_CANmodule_process(CO_CANmodule_t *CANmodule) {
    uint32_t err;

    FDCAN_ErrorCountersTypeDef errCounters;

    HAL_FDCAN_GetErrorCounters((FDCAN_HandleTypeDef *)CANmodule->CANptr, &errCounters);

    txErrors = errCounters.TxErrorCnt;
    rxErrors = errCounters.RxErrorCnt;

    err = ((uint32_t)txErrors << 16) | ((uint32_t)rxErrors << 8) | overflow;
    if (CANmodule->errOld != err) {
        uint16_t status = CANmodule->CANerrorStatus;

        CANmodule->errOld = err;

        if (txErrors >= 256U) {
            /* bus off */
            status |= CO_CAN_ERRTX_BUS_OFF;
        }
        else {
          /* recalculate CANerrorStatus, first clear some flags */
          status &= 0xFFFF ^ (CO_CAN_ERRTX_BUS_OFF |
                              CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE |
                              CO_CAN_ERRTX_WARNING | CO_CAN_ERRTX_PASSIVE);

          /* rx bus warning or passive */
          if (rxErrors >= 128) {
            status |= CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE;
          } else if (rxErrors >= 96) {
            status |= CO_CAN_ERRRX_WARNING;
          }

          /* tx bus warning or passive */
          if (txErrors >= 128) {
            status |= CO_CAN_ERRTX_WARNING | CO_CAN_ERRTX_PASSIVE;
          } else if (rxErrors >= 96) {
            status |= CO_CAN_ERRTX_WARNING;
          }

          /* if not tx passive clear also overflow */
          if ((status & CO_CAN_ERRTX_PASSIVE) == 0) {
            status &= 0xFFFF ^ CO_CAN_ERRTX_OVERFLOW;
          }
        }

        if (overflow != 0) {
          /* CAN RX bus overflow */
          status |= CO_CAN_ERRRX_OVERFLOW;
        }

        CANmodule->CANerrorStatus = status;
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
static void readCanReceivedMsg(FDCAN_HandleTypeDef* hfdcan, uint32_t fifo, uint32_t fifo_isrs) {
  static FDCAN_RxHeaderTypeDef rx_hdr;
  CO_CANrxMsg_t rcvMsg;
  CO_CANrx_t *buffer = NULL;              /* receive message buffer from CO_CANmodule_t object. */
  uint16_t index;                         /* index of received message */
  uint32_t rcvMsgIdent;                   /* identifier of the received message */
  uint8_t messageFound = 0;

  /* Read received message from FIFO */
  if (HAL_FDCAN_GetRxMessage(hfdcan, fifo, &rx_hdr, rcvMsg.data) != HAL_OK) {
    return;
  }

  /* Setup identifier (with RTR) and length */
  rcvMsg.ident = rx_hdr.Identifier | (rx_hdr.RxFrameType == FDCAN_REMOTE_FRAME ? FLAG_RTR : 0x00);
  switch (rx_hdr.DataLength) {
    case FDCAN_DLC_BYTES_0: rcvMsg.DLC = 0; break;
    case FDCAN_DLC_BYTES_1: rcvMsg.DLC = 1; break;
    case FDCAN_DLC_BYTES_2: rcvMsg.DLC = 2; break;
    case FDCAN_DLC_BYTES_3: rcvMsg.DLC = 3; break;
    case FDCAN_DLC_BYTES_4: rcvMsg.DLC = 4; break;
    case FDCAN_DLC_BYTES_5: rcvMsg.DLC = 5; break;
    case FDCAN_DLC_BYTES_6: rcvMsg.DLC = 6; break;
    case FDCAN_DLC_BYTES_7: rcvMsg.DLC = 7; break;
    case FDCAN_DLC_BYTES_8: rcvMsg.DLC = 8; break;
    default: rcvMsg.DLC = 0; break;     /* Invalid length when more than 8 */
  }
  rcvMsgIdent = rcvMsg.ident;

  for(uint8_t i = 0; i < COInstanceCnt; i++)
  {
    if(COInstanceList[i].CO->CANmodule->CANptr == hfdcan) {
        /*
         * We are not using hardware filters, hence it is necessary
         * to manually match received message ID with all buffers
         */
        COInstanceList[i].rxMessageCounter++;
        COInstanceList[i].lastSuccRxMsgTime = HAL_GetTick();
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
      if(COLegacyCallbackList[i].CANPtr == hfdcan)
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
 *                      the configuration information for the specified FDCAN.
 * \param[in]       RxFifo0ITs: indicates which Rx FIFO 0 interrupts are signaled.
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs) {
  if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
    readCanReceivedMsg(hfdcan, FDCAN_RX_FIFO0, RxFifo0ITs);
  }
}

/**
 * \brief           Rx FIFO 1 callback.
 * \param[in]       hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified FDCAN.
 * \param[in]       RxFifo1ITs: indicates which Rx FIFO 0 interrupts are signaled.
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo1ITs) {
  if (RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) {
    readCanReceivedMsg(hfdcan, FDCAN_RX_FIFO1, RxFifo1ITs);
  }
}

/**
 * \brief           TX buffer has been well transmitted callback
 * \param[in]       hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified FDCAN.
 * \param[in]       BufferIndexes: Bits of successfully sent TX buffers
 */
void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes) {
  for(uint8_t j = 0; j < COInstanceCnt; j++)
  {
    if(COInstanceList[j].CO->CANmodule->CANptr == hfdcan){
      COInstanceList[j].CO->CANmodule->firstCANtxMessage = false; /* First CAN message (bootup) was sent successfully */
      COInstanceList[j].CO->CANmodule->bufferInhibitFlag = false; /* Clear flag from previous message */
      COInstanceList[j].txMessageCounter++;
      COInstanceList[j].lastSuccTxMsgTime = HAL_GetTick();
      if (COInstanceList[j].CO->CANmodule->CANtxCount > 0U) {     /* Are there any new messages waiting to be send */
        CO_CANtx_t *buffer = &COInstanceList[j].CO->CANmodule->txArray[0];  /* Start with first buffer handle */
        uint16_t i;

        /*
         * Try to send more buffers, process all empty ones
         *
         * This function is always called from interrupt,
         * however to make sure no preemption can happen, interrupts are anyway locked
         * (unless you can guarantee no higher priority interrupt will try to access to FDCAN instance and send data,
         *  then no need to lock interrupts..)
         */

        for (i = COInstanceList[j].CO->CANmodule->txSize; i > 0U; --i, ++buffer) {
          /* Try to send message */
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
        /* Clear counter if no more messages */
        if (i == 0U) {
          COInstanceList[j].CO->CANmodule->CANtxCount = 0U;
        }
      }
      break;
    }
  }
  for(uint8_t i = 0; i < COLegacyCanCallbackCnt; i++)
  {
      if(COLegacyCallbackList[i].CANPtr == hfdcan)
      {
          CO_legacyTxCpltCallback(hfdcan);
      }
  }
}

void CO_System_reset()
{
    HAL_NVIC_SystemReset();
}

void CO_legacyCanInit(void *CANPtr)
{
    HAL_FDCAN_ConfigGlobalFilter(CANPtr, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT,FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ActivateNotification(CANPtr, 0 | FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE | FDCAN_IT_TX_COMPLETE | FDCAN_IT_TX_FIFO_EMPTY | FDCAN_IT_BUS_OFF | FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING, 0xFFFFFFFF);
    HAL_StatusTypeDef stat = HAL_FDCAN_Start(CANPtr);
}

void CO_legacyCanExceptionHandler(void *CANPtr)
{
    for(uint8_t i = 0; i < COLegacyCanCallbackCnt; i++) {
        if(CANPtr == COLegacyCallbackList[i].CANPtr) {
            FDCAN_ErrorCountersTypeDef errCounters;
            HAL_FDCAN_GetErrorCounters(CANPtr, &errCounters);

            uint16_t txErrors = errCounters.TxErrorCnt;
            uint16_t rxErrors = errCounters.RxErrorCnt;
            if (txErrors > 100 || rxErrors > 100) {
                if(HAL_GetTick() - COLegacyCallbackList[i].lastCanResetTime > CO_CAN_RESET_TIMEOUT)
                {
                    HAL_FDCAN_Stop(CANPtr);
                    HAL_FDCAN_Start(CANPtr);
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
            FDCAN_TxHeaderTypeDef tx_hdr;
            tx_hdr.Identifier = ID & 0x07FF;
            tx_hdr.TxFrameType = FDCAN_DATA_FRAME;
            tx_hdr.IdType = FDCAN_STANDARD_ID;
            tx_hdr.FDFormat = FDCAN_CLASSIC_CAN;
            tx_hdr.BitRateSwitch = FDCAN_BRS_OFF;
            tx_hdr.MessageMarker = 0;
            tx_hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
            tx_hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

            switch (DLC) {
                case 0:
                    tx_hdr.DataLength = FDCAN_DLC_BYTES_0;
                    break;
                case 1:
                    tx_hdr.DataLength = FDCAN_DLC_BYTES_1;
                    break;
                case 2:
                    tx_hdr.DataLength = FDCAN_DLC_BYTES_2;
                    break;
                case 3:
                    tx_hdr.DataLength = FDCAN_DLC_BYTES_3;
                    break;
                case 4:
                    tx_hdr.DataLength = FDCAN_DLC_BYTES_4;
                    break;
                case 5:
                    tx_hdr.DataLength = FDCAN_DLC_BYTES_5;
                    break;
                case 6:
                    tx_hdr.DataLength = FDCAN_DLC_BYTES_6;
                    break;
                case 7:
                    tx_hdr.DataLength = FDCAN_DLC_BYTES_7;
                    break;
                case 8:
                    tx_hdr.DataLength = FDCAN_DLC_BYTES_8;
                    break;
                default: /* Hard error... */ break;
            }
            HAL_StatusTypeDef ret = HAL_FDCAN_AddMessageToTxFifoQ(CANptr, &tx_hdr, data);
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
                FDCAN_TxHeaderTypeDef tx_hdr;
                tx_hdr.Identifier = COLegacyCallbackList[i].txBuffer[index].ident & 0x07FF;
                tx_hdr.TxFrameType = FDCAN_DATA_FRAME;//(buffer->ident & FLAG_RTR) ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
                tx_hdr.IdType = FDCAN_STANDARD_ID;
                tx_hdr.FDFormat = FDCAN_CLASSIC_CAN;
                tx_hdr.BitRateSwitch = FDCAN_BRS_OFF;
                tx_hdr.MessageMarker = 0;
                tx_hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
                tx_hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

                switch (COLegacyCallbackList[i].txBuffer[index].DLC) {
                    case 0:
                        tx_hdr.DataLength = FDCAN_DLC_BYTES_0;
                        break;
                    case 1:
                        tx_hdr.DataLength = FDCAN_DLC_BYTES_1;
                        break;
                    case 2:
                        tx_hdr.DataLength = FDCAN_DLC_BYTES_2;
                        break;
                    case 3:
                        tx_hdr.DataLength = FDCAN_DLC_BYTES_3;
                        break;
                    case 4:
                        tx_hdr.DataLength = FDCAN_DLC_BYTES_4;
                        break;
                    case 5:
                        tx_hdr.DataLength = FDCAN_DLC_BYTES_5;
                        break;
                    case 6:
                        tx_hdr.DataLength = FDCAN_DLC_BYTES_6;
                        break;
                    case 7:
                        tx_hdr.DataLength = FDCAN_DLC_BYTES_7;
                        break;
                    case 8:
                        tx_hdr.DataLength = FDCAN_DLC_BYTES_8;
                        break;
                    default: /* Hard error... */ break;
                }
                HAL_StatusTypeDef ret = HAL_FDCAN_AddMessageToTxFifoQ(CANPtr, &tx_hdr, COLegacyCallbackList[i].txBuffer[index].data);

                if (ret != HAL_OK) {
                    return;
                }
                COLegacyCallbackList[i].txMsgBufferEnd = (COLegacyCallbackList[i].txMsgBufferEnd + 1) % CO_LEGACY_CAN_TX_BUFFER_SIZE;
            }
        }
    }
}

#endif