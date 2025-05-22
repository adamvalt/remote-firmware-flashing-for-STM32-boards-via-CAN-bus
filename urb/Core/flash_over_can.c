/**
 * @file flash_over_can.c
 * @brief Implementation of firmware flashing functionality over CAN bus using SDO protocol
 * 
 * This module provides functionality to flash firmware to target devices over CAN bus
 * using the CANopen SDO (Service Data Object) protocol. It handles the complete flashing
 * process including metadata transfer (size and CRC) and the actual firmware binary transfer.
 */

#include "flash_over_can.h"
#include "globals.h"
#include "remote_reboot.h"

// Bootloader Object Dictionary indices for firmware-related parameters
// this depends on the bootloader OD.h/.c generated config
#define BOOTLOADER_OD_INDEX_FIRMWARE_BINARY 0x6003
#define BOOTLOADER_OD_INDEX_FIRMWARE_SIZE   0x6001 
#define BOOTLOADER_OD_INDEX_FIRMWARE_CRC    0x6000  

/**
 * @struct can_flashing_t
 * @brief Structure to track the state of CAN flashing operation
 */
typedef struct {
    int SDO_ret_value;              ///< Return value from SDO operations
    bool SDO_client_error_occured;  ///< Flag indicating if an SDO error occurred
    bool is_buffer_partial;         ///< Flag indicating if the current buffer is partial
    int bytes_written;              ///< Number of bytes written in current flashing operation
} can_flashing_t;

can_flashing_t can_flashing = {
        .SDO_ret_value = 0,
        .SDO_client_error_occured = false,
        .is_buffer_partial = true,
        .bytes_written = 0,
};

/**
 * @brief Updates the current flashing state
 * @param state New flashing state to set
 */
void set_flashing_state(flashing_state state) {
    s_flashing.state = state;
}

/**
 * @brief Initializes an SDO write operation
 * @param SDO_C Pointer to SDO client structure
 * @param node_id Target CANopennode ID
 * @param index Object Dictionary index
 * @param sub_index Object Dictionary sub-index
 * @param data_size Size of data to be written
 * @param block_enable Whether to enable block transfer
 * @return CO_SDO_abortCode_t Abort code indicating success or failure
 */
CO_SDO_abortCode_t SDO_write_init(CO_SDOclient_t *SDO_C, uint8_t node_id,
                                  uint16_t index, uint8_t sub_index,
                                  size_t data_size, bool block_enable) {
    can_flashing.SDO_ret_value = CO_SDOclient_setup(SDO_C,
                                                    CO_CAN_ID_SDO_CLI + node_id,
                                                    CO_CAN_ID_SDO_SRV + node_id,
                                                    node_id);
    if (can_flashing.SDO_ret_value != CO_SDO_RT_ok_communicationEnd) {
        MG_ERROR(("SDO client setup failed for node: %d", node_id));
        return CO_SDO_AB_GENERAL;
    }

    // initiate download
    can_flashing.SDO_ret_value = CO_SDOclientDownloadInitiate(SDO_C, index, sub_index,
                                                              data_size, 1000, block_enable);

    if (can_flashing.SDO_ret_value != CO_SDO_RT_ok_communicationEnd) {
        MG_ERROR(("SDO download initiation failed for node: %d", node_id));
        return CO_SDO_AB_GENERAL;
    }
    return CO_SDO_AB_NONE;
}

/**
 * @brief Handles repeated SDO write operations for large data transfers
 * @param SDO_C Pointer to SDO client structure
 * @return CO_SDO_abortCode_t Abort code indicating success or failure
 */
CO_SDO_abortCode_t SDO_write_repeated_part(CO_SDOclient_t *SDO_C) {
    uint8_t const *data = (const uint8_t *) MG_FILESAVE_FLASH_ADDRESS;

    if (can_flashing.SDO_ret_value >= 0 && !can_flashing.SDO_client_error_occured) {
        // depends on the current loop HZ value, multiply ms by 1000 to get us
        uint32_t timeDifference_us = REPEATED_SDO_DELTA_TIME_MS * 1000;

        CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;

        if (can_flashing.bytes_written + 7 >= SDO_C->sizeInd) {
            // this ensures correct only last n bytes are written to the message buffer and no more
            can_flashing.is_buffer_partial = false;
            int rest = SDO_C->sizeInd - can_flashing.bytes_written;
            can_flashing.bytes_written += CO_SDOclientDownloadBufWrite(SDO_C, &data[can_flashing.bytes_written], rest);
        } else {
            can_flashing.bytes_written += CO_SDOclientDownloadBufWrite(SDO_C, &data[can_flashing.bytes_written], 7);
        }

        can_flashing.SDO_ret_value = CO_SDOclientDownload(SDO_C,
                                                          timeDifference_us,
                                                          false,
                                                          can_flashing.is_buffer_partial,
                                                          &abortCode,
                                                          NULL, NULL);

        if (can_flashing.SDO_ret_value < 0 && can_flashing.bytes_written == s_flashing.size) {
            s_flashing.state = FLASHING_COMPLETED;
            MG_DEBUG(("sdo end: %d finished: %d?", can_flashing.SDO_ret_value, SDO_C->finished));
            CO_SDOclientClose(SDO_C);
        } else if (can_flashing.SDO_ret_value < 0) {
            can_flashing.SDO_client_error_occured = true;
            s_flashing.state = NOT_FLASHING;
            s_flashing.progress = 0;
            can_flashing.bytes_written = 0;
            strcpy(s_flashing.error, "SDO transfer error");
            glue_update_state();
            MG_DEBUG(("sdo end: %d finished: %d?, abortcode: %d", can_flashing.SDO_ret_value, SDO_C->finished, abortCode));
            CO_SDOclientClose(SDO_C);
            return abortCode;
        }

        if (can_flashing.SDO_ret_value == 0) {
            s_flashing.state = FLASHING_COMPLETED;
            MG_DEBUG(("sdo end: %d finished: %d?", can_flashing.SDO_ret_value, SDO_C->finished));
            CO_SDOclientClose(SDO_C);
        }
    }
    return 0;
}

/**
 * @brief Performs an expedited SDO write operation (for small data, max length 4 bytes)
 * @param SDO_C Pointer to SDO client structure
 * @param data Array of 4 bytes to write
 * @param OD_index Object Dictionary index
 * @param OD_subindex Object Dictionary sub-index
 * @param target_node_id Target node ID
 */
void SDO_write_expedited(CO_SDOclient_t *SDO_C, uint8_t data[4], uint16_t OD_index, uint8_t OD_subindex, uint8_t target_node_id) {
    CO_SDO_return_t ret;
    CO_SDO_abortCode_t abort_code;

    SDO_write_init(
            SDO_C,
            target_node_id,
            OD_index,
            OD_subindex,
            4,
            false);

    CO_SDOclientDownloadBufWrite(SDO_C, data, 4);

    ret = CO_SDOclientDownload(SDO_C,
                         1000, // does not matter in this case
                         false,
                         false,
                         &abort_code,
                         NULL, NULL);

    if (abort_code != CO_SDO_AB_NONE || ret < 0) {
        MG_ERROR(("Expedited transfer did not work. Abort: %d, ret: %d", abort_code, ret));
    }

    CO_SDOclientClose(SDO_C);
}

void begin_can_flash() {
    CO_SDO_abortCode_t abortCode;

    int board_index = board_name_to_board_index(s_flashing.selected_board);
    int target_node_id = s_boards[board_index].canopen_id;
    CO_t *CO = s_boards[board_index].canopen_variant;

    MG_DEBUG(("CAN FLASH BEGIN: id 0x%X", target_node_id));

    // reset error
    strcpy(s_flashing.error, "");
    can_flashing.bytes_written = 0;
    can_flashing.SDO_ret_value = 0;
    can_flashing.SDO_client_error_occured = false;
    can_flashing.is_buffer_partial = true;
    can_flashing.bytes_written = 0;

    union ui32_to_ui8 {
        uint32_t ui32;
        uint8_t ui8[4];
    };

    // Sending metadata
    // Size
    union ui32_to_ui8 size;
    size.ui32 = s_flashing.size;
    MG_DEBUG(("Sending firmware size: %d bytes", size.ui32));
    SDO_write_expedited(CO->SDOclient, size.ui8, BOOTLOADER_OD_INDEX_FIRMWARE_SIZE, 0, target_node_id);

    // Crc
    union ui32_to_ui8 crc;
    crc.ui32 = s_flashing.crc;
    MG_DEBUG(("Sending firmware CRC: 0x%08X", crc.ui32));
    SDO_write_expedited(CO->SDOclient, crc.ui8, BOOTLOADER_OD_INDEX_FIRMWARE_CRC, 0, target_node_id);

    // Sending the file
    MG_DEBUG(("Initiating SDO transfer for firmware binary (index: 0x%04X)", BOOTLOADER_OD_INDEX_FIRMWARE_BINARY));
    abortCode = SDO_write_init(
            CO->SDOclient,
            target_node_id,
            BOOTLOADER_OD_INDEX_FIRMWARE_BINARY,
            0,
            s_flashing.size,
            true);

    if (abortCode == CO_SDO_AB_NONE) {
        s_flashing.state = FLASHING_IN_PROGRESS; // this allows periodic process to run SDO repeatedly
        s_flashing.is_active = true;
        s_flashing.progress = 0;
        MG_DEBUG(("SDO transfer initiated successfully"));
    } else {
        strcpy(s_flashing.error, "SDO start error");
        s_flashing.state = NOT_FLASHING;
        s_flashing.progress = 0;
        s_flashing.is_active = false;
        MG_ERROR(("Node with id: %d has not started SDO communication successfully. Abort code: 0x%08X", target_node_id, abortCode));
    }
    glue_update_state();
}

void flashing_start() {
    if (s_flashing.state != NOT_FLASHING && s_flashing.state != FLASHING_COMPLETED) {
        MG_INFO(("Incorrect flashing state, should be 'NOT FLASHING', actual state: %d", s_flashing.state));
        return;
    }

    if (s_flashing.size > MG_FILESAVE_FLASH_SIZE) {
        s_flashing.state = NOT_FLASHING;
        strcpy(s_flashing.error, "File too big.");
        glue_update_state();
        return;
    }

    int board_index = board_name_to_board_index(s_flashing.selected_board);
    struct board board = s_boards[board_index];

    s_flashing.state = NODE_RESETTING;
    bool ok = remote_reboot(board_index);
    if (ok) {
        MG_INFO(("Node with id: %d has been reset and flashing started", board.canopen_id));
    } else {
        s_flashing.state = FLASHING_FAILED;
        strcpy(s_flashing.error, "Rebooting node to bootloader has failed");
        return;
    }

    mg_timer_add(&g_mgr,
                 NODE_REBOOT_MINIMUM_TIME_MS,
                 MG_TIMER_ONCE,
                 (void (*)(void *)) set_flashing_state,
                 (void *) NODE_IN_BOOTLOADER);  // Run set_flashing_state after NODE_RESET_MINIMUM_TIME_MS ms delay

    mg_timer_add(&g_mgr,
                 NODE_REBOOT_MINIMUM_TIME_MS + 200,
                 MG_TIMER_ONCE,
                 (void (*)(void *)) begin_can_flash,
                 NULL);
}

void periodic_can_flash() {
    if (s_flashing.state != FLASHING_IN_PROGRESS) {
        return;
    }
    int board_index = board_name_to_board_index(s_flashing.selected_board);

    SDO_write_repeated_part(s_boards[board_index].canopen_variant->SDOclient);
    s_flashing.progress = (can_flashing.bytes_written * 100) / s_flashing.size; // get percentage
    MG_DEBUG(
            ("PERIOD %d %d %d %p", s_flashing.progress, can_flashing.bytes_written,
                    s_flashing.size, s_boards[board_index].canopen_variant->SDOclient));
    glue_update_state(); // needed to show progress on the screen
}