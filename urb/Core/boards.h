#ifndef BOARDS_H
#define BOARDS_H

/** @brief Maximum number of boards supported in the system */
#define BOARDS_COUNT 10

#include "Canopen_Main.h"

/**
 * Generates a JSON representation of the current state of all boards. This is used by mongoose to update the UI.
 * @param json Pointer to buffer where the JSON string will be stored
 * @param max_len Maximum length of the JSON string buffer
 */
void get_boards_state_json(char* json, int max_len);

/**
 * Retrieves and updates the state information for all boards in the system.
 */
void update_boards_state();

/**
 * Converts a board name to its corresponding index in the boards array
 * @param name Name of the board to look up
 * @return Index of the board in the boards array, or -1 if not found
 */
int board_name_to_board_index(char* name);

/**
 * @struct board
 * @brief Structure containing board configuration and state information
 */
struct board {
    char name[10];             ///< Board name/identifier
    bool online;               ///< Current online status of the board
    int canopen_id;            ///< CANopen node ID of the board
    char fw_version[40];       ///< Current firmware version
    char last_update_date[32]; ///< Date of last firmware update
    char filename[100];        ///< Name of the firmware file
    char fw_uploader[32];      ///< Name of the firmware uploader
    CO_t* canopen_variant;     ///< Pointer to CANopen instance used for updating this board
};

#endif //BOARDS_H
