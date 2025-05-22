/**
 * @file boards.c
 * @brief Implementation of board management functionality
 * 
 * This file implements the board management functions defined in boards.h.
 * It provides functionality for updating board states, generating JSON
 * representations of board states, and looking up board indices.
 */

#include <stdbool.h>
#include "boards.h"
#include "mongoose_glue.h"
#include "globals.h"
#include <string.h>
#include <stdio.h>

void update_boards_state() {
    // placeholder for future updates, this has to be first implemented at the side of the ECUs
    glue_update_state();
}

void get_boards_state_json(char* json, int max_len) {    
    int pos = 0;
    pos += snprintf(json + pos, max_len - pos, "[");

    for (int i = 0; i < BOARDS_COUNT; i++) {
        if (i > 0) {
            pos += snprintf(json + pos, max_len - pos, ",");
        }
        pos += snprintf(json + pos, max_len - pos, 
            "{\"shortname\":\"%s\",\"online\":%s,\"canopen_id\":\"0x%X\",\"fw_version\":\"%s\",\"last_update_date\":\"%s\",\"filename\":\"%s\",\"fw_uploader\":\"%s\"}",
            s_boards[i].name,
            s_boards[i].online ? "true" : "false",
            s_boards[i].canopen_id,
            s_boards[i].fw_version,
            s_boards[i].last_update_date,
            s_boards[i].filename,
            s_boards[i].fw_uploader);
        
        if (pos >= max_len - 1) {
            break; // uncomplete json will be sent
        }
    }
    snprintf(json + pos, max_len - pos, "]\0");
}

/**
 * @brief Converts a board name to its corresponding index in the boards array
 * 
 * Searches through the boards array to find a board with the matching name.
 * 
 * @param name Name of the board to look up
 * @return int Index of the board in the boards array, or -1 if not found
 */
int board_name_to_board_index(char* name){
    for (int i = 0; i < BOARDS_COUNT; i++) {
        if (strcmp(s_boards[i].name, name) == 0) {
            return i;
        }
    }
    return -1;
}