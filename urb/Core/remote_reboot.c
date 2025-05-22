#include "remote_reboot.h"
#include "mongoose_glue.h"
#include "Canopen_Main.h"
#include "boards.h"
#include "globals.h"

bool remote_reboot(int board_index) {
    if (board_index < 0 || board_index >= BOARDS_COUNT) {
        MG_ERROR(("Invalid board_index: %d", board_index));
        return false;
    }

    char* board_name = s_boards[board_index].name;

    if (!s_boards[board_index].online) {
        MG_ERROR(("Board %s is not online", board_name));
        return false;
    }

    uint8_t target_node_id = s_boards[board_index].canopen_id;
    if (target_node_id == 0) {
        MG_ERROR(("Invalid CANopen ID for board %s", board_name));
        return false;
    }

    // Send reset command
    CO_ReturnError_t ret = CO_NMT_sendCommand(s_boards[board_index].canopen_variant->NMT,
                                            CO_NMT_RESET_NODE,
                                            target_node_id);
    if (ret != CO_ERROR_NO) {
        MG_ERROR(("Failed to reset node 0x%X (board %s): error %d",
                 target_node_id, board_name, ret));
        return false;
    }

    MG_INFO(("Successfully initiated reset for board %s (node 0x%X)",
            board_name, target_node_id));
    return true;
}