#ifndef REMOTE_REBOOT_H
#define REMOTE_REBOOT_H

#include <stdbool.h>

/** @brief Minimum time in milliseconds reserved for a node to be rebooted after a reboot command was sent */
#define NODE_REBOOT_MINIMUM_TIME_MS 1500

/**
 * Initiates a remote reboot of a board over CAN.
 * @param board_index Index of the board to reboot
 * @return true if reboot command was sent successfully, false otherwise
 */
bool remote_reboot(int board_index);

#endif //REMOTE_REBOOT_H
