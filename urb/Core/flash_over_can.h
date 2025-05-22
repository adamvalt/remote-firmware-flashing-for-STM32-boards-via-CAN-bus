#ifndef FLASHING_OVER_CAN_H
#define FLASHING_OVER_CAN_H

/**
 * Starts the firmware flashing process.
 * This function is called when the user initiates a firmware update through the UI.
 * It verifies preconditions and begins the update sequence.
 * 
 * The function checks:
 * - Current flashing state
 * - Firmware file size
 * - Target board availability
 * 
 * If all checks pass, it initiates the flashing process by:
 * 1. Resetting the target node
 * 2. Waiting for bootloader
 * 3. Starting the firmware transfer
 */
void flashing_start();

/**
 * Handles ongoing firmware transfer operations.
 * This function is called periodically to continue the firmware transfer process
 * and update the UI with progress information.
 * 
 * The function:
 * - Continues the SDO transfer if in progress
 * - Updates the progress percentage
 * - Updates the UI with current status
 * - Handles transfer completion or errors
 */
void periodic_can_flash();

#endif //FLASHING_OVER_CAN_H
