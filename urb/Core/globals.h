/**
 * @file globals.h
 * @brief Global declarations and definitions for the URB system
 * 
 * This header file contains global declarations and definitions used throughout
 * the URB system, including CAN node configuration, timing parameters, and
 * global state structures.
 */

#ifndef GLOBALS_H
#define GLOBALS_H

#include "CANopen.h"
#include "mongoose_glue.h"
#include "boards.h"

/** @brief CAN node ID for this device */
#define SELF_CAN_NODE_ID 0x10

/** @brief Time interval between repeated SDO operations in milliseconds */
#define REPEATED_SDO_DELTA_TIME_MS 5

/** @brief Array of board configurations */
extern struct board s_boards[BOARDS_COUNT];

/** @brief Global flashing state structure */
extern struct flashing s_flashing;

/** @brief Global security state structure */
extern struct security s_security;

/** @brief Global mongoose state structure */
extern struct state s_mongoose_state;

/** @brief CANopen instance for CANCrit */
extern CO_t *CanOpenCrit;

/** @brief CANopen instance for CANGeneral */
extern CO_t *CanOpenGen;

#endif //GLOBALS_H
