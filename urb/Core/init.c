/**
 * @file init.c
 * @brief System initialization implementation
 * 
 * This file implements the system initialization functionality, particularly
 * focusing on CANopen initialization and configuration. It sets up the CANopen
 * stack with both critical and general communication channels.
 */

#include "init.h"
#include "fdcan.h"
#include "globals.h"

void canopen_init(){
    CO_config_t *configCrit;
    CO_config_t *configGen;
    CO_CREATE_OD_CONFIG(ODCrit, configCrit);
    CO_CREATE_OD_CONFIG(ODGen, configGen);
    
    // Configure critical CANopen instance
    ODCrit_RAM.x1280_SDOClientParameter.COB_IDClientToServer = 0x600 + SELF_CAN_NODE_ID;
    ODCrit_RAM.x1280_SDOClientParameter.COB_IDServerToClient = 0x580 + SELF_CAN_NODE_ID;
    ODCrit_RAM.x1280_SDOClientParameter.node_IDOfTheSDOServer = SELF_CAN_NODE_ID;
    CanOpenCrit = CANOpenInit(&hfdcan1, configCrit, ODCrit, SELF_CAN_NODE_ID);
    
    // Configure general CANopen instance
    ODGen_RAM.x1280_SDOClientParameter.COB_IDClientToServer = 0x600 + SELF_CAN_NODE_ID;
    ODGen_RAM.x1280_SDOClientParameter.COB_IDServerToClient = 0x580 + SELF_CAN_NODE_ID;
    ODGen_RAM.x1280_SDOClientParameter.node_IDOfTheSDOServer = SELF_CAN_NODE_ID;
    CanOpenGen = CANOpenInit(&hfdcan2, configGen, ODGen, SELF_CAN_NODE_ID);
    
    s_boards[0].canopen_variant = CanOpenCrit;
    s_boards[1].canopen_variant = CanOpenCrit;
    s_boards[2].canopen_variant = CanOpenCrit;
    s_boards[3].canopen_variant = CanOpenCrit;
    s_boards[4].canopen_variant = CanOpenCrit;
    s_boards[5].canopen_variant = CanOpenCrit;
    s_boards[6].canopen_variant = CanOpenGen;
    s_boards[7].canopen_variant = CanOpenGen;
    s_boards[8].canopen_variant = CanOpenGen;
    s_boards[9].canopen_variant = CanOpenGen;
}