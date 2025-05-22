#ifndef INIT_H
#define INIT_H

#include "Canopen_Main.h"
#include "ODCrit.h"
#include "ODGen.h"

/**
 * @brief Initializes the CANopen communication system
 * 
 * This function performs the necessary initialization steps for CANopen
 * communication, including setting up the Object Dictionary and configuring
 * the CANopen stack.
 */
void canopen_init();

#endif //INIT_H
