#ifndef CANOPEN_BOOTLOADER_H
#define CANOPEN_BOOTLOADER_H

#include <stdbool.h>
#include <stdio.h>
#include "Canopen_Main.h"
#include "ODBootloader.h"

#if defined(STM32G473xx)
#include "fdcan.h"
#include "stm32g4xx_hal.h"
#include "flash_g4.h"
#define BOOT_ADDR 0x0800A000

#elif defined(STM32H745xx)
#include "fdcan.h"
#include "stm32h7xx_hal.h"
#include "flash_h7.h"
#define BOOT_ADDR 0x08020000

#elif defined(STM32F429xx)
#error "This platform is not supported"
#else
#error "No STM32 target platform has been defined."
#endif

#define CAN_NODE_ID_VCU 0x01
#define CAN_NODE_ID_PDU 0x02
#define CAN_NODE_ID_ASB 0x03
#define CAN_NODE_ID_BMS 0x04
#define CAN_NODE_ID_SCM 0x05
#define CAN_NODE_ID_PDB 0x08
#define CAN_NODE_ID_STEERING 0x0B
#define CAN_NODE_ID_COOLING 0x0D
#define CAN_NODE_ID_TIRETEMP 0x0E

#define CAN_NODE_ID CAN_NODE_ID_ASB
#define CAN_INTERFACE_POINTER &hfdcan1

#ifndef CAN_NODE_ID
#error "Define CAN_NODE_ID for this bootloader"
#endif

/** @brief Structure containing firmware vector table entries */
struct boot_vectable_ {
    uint32_t initial_SP;       ///< Initial stack pointer
    void (*firmware)(void);    ///< Pointer to firmware entry point
};

/*
 * Mapping BOOT_ADDR to boot_vectable_ struct automatically provides us with
 * first 4 bytes of the sector as a Stack Pointer variable and pointer to the start of the firmware code
 */
#define BOOTVTAB	((struct boot_vectable_ *)BOOT_ADDR)

/** @brief Structure for periodic task management */
typedef struct {
    void (*task)(void);        ///< Task function pointer
    uint32_t period;           ///< Task period in milliseconds
    uint32_t lastTaskTime;     ///< Last execution timestamp
} task_t;

/**
 * Main loop of the bootloader.
 * Handles periodic tasks and system state management.
 */
void main_loop(void);

/**
 * Initializes the CANopen communication stack.
 */
void canopen_init();

/**
 * Runs the bootloader main loop.
 * This function never returns.
 */
_Noreturn void canopen_bootloader_run();

#endif // CANOPEN_BOOTLOADER_H