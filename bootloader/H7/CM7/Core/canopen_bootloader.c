#include "canopen_bootloader.h"

uint32_t offset = 0;
uint32_t timer = 0;
uint32_t current_address = BOOT_ADDR;
CO_t * CanOpen;
bool fw_uploading = false;
#define FREE_MEMORY_SIZE ((uint16_t) (SECTOR_SIZE * 7))

uint32_t mg_crc32(uint32_t crc, const char *buf, size_t len);
void loop_10Hz(void);
_Noreturn void jump_to_firmware();
void save_sdo_incoming_data_to_buffer_callback();
void save_buffer_data_to_flash();
bool is_firmware_in_flash();
bool is_firmware_crc_ok();
void loop_1000hz_bootloader();

uint8_t sector_buffer[SECTOR_SIZE + CO_CONFIG_SDO_SRV_BUFFER_SIZE] = {0};
int sector_buf_current_pos = 0;

#define TASKS_NUMBER 2
task_t tasks[TASKS_NUMBER] = {
        {loop_1000hz_bootloader, 1, 0},
        {loop_10Hz, 100, 0},
};

/**
 * Toggles heartbeat LED every 100ms.
 */
void loop_10Hz(void) {
    HAL_GPIO_TogglePin(HB_GPIO_Port, HB_Pin);
}

/**
 * Main loop that executes periodic tasks.
 */
void main_loop(void) {
    for (int loop = 0; loop < TASKS_NUMBER; loop++) {
        if (HAL_GetTick() >= tasks[loop].lastTaskTime + tasks[loop].period) {
            tasks[loop].lastTaskTime = HAL_GetTick();
            tasks[loop].task();
        }
    }
}

void canopen_init(){
    CO_config_t *CO_config;
    CO_CREATE_OD_CONFIG(ODBootloader, CO_config);
    CanOpen = CANOpenInit(CAN_INTERFACE_POINTER, CO_config, ODBootloader, CAN_NODE_ID);
    CO_SDOserver_initCallbackPre(CanOpen->SDOserver, NULL, save_sdo_incoming_data_to_buffer_callback);
}

/**
 * Handles CANopen communication and firmware upload process.
 * Executes at 1000Hz.
 */
void loop_1000hz_bootloader(){
    CANOpenProcess(CanOpen, HAL_GetTick());
    timer++;

    if (timer > 4000) {
        timer = 0;
        if (!fw_uploading && is_firmware_in_flash()) {
            jump_to_firmware();
        }
    }

    if (sector_buf_current_pos >= SECTOR_SIZE) {
        save_buffer_data_to_flash();
    }

    if (CanOpen->SDOserver->finished && CanOpen->SDOserver->index == 0x6003) {
        save_buffer_data_to_flash();
        if (is_firmware_in_flash() && is_firmware_crc_ok()) {
            jump_to_firmware();
        } else {
            HAL_FLASH_Lock();
            Flash_Erase(BOOT_ADDR, FREE_MEMORY_SIZE / 4);
            HAL_FLASH_Unlock();
        }
        fw_uploading = false;
        CanOpen->SDOserver->finished = false;
    }
}

/**
 * Jumps to the firmware application.
 * This function never returns.
 * https://community.st.com/t5/stm32-mcus/how-to-jump-to-system-bootloader-from-application-code-on-stm32/ta-p/49424
 */
_Noreturn void jump_to_firmware(void)
{
    /* Disable all interrupts */
    __disable_irq();

    /* Deinitialize all HALs */
    HAL_DeInit();

    /* Disable Systick timer */
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    /* Set the clock to the default state */
    HAL_RCC_DeInit();

    /* Reset all interrupts */
    for (int i = 0; i < sizeof(NVIC->ICER) / sizeof(NVIC->ICER[0]); i++)
    {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }

    /* Re-enable all interrupts */
    __enable_irq();

    // Set the Main Stack Pointer
    __set_MSP(BOOTVTAB->initial_SP);

    /* Ensure all outstanding memory accesses included buffered write are completed */
    __DSB();

    // Jump to firmware code
    BOOTVTAB->firmware();

    // This should not happen
    while (true) {
        __NOP();
    }
}

/**
 * Writes sector of buffered data to flash memory.
 */
void save_buffer_data_to_flash() {
    if (sector_buf_current_pos == 0) {
        return;
    }
    int flash_bytes_count = sector_buf_current_pos;
    if (sector_buf_current_pos > SECTOR_SIZE) {
        flash_bytes_count = SECTOR_SIZE;
    }
    Flash_Write_Data(current_address, (uint32_t *) sector_buffer, flash_bytes_count / 8);
    memset(sector_buffer, 0, flash_bytes_count);
    current_address += flash_bytes_count;

    // move remaining to the front of the buffer
    int remaining_size = sector_buf_current_pos - SECTOR_SIZE;
    if (remaining_size > 0) {
        for (int i = 0; i < remaining_size; i++) {
            sector_buffer[i] = sector_buffer[SECTOR_SIZE + i];
        }
        memset(&sector_buffer[SECTOR_SIZE], 0, remaining_size - 1);
        sector_buf_current_pos = remaining_size;
    } else {
        sector_buf_current_pos = 0;
    }
}

/**
 * Callback for handling incoming SDO data.
 * Stores received data in the sector buffer.
 */
void save_sdo_incoming_data_to_buffer_callback() {
    if (CanOpen->SDOserver->bufOffsetWr > 0 && CanOpen->SDOserver->index == 0x6003) {
        fw_uploading = true;

        for (int i = 0; i < CanOpen->SDOserver->bufOffsetWr; i++){
            sector_buffer[sector_buf_current_pos] = CanOpen->SDOserver->buf[i];
            sector_buf_current_pos++;
        }
        memset(CanOpen->SDOserver->buf, 0, CO_CONFIG_SDO_SRV_BUFFER_SIZE + 1);
        offset += CanOpen->SDOserver->bufOffsetWr;
        CanOpen->SDOserver->bufOffsetWr = 0;
    }
}

/**
 * Checks if firmware is present in flash memory.
 * @return true if firmware is present, false otherwise
 */
bool is_firmware_in_flash() {
    return *((uint32_t *)BOOT_ADDR) != 0xFFFFFFFF;
}

/**
 * Verifies firmware CRC against stored value.
 * @return true if CRC matches, false otherwise
 */
bool is_firmware_crc_ok() {
    return ODBootloader_RAM.x6000_firmware_crc32 != 0 && ODBootloader_RAM.x6000_firmware_crc32 == mg_crc32(0, (const char *) BOOT_ADDR, ODBootloader_RAM.x6001_firmware_size);
}

/**
 * Calculates CRC32 of data using the same algorithm as URB.
 * @param crc Initial CRC value
 * @param buf Data buffer
 * @param len Length of data
 * @return Calculated CRC32 value
 */
uint32_t mg_crc32(uint32_t crc, const char *buf, size_t len) {
    static const uint32_t crclut[16] = {
            // table for polynomial 0xEDB88320 (reflected)
            0x00000000, 0x1DB71064, 0x3B6E20C8, 0x26D930AC, 0x76DC4190, 0x6B6B51F4,
            0x4DB26158, 0x5005713C, 0xEDB88320, 0xF00F9344, 0xD6D6A3E8, 0xCB61B38C,
            0x9B64C2B0, 0x86D3D2D4, 0xA00AE278, 0xBDBDF21C};
    crc = ~crc;
    while (len--) {
        uint8_t b = *(uint8_t *) buf++;
        crc = crclut[(crc ^ b) & 0x0F] ^ (crc >> 4);
        crc = crclut[(crc ^ (b >> 4)) & 0x0F] ^ (crc >> 4);
    }
    return ~crc;
}

/**
 * Main bootloader entry point.
 * This function never returns.
 */
_Noreturn void canopen_bootloader_run() {
    canopen_init();
    while (true) {
        main_loop();
    }
}