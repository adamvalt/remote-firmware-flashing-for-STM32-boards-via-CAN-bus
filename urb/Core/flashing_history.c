#include "flashing_history.h"
#include "stm32h7xx_hal.h"
#include "mongoose.h"
#include <string.h>
#include <stdio.h>

typedef struct flash_history_entry {
    int magic_number;
    char name[10];
    char version[40];
    char timestamp[32];
    char filename[100];
    char author[32];
    char status[16];
} flash_history_entry_t;

#define FLASH_HISTORY_FLASH_START_ADDR 0x08040000  // Start of flash memory
#define FLASH_HISTORY_ENTRY_SIZE sizeof(flash_history_entry_t)       // Size of each history entry
#define FLASH_HISTORY_MAX_ENTRIES 48             // Maximum number of history entries to store

#define FLASHING_HISTORY_MAGIC_NUMBER 0x0102AB45



void read_flashing_history_entry(flash_history_entry_t* entry, int index) {
    uint32_t addr = FLASH_HISTORY_FLASH_START_ADDR + (index * FLASH_HISTORY_ENTRY_SIZE);
    memcpy(entry, (void*)addr, FLASH_HISTORY_ENTRY_SIZE);
}

void read_flashing_history(flash_history_entry_t* entries, int* count) {
    *count = 0;
    for (int i = 0; i < FLASH_HISTORY_MAX_ENTRIES; i++) {
        read_flashing_history_entry(&entries[*count], i);
        if (entries[*count].magic_number == FLASHING_HISTORY_MAGIC_NUMBER) {
            (*count)++;
        }
    }
}

void get_flashing_history_json(char* json, int max_len) {
    flash_history_entry_t entries[FLASH_HISTORY_MAX_ENTRIES] = {0};
    int valid_entries = 0;
    
    read_flashing_history(entries, &valid_entries);

    int pos = 0;
    pos += snprintf(json + pos, max_len - pos, "[");
    
    for (int i = 0; i < valid_entries; i++) {
        if (i > 0) {
            pos += snprintf(json + pos, max_len - pos, ",");
        }
        
        pos += snprintf(json + pos, max_len - pos,
            "{\"shortname\":\"%s\",\"version\":\"%s\",\"timestamp\":\"%s\",\"filename\":\"%s\",\"author\":\"%s\",\"status\":\"%s\"}",
            entries[i].name,
            entries[i].version,
            entries[i].timestamp,
            entries[i].filename,
            entries[i].author,
            entries[i].status);
            
        if (pos >= max_len - 1) {
            break; // uncomplete json will be sent
        }
    }
    
    snprintf(json + pos, max_len - pos, "]\0");
}

void save_flashing_history_entry(char* name, char* version, char* timestamp, char* filename, char* author, char* status) {
    flash_history_entry_t entries[FLASH_HISTORY_MAX_ENTRIES + 1] = {0};
    int valid_entries = 0;

    flash_history_entry_t* new_entry = &entries[0];
    new_entry->magic_number = FLASHING_HISTORY_MAGIC_NUMBER;
    strcpy(new_entry->name, name);
    strcpy(new_entry->version, version);
    strcpy(new_entry->timestamp, timestamp);
    strcpy(new_entry->filename, filename);
    strcpy(new_entry->author, author);
    strcpy(new_entry->status, status);
    
    read_flashing_history(&entries[1], &valid_entries);
    valid_entries++;

    mg_stm32h7_save_data_write((char *) FLASH_HISTORY_FLASH_START_ADDR, entries, FLASH_HISTORY_ENTRY_SIZE * FLASH_HISTORY_MAX_ENTRIES);
}