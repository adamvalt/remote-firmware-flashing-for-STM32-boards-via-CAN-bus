#ifndef FLASHING_HISTORY_H
#define FLASHING_HISTORY_H

/**
 * Generates a JSON representation of the firmware flashing history.
 * @param json Pointer to buffer where the JSON string will be stored
 * @param max_len Maximum length of the JSON string buffer
 */
void get_flashing_history_json(char* json, int max_len);

/**
 * Records a new entry in the firmware flashing history.
 * @param name Board name that was flashed
 * @param version Firmware version that was uploaded
 * @param timestamp When the flashing occurred
 * @param filename Name of the firmware file
 * @param author Who performed the flashing
 * @param status Success/failure status of the flashing
 */
void save_flashing_history_entry(char* name, char* version, char* timestamp, char* filename, char* author, char* status);

#endif //FLASHING_HISTORY_H
