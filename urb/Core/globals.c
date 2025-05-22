#include "globals.h"

CO_t *CanOpenCrit;
CO_t *CanOpenGen;

struct board s_boards[BOARDS_COUNT] = {
        {"URB", true, SELF_CAN_NODE_ID, "42.0.0", "2025-03-01 20:20", "URB.bin", "admin", NULL},
        {"VCU", true, 0x01, "1.0.0", "2025-03-01 20:20", "VCU.bin", "admin", NULL},
        {"PDU", true, 0x02, "1.0.0", "2025-03-02 20:20", "PDU.bin", "admin", NULL},
        {"ASB", true, 0x03, "1.0.0", "2025-03-03 20:20", "ASB.bin", "admin", NULL},
        {"BMS", true, 0x04, "1.0.0", "2025-03-03 20:20", "BMS.bin", "admin", NULL},
        {"SCM", true, 0x05, "1.0.0", "2025-03-03 20:20", "SCM.bin", "admin", NULL},
        {"PDB", true, 0x08, "1.0.0", "2025-03-04 20:20", "PDB.bin", "admin", NULL},
        {"STEERING", true, 0x0B, "1.0.0", "2025-03-04 20:20", "STEERING.bin", "admin", NULL},
        {"COOLING", true, 0x0D, "1.0.0", "2025-03-04 20:20", "COOLING.bin", "admin", NULL},
        {"TIRETEMP", true, 0x0E, "1.0.0", "2025-03-04 20:20", "TIRETEMP.bin", "admin", NULL},
};

struct flashing s_flashing = {"VCU", "1.1.0", "No file uploaded", "", false, 0, 0, NOT_FLASHING};
struct security s_security = {"admin", "user"};
struct state s_mongoose_state = {10, "1.0.0", true};