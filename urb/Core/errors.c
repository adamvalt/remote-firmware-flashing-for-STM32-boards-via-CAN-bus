#include "errors.h"
#include "globals.h"

static bool remove_scheduled = false;

void remove_errors() {
    s_flashing.state = NOT_FLASHING;
    strcpy(s_flashing.error, "");
    s_flashing.progress = 0;
    remove_scheduled = false;
    MG_DEBUG(("Error cleanup successful"));
}

void remove_errors_periodic_check() {
    if (strcmp(s_flashing.error, "") == 0 || remove_scheduled) {
        return;
    }

    remove_scheduled = true;
    mg_timer_add(&g_mgr, 5000, MG_TIMER_ONCE, (void (*)(void *)) remove_errors, NULL);
}