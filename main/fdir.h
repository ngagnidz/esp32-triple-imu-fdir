#ifndef FDIR_H
#define FDIR_H

#include "voting.h"

typedef enum
{
    NOMINAL,
    DEGRADED,
    FAILED,
} system_condition;

char *get_state_name(system_condition status);
int get_state_number(system_condition status);
system_condition update_state(voting_result result);

#endif