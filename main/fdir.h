typedef enum
{
    NOMINAL,
    DEGRADED,
    FAILED,
} system_condition;

char *state_name(system_condition status);
system_condition update_state(voting_result result);