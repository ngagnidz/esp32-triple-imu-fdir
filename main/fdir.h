typedef enum
{
    NOMINAL,
    DEGRADED,
    FAILED,
} system_condition;

const char *get_status(system_condition status);
system_condition update_state(voting_result result);