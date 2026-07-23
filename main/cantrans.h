#ifndef CAN_H
#define CAN_H

#include "voting.h"

void config();
void transmit_status(int status_code);
void transmit_data(burst_read_data_t reading[3]);
#endif