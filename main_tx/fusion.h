#ifndef FUSION_H
#define FUSION_H

#include "voting.h"

bool average_good(const burst_read_data_t readings[3], voting_result result, data_to_send *out);

#endif