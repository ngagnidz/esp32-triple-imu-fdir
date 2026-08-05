#ifndef TX_H
#define TX_H

#include <stdint.h>
#include "mpu9250.h"

#define CAN_ID 0x100
#define TX_GPIO 21
#define RX_GPIO 23

void can_link_init(void);
void can_send_state(uint8_t state);
void can_send_data(char str, float data);
#endif
