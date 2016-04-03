#ifndef HEXAPOD_H
#define HEXAPOD_H

#include <stdint.h>

typedef struct
{
    uint32_t leg_top;
    uint32_t leg_mid;
    uint32_t leg_bot;
} hexapod_leg_t;

int32_t hexapod_get_next_seq_value(uint32_t leg_number, hexapod_leg_t *leg);

void hexapod_init(void);

#endif //HEXAPOD_H
