#ifndef HEXAPOD_H
#define HEXAPOD_H

#include <stdint.h>

typedef struct
{
    int32_t leg_top;
    int32_t leg_mid;
    int32_t leg_bot;
} hexapod_leg_t;

int32_t hexapod_get_next_seq_value(uint32_t leg_number, hexapod_leg_t *leg);

void hexapod_init(void);
void hexapod_move_forward(uint8_t speed);
void hexapod_move_right(uint8_t speed);
void hexapod_move_diagonal(uint8_t speed);
void hexapod_turn_clockwise(uint8_t speed);
void hexapod_stop(uint8_t speed);

#endif //HEXAPOD_H
