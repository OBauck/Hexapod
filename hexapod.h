#ifndef HEXAPOD_H
#define HEXAPOD_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    int32_t leg_top;
    int32_t leg_mid;
    int32_t leg_bot;
} hexapod_leg_t;

int32_t hexapod_get_next_seq_value(uint32_t leg_number, hexapod_leg_t *leg);

void hexapod_init(void);
void hexapod_move_forward(bool direction, uint8_t speed);
void hexapod_move_sideways(bool direction, uint8_t speed);
void hexapod_move_diagonal(bool direction, uint8_t speed);
void hexapod_turn(bool direction, uint8_t speed);
void hexapod_stop(uint8_t speed);

void hexapod_test_sequence(uint8_t speed);
void hexapod_shutdown(void);

#endif //HEXAPOD_H
