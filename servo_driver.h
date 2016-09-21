
#ifndef PWM_DRIVER_H
#define PWM_DRIVER_H

#include "hexapod.h"

void hexapod_servo_pwm_init(hexapod_leg_t leg_pins[6]);
void hexapod_servo_pwm_stop(void);

#endif //PWM_DRIVER_H
