#ifndef HOME_H
#define HOME_H

#include "stepper.h"
#include "as5600.h"
#include "config.h"

void home(stepper_t *stepper, as5600_t *as5600);

#endif // HOME_H