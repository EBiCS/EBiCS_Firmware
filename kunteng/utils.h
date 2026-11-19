/*
 * EGG OpenSource EBike firmware
 *
 * Copyright (C) Casainho, 2015, 2106, 2017.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _UTILS_H
#define _UTILS_H

#include "main.h"

int32_t map (int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
uint8_t ui8_max (uint8_t value_a, uint8_t value_b);
uint8_t ui8_min (uint8_t value_a, uint8_t value_b);

#endif /* _UTILS_H */
