#pragma once

#include <stdint.h>

void AS5048A_init();
void AS5048A_clear_error();
uint16_t AS5048A_get_position();
void AS5048A_set_zero_position();
void AS5048A_get_diagnostics(uint8_t *comp_high, uint8_t *comp_low, uint8_t *COF, uint8_t *OCF, uint8_t * AGC);
uint8_t AS5048A_get_gain();
