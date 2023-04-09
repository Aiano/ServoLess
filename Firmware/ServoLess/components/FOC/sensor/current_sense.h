#pragma once

#define CS_OFFSET_VOLTAGE 1650 // mV
#define CS_SAMPLE_RESISTOR 0.01 // Ohm
#define CS_INA_GAIN 50.0 // V/V

extern int cs_adc_raw[2][10];
extern float cs_current[3];

void cs_init();
void cs_read();