#ifndef BME280_H
#define BME280_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define BME280_ADDR 0x76  // 0x77 if SDO is high

typedef struct {
    float temp;
    float press;
    float hum;
} bme280_data;

void bme280_init(i2c_inst_t *i2c);
bme280_data bme280_read(i2c_inst_t *i2c);

#endif
