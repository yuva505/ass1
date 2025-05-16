#include "bme280.h"
#include <math.h>

// BME280 Registers
#define BME280_REG_TEMP     0xFA
#define BME280_REG_PRESS    0xF7
#define BME280_REG_HUM      0xFD
#define BME280_REG_CONFIG   0xF5
#define BME280_REG_CTRL_HUM 0xF2
#define BME280_REG_CTRL_MEAS 0xF4

// Compensation parameters
static int32_t t_fine;
static uint16_t dig_T1;
static int16_t dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
static uint8_t dig_H1, dig_H3;
static int16_t dig_H2, dig_H4, dig_H5, dig_H6;

static void read_compensation_params(i2c_inst_t *i2c) {
    uint8_t buf[26];

    // Read temperature/pressure compensation (0x88-0xA1)
    i2c_write_blocking(i2c, BME280_ADDR, (uint8_t[]){0x88}, 1, true);
    i2c_read_blocking(i2c, BME280_ADDR, buf, 26, false);

    dig_T1 = (buf[1] << 8) | buf[0];
    dig_T2 = (buf[3] << 8) | buf[2];
    dig_T3 = (buf[5] << 8) | buf[4];
    dig_P1 = (buf[7] << 8) | buf[6];
    dig_P2 = (buf[9] << 8) | buf[8];
    dig_P3 = (buf[11] << 8) | buf[10];
    dig_P4 = (buf[13] << 8) | buf[12];
    dig_P5 = (buf[15] << 8) | buf[14];
    dig_P6 = (buf[17] << 8) | buf[16];
    dig_P7 = (buf[19] << 8) | buf[18];
    dig_P8 = (buf[21] << 8) | buf[20];
    dig_P9 = (buf[23] << 8) | buf[22];
    dig_H1 = buf[25];

    // Read humidity compensation (0xE1-0xE7)
    i2c_write_blocking(i2c, BME280_ADDR, (uint8_t[]){0xE1}, 1, true);
    i2c_read_blocking(i2c, BME280_ADDR, buf, 7, false);

    dig_H2 = (buf[1] << 8) | buf[0];
    dig_H3 = buf[2];
    dig_H4 = (buf[3] << 4) | (buf[4] & 0x0F);
    dig_H5 = (buf[5] << 4) | (buf[4] >> 4);
    dig_H6 = (int8_t)buf[6];
}

void bme280_init(i2c_inst_t *i2c) {
    // Soft reset
    uint8_t reset_cmd = 0xB6;
    i2c_write_blocking(i2c, BME280_ADDR, (uint8_t[]){0xE0, reset_cmd}, 2, false);
    sleep_ms(100);

    // Read compensation data
    read_compensation_params(i2c);

    // Configure humidity, pressure, and temperature oversampling
    i2c_write_blocking(i2c, BME280_ADDR, (uint8_t[]){BME280_REG_CTRL_HUM, 0x01}, 2, false);
    i2c_write_blocking(i2c, BME280_ADDR, (uint8_t[]){BME280_REG_CTRL_MEAS, 0x27}, 2, false);
    i2c_write_blocking(i2c, BME280_ADDR, (uint8_t[]){BME280_REG_CONFIG, 0x00}, 2, false);
}

bme280_data bme280_read(i2c_inst_t *i2c) {
    uint8_t data[8];
    bme280_data result;

    // Read all data (pressure + temp + hum)
    i2c_write_blocking(i2c, BME280_ADDR, (uint8_t[]){BME280_REG_PRESS}, 1, true);
    i2c_read_blocking(i2c, BME280_ADDR, data, 8, false);

    // Pressure (20-bit)
    int32_t adc_P = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | (data[2] >> 4);

    // Temperature (20-bit)
    int32_t adc_T = ((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | (data[5] >> 4);

    // Humidity (16-bit)
    int32_t adc_H = ((uint32_t)data[6] << 8) | data[7];

    // Calculate temperature (compensation formulas from datasheet)
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    result.temp = ((t_fine * 5 + 128) >> 8) / 100.0f;

    // Calculate pressure (compensation)
    int64_t var3 = ((int64_t)t_fine) - 128000;
    int64_t var4 = var3 * var3 * (int64_t)dig_P6;
    var4 = var4 + ((var3 * (int64_t)dig_P5) << 17);
    var4 = var4 + (((int64_t)dig_P4) << 35);
    var3 = ((var3 * var3 * (int64_t)dig_P3) >> 8) + ((var3 * (int64_t)dig_P2) << 12);
    var3 = (((((int64_t)1) << 47) + var3)) * ((int64_t)dig_P1) >> 33;
    if (var3 == 0) result.press = 0;
    else {
        int64_t p = 1048576 - adc_P;
        p = (((p << 31) - var4) * 3125) / var3;
        var3 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
        var4 = (((int64_t)dig_P8) * p) >> 19;
        p = ((p + var3 + var4) >> 8) + (((int64_t)dig_P7) << 4);
        result.press = (float)p / 256.0f;
    }

    // Calculate humidity (compensation)
    int32_t v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4);
    v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
    v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
    result.hum = (float)(v_x1_u32r >> 12) / 1024.0f;

    return result;
}
