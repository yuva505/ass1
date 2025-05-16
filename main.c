#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "bme280.h"

int main() {
    stdio_init_all();
    i2c_init(i2c_default, 400 * 1000); // 400 kHz
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    bme280_init(i2c_default);

    while (1) {
        bme280_data data = bme280_read(i2c_default);
        
        printf("Temperature: %.2f °C\n", data.temp);
        printf("Pressure: %.2f hPa\n", data.press / 100.0f);
        printf("Humidity: %.2f %%\n", data.hum);
        printf("\n");

        sleep_ms(2000);
    }

    return 0;
}
