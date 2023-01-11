#include <stdio.h>
#include "pico/stdlib.h"
#include "FIRFilter.h"

static float FIR_IMPULSE_RESPONSE=[FIR_FILTER_LENGTH] = {}; //array contain impulse response

void FIRFilter_Init(FIRFilter *fir) {

}
int main() {

    const uint led_pin = 25;

    // Initialize LED pin
    gpio_init(led_pin);
    gpio_set_dir(led_pin, GPIO_OUT);

    // Initialize chosen serial port
    stdio_init_all();

    // Loop forever
    while (true) {

        // Blink LED
        printf("Blinking!\r\n");
        gpio_put(led_pin, true);
        sleep_ms(1000);
        gpio_put(led_pin, false);
        sleep_ms(1000);
    }
}