#include <stdio.h>
#include "pico/stdlib.h"
#include "FIRFilter.h"
#include "hardware/i2c.h"


static float FIR_IMPULSE_RESPONSE[FIR_FILTER_LENGTH] = {}; //array contain impulse response

void FIRFilter_Init(FIRFilter *fir) { /*this function is just to clear stuff*/

    /*Clear filter buffer, that circle buffer noted in header*/
    for (uint8_t n = 0;  n< FIR_FILTER_LENGTH; n++){

        fir->buf[n] = 0.0f; //Havent understand this yet :(
        
    }

    /*Reset buffer index*/
    fir->bufIndex = 0;
    
    /*clear filter output*/
    fir->out = 0.0f;

}

float FIRFilter_Update(FIRFilter *fir, float inp)   {/*the actual hard as fire stuff*/

    /*Store latest sample in buffer*/
    fir->buf[fir->bufIndex] = inp;

    /*increment buffer index and wrap around if necessary*/
    fir->bufIndex++;

    if (fir->bufIndex == FIR_FILTER_LENGTH) {
        
        fir->bufIndex = 0;

    }

    /*compute new output sample (via convolution )*/
    fir->out = 0.0f;

    uint8_t sumIndex = fir->bufIndex;

    for (uint8_t n = 0; n < FIR_FILTER_LENGTH; n++) {

        /*decrement index and wrap if necessary*/
        if (sumIndex > 0) {
            
            sumIndex--;

        } else{

            sumIndex = FIR_FILTER_LENGTH - 1;

        }

        /*multiply impulse response with shifted input sample and add to output*/
        fir->out += FIR_IMPULSE_RESPONSE[n] * fir->buf[sumIndex];

    }

    /*return filtered output*/
    return fir->out;
}

int main() {

    /*
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
    */


    
}