#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "FIRFilter.h"

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

    //to initialize the input output
    stdio_init_all();

    //configure adc on board
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4); //actualy the 5th chanel what the phuc 

    //infinite loop
    while (1) {

        uint16_t raw = adc_read();
        const float conversion = 3.3f / (1 << 12);
        float voltage = raw * conversion; //basicly doing some filtered stuff
        float temperature = 27 - (voltage - 0.706) / 0.001721; //adc to temp
        output = 
        printf("Temperature:", temperature, "output =");
        sleep_ms(1000);
    }
    

}