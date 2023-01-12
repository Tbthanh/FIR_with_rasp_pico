#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "FIRFilter.h"

/* *************************************** FIRFilter function declaration *************************************** */

static float FIR_IMPULSE_RESPONSE[FIR_FILTER_LENGTH] = {
    -0.0007349825177827979,
  -0.0009316959904301274,
  -0.0012094214438907258,
  -0.001200839053747552,
  -0.0007110893180593774,
  0.0003946397787565515,
  0.002119704499772935,
  0.00427686645100855,
  0.006466290650520017,
  0.008112323671845667,
  0.008560266952326702,
  0.0072271700476080995,
  0.003785481627868066,
  -0.0016725748868835753,
  -0.008519782223016399,
  -0.015584942718847335,
  -0.02126213169327571,
  -0.023743711092941877,
  -0.021342942828122345,
  -0.012851722042743255,
  0.0021413549174536966,
  0.02304334720887553,
  0.04821898995484656,
  0.0751468775398364,
  0.10075533630429342,
  0.12188757064914492,
  0.13580965268676104,
  0.1406688226559694,
  0.13580965268676104,
  0.12188757064914492,
  0.10075533630429342,
  0.0751468775398364,
  0.04821898995484656,
  0.02304334720887553,
  0.0021413549174536966,
  -0.012851722042743255,
  -0.021342942828122345,
  -0.023743711092941877,
  -0.02126213169327571,
  -0.015584942718847335,
  -0.008519782223016399,
  -0.0016725748868835753,
  0.003785481627868066,
  0.0072271700476080995,
  0.008560266952326702,
  0.008112323671845667,
  0.006466290650520017,
  0.00427686645100855,
  0.002119704499772935,
  0.0003946397787565515,
  -0.0007110893180593774,
  -0.001200839053747552,
  -0.0012094214438907258,
  -0.0009316959904301274,
  -0.0007349825177827979    
}; //array contain impulse response

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

/* *************************************** I2C function declaration, definition and const *************************************** */

// I2C address
static const uint8_t PCF8591_ADDR = 0x48;

//d
#define I2C_PORT i2c0

// Registers - dont actually understand this
static const uint8_t REG_DEVID = 0x00;
static const uint8_t REG_POWER_CTL = 0x2D;
static const uint8_t REG_DATAX0 = 0x32;

// Other constants - also this too
static const uint8_t DEVID = 0xE5;

/* I2C Function Declarations*/
int reg_write(i2c_inst_t *i2c, 
                const uint addr, 
                const uint8_t reg, 
                uint8_t *buf,
                const uint8_t nbytes);

int reg_read(   i2c_inst_t *i2c,
                const uint addr,
                const uint8_t reg,
                uint8_t *buf,
                const uint8_t nbytes);

/* I2C Function Definitions */
// Write 1 byte to the specified register
int reg_write(  i2c_inst_t *i2c, 
                const uint addr, 
                const uint8_t reg, 
                uint8_t *buf,
                const uint8_t nbytes) {

    int num_bytes_read = 0;
    uint8_t msg[nbytes + 1];

    // Check to make sure caller is sending 1 or more bytes
    if (nbytes < 1) {
        return 0;
    }

    // Append register address to front of data packet
    msg[0] = reg;
    for (int i = 0; i < nbytes; i++) {
        msg[i + 1] = buf[i];
    }

    // Write data to register(s) over I2C
    i2c_write_blocking(i2c, addr, msg, (nbytes + 1), false);

    return num_bytes_read;
}

// Read byte(s) from specified register. If nbytes > 1, read from consecutive
// registers.
int reg_read(  i2c_inst_t *i2c,
                const uint addr,
                const uint8_t reg,
                uint8_t *buf,
                const uint8_t nbytes) {

    int num_bytes_read = 0;

    // Check to make sure caller is asking for 1 or more bytes
    if (nbytes < 1) {
        return 0;
    }

    // Read data from register(s) over I2C
    i2c_write_blocking(i2c, addr, &reg, 1, true);
    num_bytes_read = i2c_read_blocking(i2c, addr, buf, nbytes, false);

    return num_bytes_read;
}

/*create a firfilter struct*/
FIRFilter tempout;

int main() {

    //to initialize the input output, choosen serial port
    stdio_init_all();

    //configure adc on board
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4); //actualy the 5th chanel what the phuc 

    /*initialise firfilter*/
    FIRFilter_Init(&tempout); /*resting the circuler buffer*/

    /*I2C parts*/
    // I2C Pins
    const uint sda_pin = 16;
    const uint scl_pin = 17;

    // Ports
    i2c_inst_t *i2c = i2c0;

    // Buffer to store raw I2C reads
    uint8_t data[6];

    //Initialize I2C port at 400 kHz
    i2c_init(i2c, 1000 * 1000);

    // Initialize I2C pins
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);

    // Read device ID to make sure that we can communicate with the ADXL343
    reg_read(i2c, PCF8591_ADDR, REG_DEVID, data, 1);
    if (data[0] != DEVID) {
        printf("ERROR: Could not communicate with PCF8591\r\n");
        while (true);
    }

    // Read Power Control register
    reg_read(i2c, PCF8591_ADDR, REG_POWER_CTL, data, 1);
    printf("0x%02X\r\n", data[0]);

    // Tell ADXL343 to start taking measurements by setting Measure bit to high
    data[0] |= (1 << 3);
    reg_write(i2c, PCF8591_ADDR, REG_POWER_CTL, &data[0], 1);

    // Test: read Power Control register back to make sure Measure bit was set
    reg_read(i2c, PCF8591_ADDR, REG_POWER_CTL, data, 1);
    printf("0x%02X\r\n", data[0]);

    // Wait before taking measurements
    sleep_ms(2000);

    //infinite loop
    while (1) {

        uint16_t raw = adc_read();
        const float conversion = 3.3f / (1 << 12);
        float voltage = raw * conversion; //basicly doing some filtered stuff
        float temperature = 27 - (voltage - 0.706) / 0.001721; //adc to temp

        /*Using the filter, in the guide, he's using an stm32 custom board so there sure are a lot of trouble here, 
        seems that the driver that he write for the accelometer is XD*/

        FIRFilter_Update(&tempout, temperature); //using lowpass filter something something that I'll figure out
        
        /*push raw and filtered data through serial*/
        printf("%.4f,%.4f\r\n", temperature, tempout.out);

        /*push data through dac*/
        i2c_write_blocking();
        
        sleep_ms(1000);
    }
    

}