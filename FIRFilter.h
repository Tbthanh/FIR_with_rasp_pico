//All the comment is taken from this video by Phil's lab:
//https://youtu.be/uNNNj9AZisM
#ifndef FIR_FILTER_H
#define FIR_FILTER_H

#include <stdint.h> // to use uint8_t

#define FIR_FILTER_LENGTH 16 //have effect to filter coeficient (cov?)

typedef struct
{
    float   buf[FIR_FILTER_LENGTH]; //using circular buffers: 
                                    //https://www.allaboutcircuits.com/technical-articles/circular-buffer-a-critical-element-of-digital-signal-processors/
    uint8_t bufIndex;

    float out; //current filter output
} FIRFilter;

void FIRFilter_Init(FIRFilter *fir); //initialization funct, reset buffer clear output, etc
float FIRFilter_Update(FIRFilter *fir, float inp); //take struct of pointer, current input sample and return filter output
#endif

