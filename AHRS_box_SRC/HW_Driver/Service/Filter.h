
#include "math.h"

#define PI  3.1415926f

typedef struct lp{
	float  _cutoff_freq;
	float           _a1;
    float           _a2;
    float           _b0;
    float           _b1;
    float           _b2;
    float           _delay_element_1;        // buffered sample -1
    float           _delay_element_2;        // buffered sample -2
} LowPass_Filter;

void LowPassFilter_set_cutoff_frequency(LowPass_Filter *f,float sample_freq, float cutoff_freq);
float LowPassFilter_apply(LowPass_Filter *f,float sample);



