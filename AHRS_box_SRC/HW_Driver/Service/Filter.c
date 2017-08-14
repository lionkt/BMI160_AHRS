
#include "Filter.h"



void LowPassFilter_set_cutoff_frequency(LowPass_Filter *f,float sample_freq, float cutoff_freq)
{
    float fr ;
    float ohm ;
    float c ;
	f->_cutoff_freq = cutoff_freq;
	fr = sample_freq/f->_cutoff_freq;
	ohm = tanf(PI/fr);
	c = 1.0f+2.0f*cosf(PI/4.0f)*ohm + ohm*ohm;
    f->_b0 = ohm*ohm/c;
    f->_b1 = 2.0f*f->_b0;
    f->_b2 = f->_b0;
    f->_a1 = 2.0f*(ohm*ohm-1.0f)/c;
    f->_a2 = (1.0f-2.0f*cosf(PI/4.0f)*ohm+ohm*ohm)/c;
	f->_delay_element_1 = f->_delay_element_2 = 0;
}

float LowPassFilter_apply(LowPass_Filter *f,float sample)
{
    // do the filtering
	float output;
    float delay_element_0 = sample - f->_delay_element_1 * f->_a1 - f->_delay_element_2 * f->_a2;
    if (isnan(delay_element_0) || isinf(delay_element_0)) {
        // don't allow bad values to propogate via the filter
        delay_element_0 = sample;
    }
    output = delay_element_0 * f->_b0 + f->_delay_element_1 * f->_b1 + f->_delay_element_2 * f->_b2;
    
    f->_delay_element_2 = f->_delay_element_1;
    f->_delay_element_1 = delay_element_0;

    // return the value.  Should be no need to check limits
    return output;
}


