#ifndef BIQUAD_FILTER_CASCADE
#define BIQUAD_FILTER_CASCADE

#include <iostream>
#include <vector>
#include "BiquadFilter.h"

class BiquadFilterCascade
{
private:
    std::vector<BiquadFilter> _filters;
    
public:
    BiquadFilterCascade();
    ~BiquadFilterCascade();

    void add_lpf(const double f_sampling, const double f_cutoff);
    void add_differentiator(const double f_sampling, const double f_cutoff);
    double filter(const double x);
    void show_factor();
};

#endif