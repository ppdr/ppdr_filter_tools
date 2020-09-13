#ifndef BIQUAD_FILTER
#define BIQUAD_FILTER

#include <iostream>
#include <cmath>

class BiquadFilter
{
private:
    double _a[3], _b[3];
    double _x[3], _y[3];

    double pre_warping(const double T, const double cutoff_digital);
    void calc_lpf_transfer_function_factor(const double w_cutoff,double& A, double& B, double& C, double& D, double& E);
    void calc_differentiator_transfer_function_factor(const double w_cutoff,double& A, double& B, double& C, double& D, double& E);
    void calc_digital_factor_from_analog(const double T_sampling, const double C, const double D, const double E, const double A, const double B, double& a0, double& a1, double& a2, double&b1, double& b2);
public:
    BiquadFilter();
    ~BiquadFilter();

    void init_lpf(const double f_sampling, const double w_cutoff);
    void init_differentiator(const double f_sampling, const double w_cutoff);
    double filter(const double x);
    void show_factor();
};

#endif