#include "BiquadFilterCascade.h"

BiquadFilterCascade::BiquadFilterCascade()
{
}

BiquadFilterCascade::~BiquadFilterCascade()
{
}

void BiquadFilterCascade::add_lpf(const double f_sampling, const double f_cutoff){
  BiquadFilter lpf;
  lpf.init_lpf(f_sampling, f_cutoff);

  _filters.push_back(lpf);
}

void BiquadFilterCascade::add_differentiator(const double f_sampling, const double f_cutoff){
  BiquadFilter diff;
  diff.init_differentiator(f_sampling, f_cutoff);

  _filters.push_back(diff);
}

double BiquadFilterCascade::filter(const double x){

  double val = x;
  for (BiquadFilter& f : _filters) {
    val = f.filter(val);
  }

  return val;
}

void BiquadFilterCascade::show_factor(){
  int count = 0;
  for (BiquadFilter& f : _filters) {
    std::cout << "filter #" << ++count << std::endl;
    f.show_factor();
  }
}
