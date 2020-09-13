#include "BiquadFilter.h"

BiquadFilter::BiquadFilter()
{
  for (int i = 0; i < 3; i++)
  {
    _a[i] = 0;
    _b[i] = 0;
    _x[i] = 0;
    _y[i] = 0;
  }
}

BiquadFilter::~BiquadFilter()
{
}

void BiquadFilter::init_lpf(const double f_sampling, const double f_cutoff)
{
  // LPFになるように_a,_bを決定する

  double w_cutoff = 2 * M_PI * f_cutoff;
  double T_sampling = 1 / f_sampling;

  // プリワーピング：デジタルのカットオフ周波数を、アナログフィルタでのカットオフ周波数に変換する
  double Omega_cutoff = pre_warping(T_sampling, w_cutoff);

  // アナログLPFの伝達関数の係数を求める
  // H(s) = Cs^2 + Ds + E / s^2 + As + B
  double A, B, C, D, E;
  calc_lpf_transfer_function_factor(w_cutoff, A, B, C, D, E);
  calc_digital_factor_from_analog(T_sampling, C, D, E, A, B, _a[0], _a[1], _a[2], _b[1], _b[2]);
}

void BiquadFilter::init_differentiator(const double f_sampling, const double f_cutoff)
{
  // 微分器になるように_a,_bを決定する

  double w_cutoff = 2 * M_PI * f_cutoff;
  double T_sampling = 1 / f_sampling;

  double Omega_cutoff = pre_warping(T_sampling, w_cutoff);
  // std::cout << Omega_cutoff << std::endl;

  double A, B, C, D, E;
  calc_differentiator_transfer_function_factor(w_cutoff, A, B, C, D, E);
  calc_digital_factor_from_analog(T_sampling, C, D, E, A, B, _a[0], _a[1], _a[2], _b[1], _b[2]);
}

double BiquadFilter::pre_warping(const double T, const double cutoff_digital)
{
  double cutoff_analog = 2.0 / T * tan(cutoff_digital * T / 2.0);
  return cutoff_analog;
}

void BiquadFilter::calc_lpf_transfer_function_factor(const double w_cutoff,double& A, double& B, double& C, double& D, double& E)
{
  /* アナログ基準LPF H(p) から
  * 希望のカットオフ周波数 w_cutoffを持つアナログLPF H(s) を求める
  * 
  * アナログ基準LPF H(p) = 1 / p^2 + 1.41p + 1
  *     ↓
  * アナログLPF H(s) = Cs^2 + Ds + E / s^2 + As + B
  */

  A = 1.41 * w_cutoff;
  B = std::pow(w_cutoff, 2);
  C = 0;
  D = 0;
  E = std::pow(w_cutoff, 2);
}

void BiquadFilter::calc_differentiator_transfer_function_factor(const double w_cutoff,double& A, double& B, double& C, double& D, double& E)
{
  /* アナログ基準LPF H(p) から微分器を求める
  * 微分器は微分+ローパス、つまりアナログLPFの伝達関数にsをかけたもの
  * 
  * アナログ基準LPF H(p) = 1 / p^2 + 1.41p + 1
  *     ↓
  * 微分器 H(s) = Cs^2 + Ds + E / s^2 + As + B
  */

  A = 1.41 * w_cutoff;
  B = std::pow(w_cutoff, 2);
  C = 0;
  D = std::pow(w_cutoff, 2);
  E = 0;
}

void BiquadFilter::calc_digital_factor_from_analog(const double T_sampling, const double C, const double D, const double E, const double A, const double B,
                                       double &a0, double &a1, double &a2, double &b1, double &b2)
{
  // 双一次S-Z変換を行い、アナログ伝達関数→デジタル伝達関数に変換する
  // H(s) = Cs^2 + Ds + E / s^2 + As + B
  //     ↓
  // H(z) = a0 + a1*z^-1 + a2*z^-2 / 1 + b1*z^-1 + b2*z^-2

  double alpha = 2 / T_sampling;
  double beta = C * std::pow(alpha, 2);
  double G = 1 / (std::pow(alpha, 2) + A * alpha + B);

  a0 = G * (beta + D * alpha + E);
  a1 = G * 2 * (E - beta);
  a2 = G * (beta - D * alpha + E);
  b1 = 2 * G * (B - std::pow(alpha, 2));
  b2 = G * (std::pow(alpha, 2) - A * alpha + B);
}

double BiquadFilter::filter(const double x)
{
  // 差分方程式を計算する

  // x[], y[]を1ステップずらす
  _x[2] = _x[1];
  _x[1] = _x[0];
  _y[2] = _y[1];
  _y[1] = _y[0];

  // 現在ステップのx,yを差分方程式で求める
  _x[0] = x;
  _y[0] = _a[0] * _x[0] + _a[1] * _x[1] + _a[2] * _x[2] - _b[1] * _y[1] - _b[2] * _y[2];

  return _y[0];
}
