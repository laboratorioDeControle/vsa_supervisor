#include "../../../include/guidance/filters/low_pass_filter.h"

using namespace vsa_guidance;

LowPassFilter::LowPassFilter(void)
{
  
}

LowPassFilter::LowPassFilter(float gain, float cut_frequency, float sample_period)
{
  _gain = gain;
  _cut_frequency = cut_frequency;
  _T = sample_period;

  this->calculate_constants();
}

LowPassFilter::~LowPassFilter()
{
  
}

void LowPassFilter::configure(float gain, float cut_frequency, float sample_period)
{
  _gain = gain;
  _cut_frequency = cut_frequency;
  _T = sample_period;

  this->calculate_constants();
}

void LowPassFilter::reset(void)
{
    _y = 0.0f;
    _y_z1 = 0.0f;
    _u_z1 = 0.0f;
}

float LowPassFilter::compute(float u)
{
  _y = (u * _c) + (_u_z1 * _c) - (_y_z1 * _d);

  _y_z1 = _y;
  _u_z1 = u;

  return _y;
}

void LowPassFilter::set_gain(float gain)
{
  _gain = gain;
  this->calculate_constants();
}

void LowPassFilter::set_cut_frequency(float cut_frequency)
{
  _cut_frequency = cut_frequency;
  this->calculate_constants();
}

float LowPassFilter::get_gain(void)
{
  return _gain;
}

float LowPassFilter::get_cut_frequency(void)
{
  return _cut_frequency;
}

void LowPassFilter::calculate_constants(void)
{
  _k = _gain * _cut_frequency; 
  _a = (_T * _cut_frequency) - 2;
  _b = (_T * _cut_frequency) + 2;
  _c = (_k * _T) / _b;
  _d = _a / _b;
}