#ifndef __GUIDANCE_LPF__
#define __GUIDANCE_LPF__

namespace vsa_guidance
{
  class LowPassFilter
  {
    public:
      LowPassFilter(void);
      LowPassFilter(float gain, float cut_frequency, float sample_period);
      ~LowPassFilter();

      void configure(float gain, float cut_frequency, float sample_period);
      void reset(void);
      float compute(float u);

    public:
      void set_gain(float gain);
      void set_cut_frequency(float cut_frequency);
    
    public: 
      float get_gain(void);
      float get_cut_frequency(void);

    private:
      void calculate_constants(void);
    
    private:
      float _k = 1.0f;
      float _cut_frequency = 1.0f;
      float _gain = 1.0f;
      float _T = 1.0f;

      float _y = 0.0f;
      float _y_z1 = 0.0f;
      float _u_z1 = 0.0f;
      float _a = 0.0f;
      float _b = 0.0f;
      float _c = 0.0f;
      float _d = 0.0f;
  };
}

#endif