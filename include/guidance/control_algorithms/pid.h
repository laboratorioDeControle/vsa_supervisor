#ifndef __GUIDANCE_PID__
#define __GUIDANCE_PID__

#include "../../../include/guidance/types.h"

namespace vsa_guidance
{
    class PID
    {
        public:
            PID(void);   
            PID(double kp, double ki, double kd);
            PID(double kp, double ki, double kd, double dt);
            PID(double kp, double ki, double kd, double min_output_value, double max_output_value, double dt);
            PID(guidance_pid_params_t parameters, double dt);
            ~PID();

        public:
            void set_dt(double dt);
            void set_setpoint(double setpoint);
            void set_kp(double kp);
            void set_ki(double ki);
            void set_kd(double kd);
            void set_max_output_value(double max_output);
            void set_min_output_value(double min_output);
            void set_saturate_output(bool saturate_output);
            void set_norm_output(bool norm_output);
            void set_only_positive_outputs(bool only_positives_outputs);

            double get_dt(void);
            double get_setpoint(void);
            double get_kp(void);
            double get_ki(void);
            double get_kd(void);
            double get_max_output_value(void);
            double get_min_output_value(void);
            bool get_saturate_output(void);
            bool get_norm_output(void);
            bool get_only_positive_outputs(void);

        public:
            void initialize(double kp, double ki, double kd);
            void initialize(double kp, double ki, double kd, double dt);
            void initialize(double kp, double ki, double kd, double min_output_value, double max_output_value, double dt);
            void reset(void);
            
            double compute(double input);
            double compute(double input, double dt);
            double compute(double input, double setpoint, double dt);
            double compute(double input, double setpoint, double dt, bool wrap_error);

        private:
            double saturate_output(double output);

        private:
            double setpoint_ = 0.0;

            double kp_ = 1.0;
            double ki_ = 0.0;
            double kd_ = 0.0;

            double last_error_ = 0.0;
            double i_ = 0.0;

            double dt_ = -1.0;

            double max_output_value_ = 1.0;
            double min_output_value_ = -1.0;

            bool saturate_output_ = false;
            bool norm_output_ = false;
            bool only_positives_outputs_ = false;
            bool wrap_error_ = false;
    };
}

#endif