#ifndef __GUIDANCE_GUIDANCE__
#define __GUIDANCE_GUIDANCE__

#include <cmath>

#include "types.h"
#include "wgs84.h"
#include "math_operations.h"
#include "control_algorithms/pid.h"

namespace vsa_guidance
{
    class Guidance
    {
        public:
            Guidance();

            Guidance(guidance_pid_params_t thruster_controller_parameters, 
                        guidance_pid_params_t pitch_controller_parameters,
                        guidance_pid_params_t yaw_controller_parameters,
                        double abs_xy_tolerance_error,
                        double dt);

            ~Guidance();

        public:
            guidance_setpoint get_setpoint();
            guidance_state_e get_state();
            double get_abs_xy_tolerance_error(void);
            PID* get_thruster_controller();
            PID* get_pitch_controller();
            PID* get_yaw_controller();
            
        public:
            void set_setpoint(guidance_setpoint setpoint);
            void set_abs_xy_tolerance_error(double value);

        private:
            void set_state(guidance_state_e state);

        public:
            guidance_output_t execute(odometry_t current_odometry, double dt);
            void finish();

        private:
            guidance_state_e state_ = guidance_state_e::guidance_status_IDLE;
            guidance_setpoint setpoint_;

            PID *thruster_controller_ = nullptr;
            PID *pitch_controller_ = nullptr;
            PID *yaw_controller_ = nullptr;

            double abs_xy_tolerance_error_ = 1.5;
    };   
}


#endif