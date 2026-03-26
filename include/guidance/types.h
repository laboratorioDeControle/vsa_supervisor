#ifndef __GUIDANCE_TYPES__
#define __GUIDANCE_TYPES__

#include <vector>

namespace vsa_guidance
{
    struct point_t
    {
        double x; // North
        double y; // East
        double z; // Down
    };

    struct line_t
    {
        point_t initial;
        point_t final;
    };

    struct pose_t
    {
        double x; // North
        double y; // East
        double z; // Down
    };

    struct orientation_t
    {
        double roll;
        double pitch;
        double yaw;
    };

    struct velocity_t
    {
        double x;
        double y;
        double z;
    };


    struct odometry_t
    {
        pose_t pose;
        orientation_t orientation;
        velocity_t linear_velocity;
        velocity_t angular_velocity;

        double xy_abs_velocity;
    };

    struct polygon_vertex_t
    {
        double lat;
        double lon;
    };

    struct guidance_pid_params_t
    {
        double kp;
        double ki;
        double kd;

        double max_output;
        double min_output;

        bool normalize_output;
        bool only_positives_outputs;
    };

    struct guidance_setpoint
    {
        double x;
        double y;
        double z;
        double speed;

        guidance_setpoint operator+(const guidance_setpoint& other) const
        {
            return {x + other.x, y + other.y, z + other.z, speed + other.speed};
        }

        guidance_setpoint operator*(double k) const
        {
            return {x * k, y * k, z * k, speed * k};
        }
    };

    struct guidance_output_t
    {
        double thruster;
        double vertical_rudders;
        double horizontal_rudders;
    };

    enum guidance_state_e
    {
        guidance_status_IDLE = 0,
        guidance_status_IN_PROGRESS,
        guidance_status_TARGET_REACHED,
        guidance_status_ERROR,
        guidance_status_size
    };
}


#endif