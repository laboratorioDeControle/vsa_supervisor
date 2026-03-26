#ifndef __SUPERVISOR_PLANDB_MANUEVER__
#define __SUPERVISOR_PLANDB_MANUEVER__

#include <iostream>
#include <string>
#include <json/json.h>
#include "neptus_msgs/msg/maneuver.hpp"
#include "neptus_msgs/msg/plan_maneuver.hpp"
#include "../guidance/types.h"
#include "../guidance/math_operations.h"


enum maneuver_id_e
{
    maneuver_id_goto = 450,
    maneuver_id_rows = 456,
    maneuver_id_follow_path = 457, // Ripattern
    maneuver_id_station_keeping = 461, // Station Keeping
    maneuver_id_sample = 489,
    maneuver_id_cover_area = 473,
};

enum maneuver_speed_units_e
{
    maneuver_speed_unit_meters_ps = 0,
    maneuver_speed_unit_rpm,
    maneuver_speed_unit_percentage,
    maneuver_speed_unit_size
};

enum maneuver_z_units_e
{
    maneuver_z_unit_none = 0,
    maneuver_z_unit_depth,
    maneuver_z_unit_altitude,
    maneuver_z_unit_height,
    maneuver_z_unit_size
};

enum maneuver_rows_stage_e
{
    maneuver_rows_approach = 0,
    maneuver_rows_start,
    maneuver_rows_up,
    maneuver_rows_begin_curve_up,
    maneuver_rows_end_curve_up,
    maneuver_rows_down,
    maneuver_rows_begin_curve_down,
    maneuver_rows_end_curve_down
};

class Maneuver
{
    public:
        Maneuver();
        Maneuver(neptus_msgs::msg::PlanManeuver plan_maneuver);
        Maneuver(Json::Value maneuver);
        ~Maneuver();

    public:
        void get_rows_points(std::vector<vsa_guidance::guidance_setpoint> *trajectory_vector, vsa_guidance::guidance_setpoint p0);
        void get_rippatern_points(std::vector<vsa_guidance::guidance_setpoint> *trajectory_vector, vsa_guidance::guidance_setpoint p0);
        Json::Value serialize_json();
        void deserialize_json(Json::Value maneuver);

    private:
        inline bool square_curve(void) const
        {
            return (flags & 0x0001) != 0;
        }

        inline bool curve_right(void) const
        {
            return (flags & 0x0002) != 0;
        }

        inline bool curve_left(void) const
        {
            return (flags & 0x0002) == 0;
        }

        void populate_rows_stages(void);

    public:
        std::string id;
        std::string name;
        uint32_t maneuver_imc_id;

        double lat;
        double lon;

        double z;
        maneuver_z_units_e z_unit;
        double speed;
        maneuver_speed_units_e speed_unit;

        double roll;
        double pitch;
        double yaw;

        double bearing;
        double cross_angle;

        float width;
        float length;
        float hstep;

        float radius;
        uint16_t duration;

        uint8_t coff;
        uint8_t alternation;
        uint8_t flags;

        uint16_t timeout;
        std::string custom_string;

        uint8_t syringe0;
        uint8_t syringe1;
        uint8_t syringe2;

        std::vector<vsa_guidance::polygon_vertex_t> polygon;
        std::vector<vsa_guidance::point_t> rows_stages;
        std::vector<vsa_guidance::point_t> follow_path_points;
        vsa_guidance::point_t stage_abs;
    
    private:
        Json::Value maneuver_;


};

#endif