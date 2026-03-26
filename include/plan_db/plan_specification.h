#ifndef __SUPERVISOR_PLANDB_PLAN_SPECIFICATION__
#define __SUPERVISOR_PLANDB_PLAN_SPECIFICATION__

#include <string>
#include <json/json.h>
#include "neptus_msgs/msg/plan_specification.hpp"

#include "maneuver.h"

class PlanSpecification
{
    public:
        PlanSpecification();
        PlanSpecification(neptus_msgs::msg::PlanSpecification plan_spec);
        PlanSpecification(Json::Value plan_spec);
        ~PlanSpecification();

    public:
        Json::Value serialize_json();
        void deserialize_json(Json::Value plan_spec);

    public:
        std::string plan_id;
        std::string description;
        std::string vnamespace;
        std::string start_man_id;

        std::vector<Maneuver*> maneuvers;

    private:
        Json::Value plan_spec_;
};

#endif