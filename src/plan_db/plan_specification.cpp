#include "../../include/plan_db/plan_specification.h"
#include "../../include/plan_db/maneuver.h"


PlanSpecification::PlanSpecification()
{

}
        
PlanSpecification::PlanSpecification(neptus_msgs::msg::PlanSpecification plan_spec)
{
    plan_id = plan_spec.plan_id;
    vnamespace = plan_spec.vnamespace;
    description = plan_spec.description;
    start_man_id = plan_spec.start_man_id;

    for(int i = 0; i < plan_spec.maneuvers.size(); i++)
    {
        Maneuver *maneuver = new Maneuver(plan_spec.maneuvers[i]);
        maneuvers.push_back(maneuver);
    }
}
        
PlanSpecification::PlanSpecification(Json::Value plan_spec)
{
    deserialize_json(plan_spec);
}
    
PlanSpecification::~PlanSpecification()
{
    for(int i = 0; i < maneuvers.size(); i++)
    {
        delete maneuvers[i];
    }

    maneuvers.clear();
}

Json::Value PlanSpecification::serialize_json()
{
    plan_spec_["plan_id"] = plan_id;
    plan_spec_["description"] = description;
    plan_spec_["vnamespace"] = vnamespace;
    plan_spec_["start_man_id"] = start_man_id;

    Json::Value _maneuvers(Json::arrayValue);

    for(int i = 0; i < maneuvers.size(); i++)
    {   
        // Json::Value maneuver_id;
        Json::Value maneuver = maneuvers[i]->serialize_json();

        //maneuver_id["maneuver_id"] = maneuver;

        _maneuvers.append(maneuver);
    }

    plan_spec_["maneuvers"] = _maneuvers;

    return plan_spec_;
}
        
void PlanSpecification::deserialize_json(Json::Value plan_spec)
{
    maneuvers.clear();

    plan_id = plan_spec["plan_id"].asString();
    description = plan_spec["description"].asString();
    vnamespace = plan_spec["vnamespace"].asString();
    start_man_id = plan_spec["start_man_id"].asString();

    for (auto itr : plan_spec["maneuvers"])
    {
        Maneuver *maneuver = new Maneuver();
        maneuver->deserialize_json(itr);

        maneuvers.push_back(maneuver);
    }
}