#ifndef __SUPERVISOR_PLAN_DB__
#define __SUPERVISOR_PLAN_DB__

#include <string>
#include <json/json.h>
#include <json/writer.h>
#include <iostream>
#include <fstream>
#include <map>

#include "neptus_msgs/msg/plan_specification.hpp"

#include "plan_specification.h"

enum plan_db_op_e
{
    plan_db_op_set_plan = 0,
    plan_db_op_get_plan,
    plan_db_op_delete_plan,
    plan_db_op_get_plan_info,
    plan_db_op_clear_db,
    plan_db_op_get_db_state_simple,
    plan_db_op_get_db_state_detailed,
    plan_db_op_get_db_boot_notification,
    plan_db_op_size
};

enum plan_db_op_result_e
{
    plan_db_op_result_OK = 0,
};

class PlanDB
{
    public:
        PlanDB(std::string file_path);
        ~PlanDB();

    public:
        plan_db_op_result_e set_plan(neptus_msgs::msg::PlanSpecification plan_spec_msg);
        PlanSpecification* get_plan(std::string plan_id);
        std::vector<std::string> get_all_plans_id();
        void remove_plan(std::string plan_id, bool export_file);
        void clear_db();
        void export_to_file();

    private:
        void initialize_plan_db();
        void serialize_json();
        void deserialize_json();

        bool have_plan(std::string plan_id);
        
        uint16_t plan_count();
        uint32_t size_of_all_plans();

    private:
        std::string file_path_;
        Json::Value plan_db_;

        std::map<std::string, PlanSpecification*> plans;
};

#endif