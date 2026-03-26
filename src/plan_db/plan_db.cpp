#include "../../include/plan_db/plan_db.h"


PlanDB::PlanDB(std::string file_path)
{
    file_path_ = file_path;
    
    initialize_plan_db();
}

plan_db_op_result_e PlanDB::set_plan(neptus_msgs::msg::PlanSpecification plan_spec_msg)
{
    PlanSpecification *plan_spec = new PlanSpecification(plan_spec_msg);

    // check if have a plan with same name
    // and remove it
    remove_plan(plan_spec->plan_id, false);

    plans.insert({plan_spec->plan_id, plan_spec});

    export_to_file();

    return plan_db_op_result_e::plan_db_op_result_OK;
}

PlanSpecification* PlanDB::get_plan(std::string plan_id)
{
    if(have_plan(plan_id))
    {
        return plans[plan_id];
    }

    return nullptr;
}

std::vector<std::string> PlanDB::get_all_plans_id()
{   
    std::vector<std::string> keys;

    for(std::map<std::string,PlanSpecification*>::iterator it = plans.begin(); it != plans.end(); ++it)
    {
        keys.push_back(it->first);
    }

    return keys;
}

void PlanDB::remove_plan(std::string plan_id, bool export_file)
{
    if(have_plan(plan_id))
    {
        PlanSpecification* exist_plan = plans[plan_id];
        delete exist_plan;

        plans.erase(plan_id);

        if(export_file)
        {
            export_to_file();
        }
    }
}

void PlanDB::clear_db()
{
    std::vector<std::string> keys = get_all_plans_id();

    for(int i = 0; i < keys.size(); i++)
    {
        remove_plan(keys[i], false);
    }

    export_to_file();
}

void PlanDB::export_to_file()
{
    serialize_json();

    Json::StreamWriterBuilder builder;
    builder["commentStyle"] = "None";
    builder["indentation"] = "   ";

    std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
    std::ofstream outputFileStream(file_path_);

    writer->write(plan_db_, &outputFileStream);
    
    outputFileStream.close();
}

void PlanDB::initialize_plan_db()
{
    std::ifstream plan_db_file(file_path_, std::ifstream::binary);

    if(plan_db_file.good())
    {
        plan_db_file >> plan_db_;
        deserialize_json();
    }

    plan_db_file.close();
}

void PlanDB::serialize_json()
{
    plan_db_.clear();

    for (auto plan : plans)
    {
        plan_db_[plan.first] = plan.second->serialize_json();
    }
}

void PlanDB::deserialize_json()
{
    for (auto itr : plan_db_)
    {
        PlanSpecification *plan_spec = new PlanSpecification(itr);
        plans.insert({plan_spec->plan_id, plan_spec});
    }
}

bool PlanDB::have_plan(std::string plan_id)
{
    return plans.find(plan_id) != plans.end();
}

uint16_t PlanDB::plan_count()
{
    return (uint16_t)plan_db_.getMemberNames().size();
}

uint32_t PlanDB::size_of_all_plans()
{
    return 100;
}