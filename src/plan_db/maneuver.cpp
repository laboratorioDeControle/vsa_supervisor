#include "../../include/plan_db/maneuver.h"

Maneuver::Maneuver()
{

}

Maneuver::Maneuver(neptus_msgs::msg::PlanManeuver maneuver)
{
    id = maneuver.maneuver_id;

    name = maneuver.maneuver.maneuver_name;
    maneuver_imc_id = maneuver.maneuver.maneuver_imc_id;
    
    lat = maneuver.maneuver.lat;
    lon = maneuver.maneuver.lon;
    z = maneuver.maneuver.z;
    z_unit = (maneuver_z_units_e)maneuver.maneuver.z_units;
    speed = maneuver.maneuver.speed;
    speed_unit = (maneuver_speed_units_e)maneuver.maneuver.speed_units;
    
    roll = maneuver.maneuver.roll;
    pitch = maneuver.maneuver.pitch;
    yaw = maneuver.maneuver.yaw;

    timeout = maneuver.maneuver.timeout;
    custom_string = maneuver.maneuver.custom_string;

    syringe0 = maneuver.maneuver.syringe0;
    syringe1 = maneuver.maneuver.syringe1;
    syringe2 = maneuver.maneuver.syringe2;

    bearing = maneuver.maneuver.bearing;
	cross_angle = maneuver.maneuver.cross_angle;
	
    width = maneuver.maneuver.width;
	length = maneuver.maneuver.length;
    hstep = maneuver.maneuver.hstep;

	coff = maneuver.maneuver.coff;
	alternation = maneuver.maneuver.alternation;
	flags = maneuver.maneuver.flags;

    duration = maneuver.maneuver.duration;
    radius = maneuver.maneuver.radius;

    for(int i = 0; i < (int)maneuver.maneuver.polygon.size(); i++)
    {
        vsa_guidance::polygon_vertex_t vertex;

        vertex.lat = maneuver.maneuver.polygon[i].lat;
        vertex.lon = maneuver.maneuver.polygon[i].lon;

        polygon.push_back(vertex);
    }

    for(int i = 0; i < (int)maneuver.maneuver.points.size(); i++)
    {
        vsa_guidance::point_t point;
        point.x = maneuver.maneuver.points[i].x;
        point.y = maneuver.maneuver.points[i].y;
        point.z = maneuver.maneuver.points[i].z;

        follow_path_points.push_back(point);
    }
}

Maneuver::Maneuver(Json::Value maneuver)
{
    deserialize_json(maneuver);
}

Maneuver::~Maneuver()
{

}

void Maneuver::get_rows_points(std::vector<vsa_guidance::guidance_setpoint> *trajectory_vector, vsa_guidance::guidance_setpoint p0)
{   
    populate_rows_stages();

    int num_lines = (int)(width / hstep) + 1;
    int num_points = (3 * num_lines);

    // std::cout << "p0: " << p0.x << "," << p0.y << std::endl;

    if(!square_curve())
    {
        num_points = (2 * num_lines) + 1;
    }

    int stage_index = 0;
    for(int i = 0; i < num_points; i++)
    {
        if(stage_index == (int)rows_stages.size())
        {
            stage_index = 2;
        }

        vsa_guidance::guidance_setpoint point;
        vsa_guidance::point_t stage = rows_stages.at(stage_index);

        double adx = stage.x;
        double ady = stage.y;

        stage_abs.x += adx;
        stage_abs.y += ady;

        double dx = stage_abs.x;
        double dy = stage_abs.y;

        // std::cout << "stage_abs(" << i << "): " << stage_abs.x << "," << stage_abs.y << std::endl;
        vsa_guidance::rotate_angle(bearing, false, dx, dy);

        // std::cout << "dx_dy(" << i << "): " << dx << "," << dy << std::endl;

        point.x = p0.x + dx;
        point.y = p0.y + dy;
        point.z = p0.z;
        point.speed = p0.speed;

        stage_index++;

        trajectory_vector->push_back(point);

        // std::cout << "point(" << i << "): " << point.x << "," << point.y << std::endl;
    }
}

void Maneuver::get_rippatern_points(std::vector<vsa_guidance::guidance_setpoint> *trajectory_vector, vsa_guidance::guidance_setpoint p0)
{
    for(int i = 0; i < (int)follow_path_points.size(); i++)
    {
        vsa_guidance::guidance_setpoint point;
        point.x = p0.x + follow_path_points.at(i).x;
        point.y = p0.y + follow_path_points.at(i).y;
        point.z = p0.z;
        point.speed = p0.speed;

        trajectory_vector->push_back(point);
    }
}

Json::Value Maneuver::serialize_json()
{
    maneuver_["maneuver_id"] = id;
    maneuver_["maneuver_imc_id"] = maneuver_imc_id;
    maneuver_["lat"] = lat;
    maneuver_["lon"] = lon;
    maneuver_["z"] = z;
    maneuver_["z_units"] = (uint8_t)z_unit;
    maneuver_["speed"] = speed;
    maneuver_["speed_units"] = (uint8_t)speed_unit;
    maneuver_["roll"] = roll;
    maneuver_["pitch"] = pitch;
    maneuver_["yaw"] = yaw;

    maneuver_["bearing"] = bearing;
    maneuver_["cross_angle"] = cross_angle;
    maneuver_["width"] = width;
    maneuver_["length"] = length;
    maneuver_["hstep"] = hstep;
    maneuver_["coff"] = coff;
    maneuver_["alternation"] = alternation;
    maneuver_["flags"] = flags;
    
    maneuver_["timeout"] = timeout;
    maneuver_["custom_string"] = custom_string;
    maneuver_["syringe0"] = syringe0;
    maneuver_["syringe1"] = syringe1;
    maneuver_["syringe2"] = syringe2;

    maneuver_["duration"] = duration;
    maneuver_["radius"] = radius;

    Json::Value _polygon(Json::arrayValue);
    for(int i = 0; i < (int)polygon.size(); i++)
    {
        Json::Value vertex;

        vertex["lat"] = polygon[i].lat;
        vertex["lon"] = polygon[i].lon;

        _polygon.append(vertex);
    }

    maneuver_["polygon"] = _polygon;

    Json::Value _points(Json::arrayValue);
    for(int i = 0; i < (int)follow_path_points.size(); i++)
    {
        Json::Value point;
        point["x"] = follow_path_points[i].x;
        point["y"] = follow_path_points[i].y;
        point["z"] = follow_path_points[i].z;

        _points.append(point);
    }

    maneuver_["follow_path_points"] = _points;

    return maneuver_;
}
        
void Maneuver::deserialize_json(Json::Value maneuver)
{
    polygon.clear();
    follow_path_points.clear();

    id = maneuver["maneuver_id"].asString();
    maneuver_imc_id = (uint32_t)maneuver["maneuver_imc_id"].asUInt64();
    lat = maneuver["lat"].asDouble();
    lon = maneuver["lon"].asDouble();
    z = maneuver["z"].asDouble();
    z_unit = (maneuver_z_units_e)maneuver["z_units"].asUInt64();
    speed = maneuver["speed"].asDouble();
    speed_unit = (maneuver_speed_units_e)maneuver["speed_units"].asUInt64();
    roll = maneuver["roll"].asDouble();
    pitch = maneuver["pitch"].asDouble();
    yaw = maneuver["yaw"].asDouble();

    bearing = maneuver["bearing"].asDouble();
    cross_angle = maneuver["cross_angle"].asDouble();
    width = maneuver["width"].asFloat();
    length = maneuver["length"].asFloat();
    hstep = maneuver["hstep"].asFloat();
    coff = (uint8_t)maneuver["coff"].asUInt64();
    alternation = (uint8_t)maneuver["alternation"].asUInt64();
    flags = (uint8_t)maneuver["flags"].asUInt64();

    timeout = (uint16_t)maneuver["timeout"].asUInt64();
    custom_string = maneuver["custom_string"].asString();
    syringe0 = (uint8_t)maneuver["syringe0"].asUInt64();
    syringe1 = (uint8_t)maneuver["syringe1"].asUInt64();
    syringe2 = (uint8_t)maneuver["syringe2"].asUInt64();

    duration = (uint16_t)maneuver["duration"].asUInt64();
    radius = maneuver["radius"].asFloat();

    for (auto itr : maneuver["polygon"])
    {
        vsa_guidance::polygon_vertex_t vertex;
        vertex.lat = itr["lat"].asDouble();
        vertex.lon = itr["lon"].asDouble();
        polygon.push_back(vertex);
    }

    for (auto itr : maneuver["follow_path_points"])
    {
        vsa_guidance::point_t point;
        point.x = itr["x"].asDouble();
        point.y = itr["y"].asDouble();
        point.z = itr["z"].asDouble();
        follow_path_points.push_back(point);
    }
}

void Maneuver::populate_rows_stages(void)
{
    rows_stages.clear();

    double curve_sign = 1.0;
    if(curve_left())
    {
        curve_sign = -1.0;
    }

    double alt_frac_up = 0.01 * (double)alternation;
    double alt_frac_down = 2 - (0.01 * (double)alternation);

    // Stage 0: Approach
    vsa_guidance::point_t approach;
    approach.x = -coff;
    approach.y = 0;
    rows_stages.push_back(approach);

    // Stage 1: Start
    vsa_guidance::point_t start;
    start.x = coff;
    start.y = 0;
    rows_stages.push_back(start);

    // Stage 2: Up
    vsa_guidance::point_t up;
    up.x = length + coff;
    up.y = 0;
    rows_stages.push_back(up);

    // Stage 3: Begin Curve Up
    vsa_guidance::point_t begin_curve_up;
    begin_curve_up.x = 0;
    begin_curve_up.y = curve_sign * alt_frac_up * hstep;

    // Stage 4: End Curve Up
    vsa_guidance::point_t end_curve_up;
    end_curve_up.x = -coff;
    end_curve_up.y = 0;

    if(!square_curve())
    {
        vsa_guidance::rotate_angle(cross_angle, curve_left(), begin_curve_up.x, begin_curve_up.y);
        begin_curve_up.x -= coff;
        rows_stages.push_back(begin_curve_up);
    }
    else
    {
        vsa_guidance::rotate_angle(cross_angle, curve_left(), begin_curve_up.x, begin_curve_up.y);
        rows_stages.push_back(begin_curve_up);
        rows_stages.push_back(end_curve_up);
    }
    
    // Stage 5: Down
    vsa_guidance::point_t down;
    down.x = -up.x;
    down.y = 0;
    rows_stages.push_back(down);

    // Stage 6: Begin Down Curve
    vsa_guidance::point_t begin_curve_down;
    begin_curve_down.x = 0;
    begin_curve_down.y = curve_sign * alt_frac_down * hstep;

    // Stage 7: End Down Curve
    vsa_guidance::point_t end_curve_down;
    end_curve_down.x = -end_curve_up.x;
    end_curve_down.y = 0;

    if(!square_curve())
    {
        vsa_guidance::rotate_angle(cross_angle, curve_left(), begin_curve_down.x, begin_curve_down.y);
        begin_curve_down.x += coff;
        rows_stages.push_back(begin_curve_down);
    }
    else
    {
        vsa_guidance::rotate_angle(cross_angle, curve_left(), begin_curve_down.x, begin_curve_down.y);
        rows_stages.push_back(begin_curve_down);
        rows_stages.push_back(end_curve_down);
    }

    stage_abs.x = 0;
    stage_abs.y = 0;
}