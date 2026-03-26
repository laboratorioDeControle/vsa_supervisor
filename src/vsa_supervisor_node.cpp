#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/empty.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "neptus_msgs/msg/remote_state.hpp"
#include "neptus_msgs/msg/plan_db.hpp"
#include "neptus_msgs/msg/plan_control.hpp"
#include "neptus_msgs/msg/plan_control_state.hpp"
#include "neptus_msgs/msg/estimated_state.hpp"
#include "neptus_msgs/msg/vehicle_state.hpp"
#include "neptus_msgs/msg/entity_monitoring_state.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <imc_ros_bridge/ros_to_imc/VehicleState.h>
#include <json/json.h>

#include "../include/plan_db/plan_db.h"
#include "../include/guidance/guidance.h"
#include "../include/guidance/wgs84.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Supervisor : public rclcpp::Node
{
  public:
    Supervisor()
    : Node("supervisor"), count_(0)
    {
      std::cout << "::Supervisor::" << std::endl;
      
      // Initialization
      declare_parameters();
      initialize_parameters();
      initialize_guidance();
      initialize_plan_control_state();
      initialize_plan_db();
      initialize_publishers();
      initialize_subscribers();
      initialize_vehicle_state();
      initialize_timers();
    }

  public:
    void main_loop(void)
    {
      // Main Loop of Supervisor
      bool health = check_health();

      if(health)
      {
        set_outputs();
        get_inputs();
        execute_state_machine();
      }
    }

  private:
  // Main set functions
    bool check_health(void)
    {
      // std::cout << "::Check Health::" << std::endl;
      return true;
    }

    void execute_state_machine(void)
    {
      if(current_state == neptus_msgs::msg::VehicleState::BOOT)
      {
        state_boot();
      }
      else if(current_state == neptus_msgs::msg::VehicleState::SERVICE)
      {
        state_service();
      }
      else if(current_state == neptus_msgs::msg::VehicleState::CALIBRATION)
      {
        state_calibration();
      }
      else if(current_state == neptus_msgs::msg::VehicleState::MANEUVER)
      {
        state_manuever();
      }
      else if(current_state == neptus_msgs::msg::VehicleState::EXTERNAL)
      {
        state_teleoperation();
      }
      else if(current_state == neptus_msgs::msg::VehicleState::ERROR)
      {
        state_error();
      }
    }

    void get_inputs(void)
    {
      // Convert last odometry msg in odometry type to use in guidance
      odometry = vsa_guidance::msg_odometry_to_odometry_t(msg_current_odometry);
    }

    void set_outputs(void)
    {
      // Publish Entity Monitoring State
      pub_imc_entity_monitoring_state->publish(msg_current_entity_monitoring_state);

      // Publish Vehicle State to Neptus
      pub_imc_vehicle_state->publish(msg_current_vehicle_state);

      // Publish Plan Control State
      pub_imc_plan_control_state->publish(msg_current_plan_control_state);

      // Convert Odometry Input in Neptus Estimate State format and publish
      odometry_to_estimate_state();

      // Send values to actuators (calculated values from automatic controller, parsed values from teleoperation or
      // values to stop vehicle in case of error or abort maneuver).
      //send_signal_to_actuators();
    }

  // Subscribers Callbacks
  private:
    void imc_goto_waypoint_callback(const geometry_msgs::msg::Pose msg)
    {
      std::cout << ":: Chegou mensagem de goto waypoint ::" << std::endl;
    }
    
    void imc_abort_callback(const std_msgs::msg::Empty msg)
    {
      stop_thruster_align_rudders();
      set_state(neptus_msgs::msg::VehicleState::SERVICE);
    }

    void imc_heartbeat_callback(const std_msgs::msg::Empty msg)
    {
    }

    void imc_plan_db_callback(const neptus_msgs::msg::PlanDB msg)
    {
      for(int i = 0; i < (int)msg.plan_spec.maneuvers.size(); i++)
      {
        std::cout << msg.plan_spec.maneuvers[i].maneuver_id << std::endl;
      }

      if(msg.op == plan_db_op_e::plan_db_op_set_plan)
      {        
        // Set plan case
        plan_db_op_result_e operation_result = plan_db_->set_plan(msg.plan_spec);
      }
      else if(msg.op == plan_db_op_e::plan_db_op_get_plan)
      {
        // Get plan case
      }
      else if(msg.op == plan_db_op_e::plan_db_op_delete_plan)
      {
        plan_db_->remove_plan(msg.plan_id, true);
      }
      else if(msg.op == plan_db_op_e::plan_db_op_get_plan_info)
      {
        PlanSpecification* plan = plan_db_->get_plan(msg.plan_id);

        if(plan != nullptr)
        {
          std::cout << plan->serialize_json() << std::endl;
        }
      }
      else if(msg.op == plan_db_op_e::plan_db_op_clear_db)
      {
        plan_db_->clear_db();
      }
      else if(msg.op == plan_db_op_e::plan_db_op_get_db_state_simple)
      {
        // Get plan db state simplfy
      }
      else if(msg.op == plan_db_op_e::plan_db_op_get_db_state_detailed)
      {
        // Get plan db state detailed
      }
      else if(msg.op == plan_db_op_e::plan_db_op_get_db_boot_notification)
      {
        // Boot notification
      }
    }

    void imc_plan_control_callback(const neptus_msgs::msg::PlanControl msg)
    {      
      if(msg.op == 0) // Start Plan
      {
        if((msg.plan_id == "teleoperation-mode") || (msg.plan_id == "teleoperation-16388"))
        {
          msg_current_plan_control_state.man_id = msg.plan_id;
          set_state(neptus_msgs::msg::VehicleState::EXTERNAL);
        }
        else
        {
          current_plain_ = plan_db_->get_plan(msg.plan_id);
          if(current_plain_ != nullptr)
          {
            populate_trajectory_vector();
            set_state(neptus_msgs::msg::VehicleState::CALIBRATION);
          }
        }
      }
      else if(msg.op == 1) // Stop Plan
      {
        stop_thruster_align_rudders();
        set_state(neptus_msgs::msg::VehicleState::SERVICE);
      }
    }

    void teleoperation_callback(const std_msgs::msg::Float64MultiArray msg)
    {
      if(msg.data[0] > 0)
      {
        if(current_state == neptus_msgs::msg::VehicleState::MANEUVER)
        {
          finish_plan_execution();
        }

        if(current_state != neptus_msgs::msg::VehicleState::EXTERNAL)
        {
          set_state(neptus_msgs::msg::VehicleState::EXTERNAL);
        }

        actuators_signals.thruster = msg.data[1];
        actuators_signals.vertical_rudders = msg.data[2];
        actuators_signals.horizontal_rudders = msg.data[3];
      }
      else
      {
        if(current_state == neptus_msgs::msg::VehicleState::EXTERNAL)
        {
          finish_plan_execution();
        }
      }
    }

    void navigation_callback(const nav_msgs::msg::Odometry msg)
    {
      // std::cout << ":: Chegou Msg navigation ::" << std::endl;
      msg_current_odometry = msg;
    }
  
  // Initialize Functions
  private:
    void declare_parameters(void)
    {
      // Teleoperation Output Topic (Teleopration -> Supervisor)
      this->declare_parameter("teleoperation_out_topic", std::string("/teleoperation"));

      // Navigation Output Topic (Navigation -> Supervisor)
      this->declare_parameter("navigation_out_topic", std::string("/dynamics/odometry"));

      // IMC Neptus Bridge Output Topics (Bridge -> Supervisor)
      this->declare_parameter("imc_goto_waypoint_out_topic", std::string("/goto_waypoint"));
      this->declare_parameter("imc_abort_out_topic", std::string("/abort"));
      this->declare_parameter("imc_heartbeat_out_topic", std::string("/imc_heartbeat"));
      this->declare_parameter("imc_plan_db_out_topic", std::string("/plan_db"));
      this->declare_parameter("imc_plan_control_out_topic", std::string("/plan_control"));
      this->declare_parameter("imc_entity_monitoring_state_out_topic", std::string("/entity_monitoring_state"));

      // IMC Neptus Bridge Input Topics (Supervisor -> Bridge)
      this->declare_parameter("imc_heartbeat_in_topic", std::string("/heartbeat"));
      this->declare_parameter("imc_gps_fix_in_topic", std::string("/gps_fix"));
      this->declare_parameter("imc_goto_waypoint_in_topic", std::string("/goto_input"));
      this->declare_parameter("imc_gps_nav_data_in_topic", std::string("/gps_nav_data"));
      this->declare_parameter("imc_remote_state_in_topic", std::string("/remote_state"));
      this->declare_parameter("imc_estimated_state_in_topic", std::string("/estimated_state"));
      this->declare_parameter("imc_vehicle_state_in_topic", std::string("/vehicle_state"));
      this->declare_parameter("imc_plan_control_state_in_topic", std::string("/plan_control_state"));

      // Acutuators Input Topic (Supervisor -> Actuators)
      this->declare_parameter("thruster_in_topic", std::string("/controller/thrusters_setpoints"));
      this->declare_parameter("rudders_in_topic", std::string("/controller/rudders_setpoints"));

      // Health Input Topic (Supervisor -> Health)
      this->declare_parameter("health_in_topic", std::string("/supervisor/health"));

      // Main Loop Rate
      this->declare_parameter("main_loop_rate_hz", 1.0);

      // Motors Command Update Loop Rate
      this->declare_parameter("motors_command_loop_rate_hz", 1.0);

      // Paramenters
      this->declare_parameter("initial_lat", 0.0);
      this->declare_parameter("initial_long", 0.0);

      // Paths
      this->declare_parameter("plan_db_path", "");

      // Guidance Controllers Parameters
      this->declare_parameter("guidance_thruster_kp", 1.0);
      this->declare_parameter("guidance_thruster_ki", 0.0);
      this->declare_parameter("guidance_thruster_kd", 0.0);
      this->declare_parameter("guidance_thruster_max_output", 1.0); //  Max value in RPM  used in thruster
      this->declare_parameter("guidance_thruster_min_output", 0.0); // Min Value in RPM used in thruster
      this->declare_parameter("guidance_thruster_normalize_output", 1.0); // Output normlized values after compute the controller
      this->declare_parameter("guidance_thruster_only_positives_outputs", 1.0); //  Output only positive values after compute the controller

      this->declare_parameter("guidance_pitch_kp", 1.0);
      this->declare_parameter("guidance_pitch_ki", 0.0);
      this->declare_parameter("guidance_pitch_kd", 0.0);
      this->declare_parameter("guidance_pitch_max_output", M_PI_4); //  Max value in Rad used in horizontal rudders
      this->declare_parameter("guidance_pitch_min_output", -M_PI_4); // Min Value in Rad used in horizontal rudders
      this->declare_parameter("guidance_pitch_normalize_output", 1.0); // Output normlized values after compute the controller
      this->declare_parameter("guidance_pitch_only_positives_outputs", 0.0); //  Output only positive values after compute the controller

      this->declare_parameter("guidance_yaw_kp", 1.0);
      this->declare_parameter("guidance_yaw_ki", 0.0);
      this->declare_parameter("guidance_yaw_kd", 0.0);
      this->declare_parameter("guidance_yaw_max_output", M_PI_4); //  Max value in Rad used in vertical rudders
      this->declare_parameter("guidance_yaw_min_output", -M_PI_4); // Min Value in Rad used in vertical rudders
      this->declare_parameter("guidance_yaw_normalize_output", 1.0); // Output normlized values after compute the controller
      this->declare_parameter("guidance_yaw_only_positives_outputs", 0.0); //  Output only positive values after compute the controller

      this->declare_parameter("guidance_xy_error_tolerance", 1.5); // Min distance to consider that the target has been reached
    }

    void initialize_publishers(void)
    {
      pub_imc_estimated_state = this->create_publisher<neptus_msgs::msg::EstimatedState>(this->get_parameter("imc_estimated_state_in_topic").as_string(), 10);
      pub_imc_vehicle_state = this->create_publisher<neptus_msgs::msg::VehicleState>(this->get_parameter("imc_vehicle_state_in_topic").as_string(), 10);
      pub_imc_entity_monitoring_state = this->create_publisher<neptus_msgs::msg::EntityMonitoringState>(this->get_parameter("imc_entity_monitoring_state_out_topic").as_string(), 10);
      pub_imc_plan_control_state = this->create_publisher<neptus_msgs::msg::PlanControlState>(this->get_parameter("imc_plan_control_state_in_topic").as_string(), 10);

      pub_thruster = this->create_publisher<std_msgs::msg::Float64MultiArray>(this->get_parameter("thruster_in_topic").as_string(), 10);
      pub_rudders = this->create_publisher<std_msgs::msg::Float64MultiArray>(this->get_parameter("rudders_in_topic").as_string(), 10);
    }

    void initialize_subscribers(void)
    {
      // IMC Bridge Subscribers
      sub_imc_goto_waypoint = this->create_subscription<geometry_msgs::msg::Pose>(this->get_parameter("imc_goto_waypoint_out_topic").as_string(), 
                                                                                  10, 
                                                                                  std::bind(&Supervisor::imc_goto_waypoint_callback, this, _1));

      sub_imc_abort = this->create_subscription<std_msgs::msg::Empty>(this->get_parameter("imc_abort_out_topic").as_string(), 
                                                                      10, 
                                                                      std::bind(&Supervisor::imc_abort_callback, this, _1));

      sub_imc_heartbeat = this->create_subscription<std_msgs::msg::Empty>(this->get_parameter("imc_heartbeat_out_topic").as_string(), 
                                                                          10, 
                                                                          std::bind(&Supervisor::imc_heartbeat_callback, this, _1));

      sub_imc_plan_db = this->create_subscription<neptus_msgs::msg::PlanDB>(this->get_parameter("imc_plan_db_out_topic").as_string(),
                                                                          10,
                                                                          std::bind(&Supervisor::imc_plan_db_callback, this, _1));

      sub_imc_plan_control = this->create_subscription<neptus_msgs::msg::PlanControl>(this->get_parameter("imc_plan_control_out_topic").as_string(),
                                                                                      10,
                                                                                      std::bind(&Supervisor::imc_plan_control_callback, this, _1));

      // General Subscribers
      sub_teleoperation = this->create_subscription<std_msgs::msg::Float64MultiArray>(this->get_parameter("teleoperation_out_topic").as_string(),
                                                                                      10,
                                                                                      std::bind(&Supervisor::teleoperation_callback, this, _1));
      
      sub_navigation = this->create_subscription<nav_msgs::msg::Odometry>(this->get_parameter("navigation_out_topic").as_string(),
                                                                          10,
                                                                          std::bind(&Supervisor::navigation_callback, this, _1));

    }

    void initialize_timers(void)
    {
      double main_timer_frequency = this->get_parameter("main_loop_rate_hz").as_double();
      double main_timer_period = (1.0 / main_timer_frequency) * 1000.0;
      std::chrono::duration<double, std::milli>  main_timer_period_ms{main_timer_period};
      timer_main_loop = this->create_wall_timer(main_timer_period_ms, std::bind(&Supervisor::main_loop, this));

      double motors_command_timer_frequency = this->get_parameter("motors_command_loop_rate_hz").as_double();
      double motors_command_timer_period = (1.0 / motors_command_timer_frequency) * 1000.0;
      std::chrono::duration<double, std::milli>  motors_command_timer_period_ms{motors_command_timer_period};
      timer_motors_command_loop = this->create_wall_timer(motors_command_timer_period_ms, std::bind(&Supervisor::send_signal_to_actuators, this));
    }

    void initialize_parameters(void)
    {
      initial_latitude_rad = this->get_parameter("initial_lat").as_double();
      initial_longitude_rad = this->get_parameter("initial_long").as_double();

      initial_latitude_deg = initial_latitude_rad * (180.0 / M_PI);
      initial_longitude_deg = initial_longitude_rad * (180.0 / M_PI);

      double main_loop_frequency = this->get_parameter("main_loop_rate_hz").as_double();
      main_loop_dt = 1.0 / main_loop_frequency;
    }

    void initialize_guidance(void)
    {
      vsa_guidance::guidance_pid_params_t thruster_controller_params;
      thruster_controller_params.kp = this->get_parameter("guidance_thruster_kp").as_double();
      thruster_controller_params.ki = this->get_parameter("guidance_thruster_ki").as_double();
      thruster_controller_params.kd = this->get_parameter("guidance_thruster_kd").as_double();
      thruster_controller_params.max_output = this->get_parameter("guidance_thruster_max_output").as_double();
      thruster_controller_params.min_output = this->get_parameter("guidance_thruster_min_output").as_double();
      thruster_controller_params.normalize_output = this->get_parameter("guidance_thruster_normalize_output").as_double() > 0;
      thruster_controller_params.only_positives_outputs = this->get_parameter("guidance_thruster_only_positives_outputs").as_double() > 0;

      vsa_guidance::guidance_pid_params_t pitch_controller_params;
      pitch_controller_params.kp = this->get_parameter("guidance_pitch_kp").as_double();
      pitch_controller_params.ki = this->get_parameter("guidance_pitch_ki").as_double();
      pitch_controller_params.kd = this->get_parameter("guidance_pitch_kd").as_double();
      pitch_controller_params.max_output = this->get_parameter("guidance_pitch_max_output").as_double();
      pitch_controller_params.min_output = this->get_parameter("guidance_pitch_min_output").as_double();
      pitch_controller_params.normalize_output = this->get_parameter("guidance_pitch_normalize_output").as_double() > 0;
      pitch_controller_params.only_positives_outputs = this->get_parameter("guidance_pitch_only_positives_outputs").as_double() > 0;

      vsa_guidance::guidance_pid_params_t yaw_controller_params;
      yaw_controller_params.kp = this->get_parameter("guidance_yaw_kp").as_double();
      yaw_controller_params.ki = this->get_parameter("guidance_yaw_ki").as_double();
      yaw_controller_params.kd = this->get_parameter("guidance_yaw_kd").as_double();
      yaw_controller_params.max_output = this->get_parameter("guidance_yaw_max_output").as_double();
      yaw_controller_params.min_output = this->get_parameter("guidance_yaw_min_output").as_double();
      yaw_controller_params.normalize_output = this->get_parameter("guidance_yaw_normalize_output").as_double() > 0;
      yaw_controller_params.only_positives_outputs = this->get_parameter("guidance_yaw_only_positives_outputs").as_double() > 0;

      double xy_abs_error_tolerance = this->get_parameter("guidance_xy_error_tolerance").as_double();
      double dt = main_loop_dt;

      guidance = new vsa_guidance::Guidance(thruster_controller_params,
                                            pitch_controller_params,
                                            yaw_controller_params,
                                            xy_abs_error_tolerance,
                                            dt);
    }

    void initialize_vehicle_state(void)
    {
        msg_current_vehicle_state.op_mode = current_state;
        msg_current_vehicle_state.maneuver_type = 0xFFFF;
        msg_current_vehicle_state.maneuver_stime = -1;
        msg_current_vehicle_state.maneuver_eta = 0xFFFF;
        msg_current_vehicle_state.error_ents.clear();
        msg_current_vehicle_state.error_count = 0;
        msg_current_vehicle_state.flags = 0;
        msg_current_vehicle_state.last_error.clear();
        msg_current_vehicle_state.last_error_time = -1;
        msg_current_vehicle_state.control_loops = 0;
    }

    void initialize_plan_control_state()
    {
      msg_current_plan_control_state.state = (uint8_t)neptus_msgs::msg::PlanControlState::READY;
      msg_current_plan_control_state.man_id = "";
      msg_current_plan_control_state.man_eta = 0;
      //msg_current_plan_control_state.man_type = neptus_msgs::msg::PlanControlState::
      msg_current_plan_control_state.last_outcome = 0;
      msg_current_plan_control_state.plan_id = "";
      msg_current_plan_control_state.plan_eta = 0;
      msg_current_plan_control_state.plan_progress = 0.0;
    }

    void initialize_plan_db(void)
    {
      plan_db_ = new PlanDB(this->get_parameter("plan_db_path").as_string());
    }

  // General purpose functions
  private:
    void odometry_to_estimate_state(void)
    {
      msg_current_estimated_state = neptus_msgs::msg::EstimatedState();

      double Re = 6371000.0;
      double delta_lat = msg_current_odometry.pose.pose.position.x / Re;
      double delta_lon = msg_current_odometry.pose.pose.position.y / (Re * std::cos(initial_latitude_rad));

      msg_current_estimated_state.lat = initial_latitude_rad;// + delta_lat;
      msg_current_estimated_state.lon = initial_longitude_rad;// + delta_lon;

      tf2::Quaternion quaternion(msg_current_odometry.pose.pose.orientation.x,
            msg_current_odometry.pose.pose.orientation.y,
            msg_current_odometry.pose.pose.orientation.z,
            msg_current_odometry.pose.pose.orientation.w);

      tf2::Matrix3x3 rot;
      rot.setRotation(quaternion);
      double roll, pitch, yaw;
      rot.getRPY(roll, pitch, yaw);

      msg_current_estimated_state.height = msg_current_odometry.pose.pose.position.z;
      msg_current_estimated_state.x = msg_current_odometry.pose.pose.position.x;
      msg_current_estimated_state.y = msg_current_odometry.pose.pose.position.y;
      msg_current_estimated_state.z = msg_current_odometry.pose.pose.position.z;

      msg_current_estimated_state.phi = roll;
      msg_current_estimated_state.theta = pitch;
      msg_current_estimated_state.psi = yaw;

      pub_imc_estimated_state->publish(msg_current_estimated_state);
    }

    void send_signal_to_actuators(void)
    {
      std_msgs::msg::Float64MultiArray thruster;
      std_msgs::msg::Float64MultiArray rudders;

      thruster.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
      thruster.layout.dim[0].size = 1;
      thruster.data.resize(1);
      thruster.data[0] = actuators_signals.thruster;

      rudders.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
      rudders.layout.dim[0].size = 4;
      rudders.data.resize(4);
      rudders.data[0] = actuators_signals.vertical_rudders; // yaw
      rudders.data[1] = -actuators_signals.vertical_rudders; // -yaw
      rudders.data[2] = actuators_signals.horizontal_rudders; // pitch
      rudders.data[3] = -actuators_signals.horizontal_rudders; // -pitch

      pub_thruster->publish(thruster);
      pub_rudders->publish(rudders);
    }

    void populate_trajectory_vector(void)
    {
      trajectory_setpoints.clear();

      for(int i = 0; i < (int)current_plain_->maneuvers.size(); i++)
      {
        Maneuver* maneuver = current_plain_->maneuvers[i];
        double current_maneuver_lat = maneuver->lat;
        double current_maneuver_lon = maneuver->lon;

        vsa_guidance::guidance_setpoint setpoint;
        setpoint.speed = maneuver->speed;
        setpoint.z = maneuver->z;

        vsa_guidance::WGS84::displacement(msg_current_estimated_state.lat, msg_current_estimated_state.lon, 0, 
          current_maneuver_lat, current_maneuver_lon, 0,
          &setpoint.x, &setpoint.y);

        if(maneuver->maneuver_imc_id == maneuver_id_e::maneuver_id_rows)
        {
          maneuver->get_rows_points(&trajectory_setpoints, setpoint);
        }
        else if(maneuver->maneuver_imc_id == maneuver_id_e::maneuver_id_follow_path)
        {
          maneuver->get_rippatern_points(&trajectory_setpoints, setpoint);
        }
        else if(maneuver->maneuver_imc_id == maneuver_id_e::maneuver_id_station_keeping)
        {
          is_station_keeping = true;
          station_keeping_duration = (float)maneuver->duration;
          station_keeping_radius = maneuver->radius;

          trajectory_setpoints.push_back(setpoint);
        }
        else
        {
          trajectory_setpoints.push_back(setpoint);
        }
        
        /*
        std::cout << "msg_current_estimated_state.lat: " << msg_current_estimated_state.lat << std::endl;
        std::cout << "msg_current_estimated_state.lon: " << msg_current_estimated_state.lon << std::endl;
        std::cout << "current_maneuver_lat: " << current_maneuver_lat << std::endl;
        std::cout << "current_maneuver_lon: " << current_maneuver_lon << std::endl;
        std::cout << "==============================================" << std::endl;

        std::cout << "setpoint_x: " << setpoint.x << std::endl;
        std::cout << "setpoint_y: " << setpoint.y << std::endl;
        std::cout << "setpoint_z: " << setpoint.z << std::endl;
        std::cout << "speed: " << setpoint.speed << std::endl;
        std::cout << "==============================================" << std::endl;
        */
      }
    }

    void finish_plan_execution()
    {
      guidance->finish();
      current_plain_ = nullptr;
      is_station_keeping = false;

      int num_trajectory_points = (int)trajectory_setpoints.size();
      for(int i = 0; i < num_trajectory_points; i++)
      {
        trajectory_setpoints.erase(trajectory_setpoints.begin());
      }

      if(timer_station_keeping != nullptr)
      {
        timer_station_keeping->cancel();
      }

      guidance->set_abs_xy_tolerance_error(this->get_parameter("guidance_xy_error_tolerance").as_double());
      
      stop_thruster_align_rudders();
      set_state(neptus_msgs::msg::VehicleState::SERVICE);
    }

    void stop_thruster_align_rudders(void)
    {
      actuators_signals.horizontal_rudders = 0.0;
      actuators_signals.vertical_rudders = 0.0;
      actuators_signals.thruster = 0.0;
    }
  
  // State Machine Functions
  private:
    void set_state(uint8_t state)
    {
      current_state = state;
      msg_current_vehicle_state.op_mode = current_state;
    }

    void state_boot(void)
    {
      rclcpp::sleep_for(std::chrono::seconds(2));
      set_state(neptus_msgs::msg::VehicleState::SERVICE);
    }

    void state_service(void)
    {
    }

    void state_calibration(void)
    {
      rclcpp::sleep_for(std::chrono::seconds(5));
      set_state(neptus_msgs::msg::VehicleState::MANEUVER);
    }

    void state_manuever(void)
    {
      vsa_guidance::guidance_state_e current_guidance_state = guidance->get_state();

      if(current_guidance_state == vsa_guidance::guidance_state_e::guidance_status_IDLE)
      {
        if(current_plain_ != nullptr)
        {
          // Before start a plain
          guidance->set_setpoint(trajectory_setpoints[0]);
          trajectory_setpoints.erase(trajectory_setpoints.begin());
        }
      }

      else if(current_guidance_state == vsa_guidance::guidance_state_e::guidance_status_IN_PROGRESS)
      {
        // Moving to the setpoint case
        guidance->set_abs_xy_tolerance_error(this->get_parameter("guidance_xy_error_tolerance").as_double());
        actuators_signals = guidance->execute(odometry, main_loop_dt);
      }

      else if(current_guidance_state == vsa_guidance::guidance_state_e::guidance_status_TARGET_REACHED)
      {
        // Reach the setpoint case
        if(!is_station_keeping)
        {
          if(trajectory_setpoints.size() != 0)
          {
            guidance->set_setpoint(trajectory_setpoints[0]);
            trajectory_setpoints.erase(trajectory_setpoints.begin());
          }
          else
          {
            finish_plan_execution();
          }
        }
        else
        {
          if(timer_station_keeping == nullptr)
          {
            if(station_keeping_duration > 0)
            {
              std::chrono::duration<double, std::milli>  station_keeping_time_ms{station_keeping_duration * 1000};
              timer_station_keeping = this->create_wall_timer(station_keeping_time_ms, std::bind(&Supervisor::finish_plan_execution, this));
            }
          }
          else
          {
            if((timer_station_keeping->is_canceled()) && (station_keeping_duration > 0))
            {
              std::chrono::duration<double, std::milli>  station_keeping_time_ms{station_keeping_duration * 1000};
              timer_station_keeping = this->create_wall_timer(station_keeping_time_ms, std::bind(&Supervisor::finish_plan_execution, this));
            }
          }

          guidance->set_abs_xy_tolerance_error(station_keeping_radius);
          actuators_signals = guidance->execute(odometry, main_loop_dt);
        }
      }

      else if(current_guidance_state == vsa_guidance::guidance_state_e::guidance_status_ERROR)
      {
        // Error guidance case
      }
    }

    void state_teleoperation(void)
    {
    }

    void state_error(void)
    {
      stop_thruster_align_rudders();
      set_state(neptus_msgs::msg::VehicleState::SERVICE);
    }

    private:
      // IMC Publishers
      rclcpp::Publisher<neptus_msgs::msg::EstimatedState>::SharedPtr pub_imc_estimated_state;
      rclcpp::Publisher<neptus_msgs::msg::PlanControlState>::SharedPtr pub_imc_plan_control_state;
      rclcpp::Publisher<neptus_msgs::msg::VehicleState>::SharedPtr pub_imc_vehicle_state;
      rclcpp::Publisher<neptus_msgs::msg::EntityMonitoringState>::SharedPtr pub_imc_entity_monitoring_state;

      // IMC Subscribers
      rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_imc_goto_waypoint;
      rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_imc_abort;
      rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_imc_heartbeat;
      rclcpp::Subscription<neptus_msgs::msg::PlanDB>::SharedPtr sub_imc_plan_db;
      rclcpp::Subscription<neptus_msgs::msg::PlanControl>::SharedPtr sub_imc_plan_control;
      
      // VSA Structures Publishers
      rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_thruster;
      rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_rudders;

      // VSA Structures Subscribers
      rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_teleoperation;
      rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_navigation;

      // Timers
      rclcpp::TimerBase::SharedPtr timer_main_loop;
      rclcpp::TimerBase::SharedPtr timer_motors_command_loop;
      rclcpp::TimerBase::SharedPtr timer_entity_monitoring_state_send_msg;
      rclcpp::TimerBase::SharedPtr timer_vehicle_state_send_msg;
      rclcpp::TimerBase::SharedPtr timer_station_keeping;

      // Current Recever Msgs
      nav_msgs::msg::Odometry msg_current_odometry;

      neptus_msgs::msg::EstimatedState msg_current_estimated_state;
      neptus_msgs::msg::PlanControlState msg_current_plan_control_state;

      // Current Msgs to Send
      neptus_msgs::msg::VehicleState msg_current_vehicle_state;
      neptus_msgs::msg::EntityMonitoringState msg_current_entity_monitoring_state;

      // State Machine
      uint8_t current_state = neptus_msgs::msg::VehicleState::BOOT;
      uint8_t current_plan_control_state = neptus_msgs::msg::PlanControlState::INITIALIZING;

      double initial_latitude_rad = 0.0; // Initial latitude in radians
      double initial_longitude_rad = 0.0; // Initial longitude in radians
      double initial_latitude_deg = 0.0; // Initial latitude in degrees
      double initial_longitude_deg = 0.0; // Initial longitude in degrees
      double main_loop_dt = 1.0; // Main Loop dt in secconds

      // Station Keeping Parameters
      bool is_station_keeping = false;
      float station_keeping_duration = -1.0;
      float station_keeping_radius = 0.0;

      // PlanDB Object pointer
      PlanDB *plan_db_ = nullptr;
      
      // Current Executed Plan
      PlanSpecification* current_plain_ = nullptr;
      std::vector<vsa_guidance::guidance_setpoint> trajectory_setpoints;

      // Guidance Object pointer
      vsa_guidance::Guidance *guidance = nullptr;

      // Structures used in guidance process
      vsa_guidance::odometry_t odometry;
      vsa_guidance::guidance_output_t actuators_signals;

      size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Supervisor>());
  rclcpp::shutdown();
  return 0;
}