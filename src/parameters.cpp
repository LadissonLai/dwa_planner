// Copyright 2020 amsl

#include <algorithm>
#include <string>

#include "dwa_planner/dwa_planner.h"

void DWAPlanner::load_params(void)
{
  // - A -
  local_nh_.param<double>("ANGLE_RESOLUTION", angle_resolution_, 0.087);
  // - F -
  local_nh_.param<double>("FOOTPRINT_PADDING", footprint_padding_, 0.01);
  // - G -
  local_nh_.param<double>("GOAL_THRESHOLD", dist_to_goal_th_, 0.1);
  // - M -
  local_nh_.param<double>("MAX_ACCELERATION", max_acceleration_, 0.5);
  local_nh_.param<double>("MAX_DECELERATION", max_deceleration_, 2.0);

  local_nh_.param<double>("MAX_VELOCITY", max_velocity_, 1.0);


  local_nh_.param<double>("MIN_VELOCITY", min_velocity_, 0.0);

  // - O -
  local_nh_.param<double>("OBSTACLE_COST_GAIN", obs_cost_gain_, 1.0);
  local_nh_.param<double>("OBS_RANGE", obs_range_, 2.5);

  local_nh_.param<double>("PREDICT_TIME", predict_time_, 3.0);
  // - R -
  local_nh_.param<std::string>("ROBOT_FRAME", robot_frame_, std::string("base_link"));
  // - S -
  local_nh_.param<double>("SIM_PERIOD", sim_period_, 0.1);
  
  local_nh_.param<double>("SPEED_COST_GAIN", speed_cost_gain_, 0.4);

  // - T -
  local_nh_.param<double>("TARGET_VELOCITY", target_velocity_, 0.55);
  local_nh_.param<double>("TO_GOAL_COST_GAIN", to_goal_cost_gain_, 0.8);
  local_nh_.param<double>("TO_GOAL_ORIENTATION_COST_GAIN", to_goal_orientation_cost_gain_, 0.8);

  // - V -
  local_nh_.param<int>("VELOCITY_SAMPLES", velocity_samples_, 3);
  local_nh_.param<double>("V_PATH_WIDTH", v_path_width_, 0.05);

  target_velocity_ = std::min(target_velocity_, max_velocity_);

  // ackerman parameters
  local_nh_.param<int>("STEER_ANGLE_SAMPLES", steer_angle_samples_, 10);
  local_nh_.param<int>("VELOCITY_SAMPLES", velocity_samples_, 5);
  local_nh_.param<double>("PREDICT_TIME", predict_time_, 3.0);
  local_nh_.param<double>("ROBOT_WIDTH", robot_width_, 0.5);
  local_nh_.param<double>("ROBOT_LENGTH", robot_length_, 0.8);
  local_nh_.param<double>("WHEELBASE", wheelbase_, 0.5);
  local_nh_.param<double>("MAX_STEER_ANGLE", max_steer_angle_, 0.5);
  local_nh_.param<double>("FRONT_OVERHANG", front_overhang_, 0.25);
  local_nh_.param<double>("REAR_OVERHANG", rear_overhang_, 0.25);
  local_nh_.param<int>("SIM_TIME_SAMPLES", sim_time_samples_, 10); // 每条轨迹的采样点数

}

void DWAPlanner::print_params(void)
{
  // - A -
  ROS_INFO_STREAM("ANGLE_RESOLUTION: " << angle_resolution_);
  // - F -
  ROS_INFO_STREAM("FOOTPRINT_PADDING: " << footprint_padding_);
  // - G -
  ROS_INFO_STREAM("GOAL_THRESHOLD: " << dist_to_goal_th_);

  // - M -
  ROS_INFO_STREAM("MAX_ACCELERATION: " << max_acceleration_);
  ROS_INFO_STREAM("MAX_DECELERATION: " << max_deceleration_);

  ROS_INFO_STREAM("MAX_VELOCITY: " << max_velocity_);

  // - O -
  ROS_INFO_STREAM("OBSTACLE_COST_GAIN: " << obs_cost_gain_);
  ROS_INFO_STREAM("OBS_RANGE: " << obs_range_);

  ROS_INFO_STREAM("PREDICT_TIME: " << predict_time_);
  // - R -
  ROS_INFO_STREAM("ROBOT_FRAME: " << robot_frame_);
  // - S -
  ROS_INFO_STREAM("SIM_PERIOD: " << sim_period_);
  
  ROS_INFO_STREAM("SPEED_COST_GAIN: " << speed_cost_gain_);

  // - T -
  ROS_INFO_STREAM("TARGET_VELOCITY: " << target_velocity_);
  ROS_INFO_STREAM("TO_GOAL_COST_GAIN: " << to_goal_cost_gain_);
  ROS_INFO_STREAM("TO_GOAL_ORIENTATION_COST_GAIN: " << to_goal_orientation_cost_gain_);

  // - V -
  ROS_INFO_STREAM("VELOCITY_SAMPLES: " << velocity_samples_);
  ROS_INFO_STREAM("V_PATH_WIDTH: " << v_path_width_);

  // ackerman parameters
  ROS_INFO_STREAM("+++++++++++++++++++ACKERMAN PARAMS+++++++++++++++++++++++");
  ROS_INFO_STREAM("STEER_ANGLE_SAMPLES: " << steer_angle_samples_);
  ROS_INFO_STREAM("VELOCITY_SAMPLES: " << velocity_samples_);
  ROS_INFO_STREAM("PREDICT_TIME: " << predict_time_);
  ROS_INFO_STREAM("ROBOT_WIDTH: " << robot_width_);
  ROS_INFO_STREAM("ROBOT_LENGTH: " << robot_length_);
  ROS_INFO_STREAM("WHEELBASE: " << wheelbase_);
  ROS_INFO_STREAM("MAX_STEER_ANGLE: " << max_steer_angle_);
  ROS_INFO_STREAM("FRONT_OVERHANG: " << front_overhang_);
  ROS_INFO_STREAM("REAR_OVERHANG: " << rear_overhang_);
  ROS_INFO_STREAM("SIM_TIME_SAMPLES: " << sim_time_samples_);
  ROS_INFO_STREAM("+++++++++++++++++++ACKERMAN PARAMS+++++++++++++++++++++++");
}
