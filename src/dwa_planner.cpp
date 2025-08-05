// Copyright 2020 amsl

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "dwa_planner/dwa_planner.h"

DWAPlanner::DWAPlanner(void)
    : local_nh_("~"), 
      use_speed_cost_(false)
{
  load_params();

  ROS_INFO("=== DWA Planner ===");
  print_params();

  candidate_trajectories_pub_ = local_nh_.advertise<visualization_msgs::MarkerArray>("candidate_trajectories", 1);
  selected_trajectory_pub_ = local_nh_.advertise<visualization_msgs::Marker>("selected_trajectory", 1);
  predict_footprints_pub_ = local_nh_.advertise<visualization_msgs::MarkerArray>("predict_footprints", 1);
  robot_polygon_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/robot_polygon_marker", 1);
  obstacle_points_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("obstacle_points", 1);
}

DWAPlanner::State::State(void) : x_(0.0), y_(0.0), yaw_(0.0), velocity_(0.0), yawrate_(0.0) {}

DWAPlanner::State::State(const double x, const double y, const double yaw, const double velocity, const double yawrate)
    : x_(x), y_(y), yaw_(yaw), velocity_(velocity), yawrate_(yawrate)
{
}

DWAPlanner::Window::Window(void) : min_velocity_(0.0), max_velocity_(0.0), min_yawrate_(0.0), max_yawrate_(0.0) {}

void DWAPlanner::Window::show(void)
{
  ROS_INFO_STREAM("Window:");
  ROS_INFO_STREAM("\tVelocity:");
  ROS_INFO_STREAM("\t\tmax: " << max_velocity_);
  ROS_INFO_STREAM("\t\tmin: " << min_velocity_);
  ROS_INFO_STREAM("\tYawrate:");
  ROS_INFO_STREAM("\t\tmax: " << max_yawrate_);
  ROS_INFO_STREAM("\t\tmin: " << min_yawrate_);
}

DWAPlanner::Cost::Cost(void) : obs_cost_(0.0), to_goal_cost_(0.0), to_goal_orientation_cost_(0.0), speed_cost_(0.0), path_cost_(0.0), total_cost_(0.0)
{
}

DWAPlanner::Cost::Cost(
    const float obs_cost, const float to_goal_cost, const float speed_cost, const float path_cost,
    const float total_cost)
    : obs_cost_(obs_cost), to_goal_cost_(to_goal_cost), to_goal_orientation_cost_(0), speed_cost_(speed_cost), path_cost_(path_cost),
      total_cost_(total_cost)
{
}

void DWAPlanner::Cost::show(void)
{
  ROS_INFO_STREAM("Cost: " << total_cost_);
  ROS_INFO_STREAM("\tObs cost: " << obs_cost_);
  ROS_INFO_STREAM("\tGoal cost: " << to_goal_cost_);
  ROS_INFO_STREAM("\tGoal Orientation cost: " << to_goal_orientation_cost_);
  ROS_INFO_STREAM("\tSpeed cost: " << speed_cost_);
  ROS_INFO_STREAM("\tPath cost: " << path_cost_);
}

void DWAPlanner::Cost::calc_total_cost(void) { total_cost_ = obs_cost_ + to_goal_cost_ + to_goal_orientation_cost_ + speed_cost_ + path_cost_; }


void DWAPlanner::publishRobotMarker(const std_msgs::Header& header, const geometry_msgs::Pose& pose) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = header.frame_id; // 使用姿态消息的帧ID
  marker.header.stamp = ros::Time::now();

  marker.ns = "robot_visualization";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE; // 立方体最适合表示矩形
  marker.action = visualization_msgs::Marker::ADD;

  // 设置Marker的姿态与机器人姿态一致
  marker.pose.position.x = pose.position.x;
  marker.pose.position.y = pose.position.y;
  marker.pose.position.z = pose.position.z;

  marker.pose.orientation.x = pose.orientation.x;
  marker.pose.orientation.y = pose.orientation.y;
  marker.pose.orientation.z = pose.orientation.z;
  marker.pose.orientation.w = pose.orientation.w;

  // 根据机器人尺寸设置Marker的缩放
  marker.scale.x = robot_length_;
  marker.scale.y = robot_width_;
  marker.scale.z = 0.1; // 较小的Z轴高度，方便2D可视化

  // 设置Marker的颜色（蓝色，带一点透明度）
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 0.7f; // Alpha (透明度)

  robot_polygon_marker_pub_.publish(marker);
}

void DWAPlanner::initialize(const nav_msgs::OccupancyGrid &costmap, const geometry_msgs::Twist& cur_vel)
{
  costmap_ = costmap;
  current_cmd_vel_ = cur_vel;
  costmap_initialized_ = true;
  create_obs_list(costmap_);
  path_.poses.clear();
}
bool DWAPlanner::makePlan(const geometry_msgs::PoseStamped &start_pose, const geometry_msgs::PoseStamped &goal_pose)
{
  if(!costmap_initialized_){
    ROS_ERROR("mvp dwa Costmap is not initialized. Please call initialize() before makePlan().");
    return false;
  }
  costmap_initialized_ = false;
  geometry_msgs::Twist cmd_vel;
  std::pair<std::vector<State>, bool> best_traj;
  std::vector<std::pair<std::vector<State>, bool>> trajectories;
  const size_t trajectories_size = velocity_samples_ * (steer_angle_samples_ + 1);
  trajectories.reserve(trajectories_size);

  geometry_msgs::PoseStamped goal_in_robot_frame;
  try
  {
    listener_.transformPose(robot_frame_, ros::Time(0), goal_pose, goal_pose.header.frame_id, goal_in_robot_frame);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("goal from local map to robot frame error:%s", ex.what());
    return false;
  }
  const Eigen::Vector3d goal(goal_in_robot_frame.pose.position.x, goal_in_robot_frame.pose.position.y, tf::getYaw(goal_in_robot_frame.pose.orientation));

  const double angle_to_goal = atan2(goal.y(), goal.x());
  if (M_PI / 4.0 < fabs(angle_to_goal))
    use_speed_cost_ = true;
  else{
    use_speed_cost_ = false;
  }

  if (dist_to_goal_th_ < goal.segment(0, 2).norm()) // too far
  {
    best_traj.first = dwa_planning(goal, trajectories);
    cmd_vel.linear.x = best_traj.first.front().velocity_;
    cmd_vel.angular.z = best_traj.first.front().yawrate_;

  }
  else // close enough to goal
  {
    best_traj.first = generate_trajectory(cmd_vel.linear.x, cmd_vel.angular.z, true);
    trajectories.push_back(best_traj);
  }

  visualize_trajectory(best_traj.first, selected_trajectory_pub_); // best trajectory
  visualize_trajectories(trajectories, candidate_trajectories_pub_);
  visualize_footprints(best_traj.first, predict_footprints_pub_);

  // return cmd_vel;
  dwa_cmd_vel_.linear.x = cmd_vel.linear.x;
  dwa_cmd_vel_.angular.z = cmd_vel.angular.z;
  tranform_trajectory_to_path(best_traj.first);
  return true;
}


std::vector<DWAPlanner::State>
DWAPlanner::dwa_planning(const Eigen::Vector3d &goal, std::vector<std::pair<std::vector<State>, bool>> &trajectories)
{
  Cost min_cost(0.0, 0.0, 0.0, 0.0, 1e6);
  const Window dynamic_window = calc_dynamic_window();
  ROS_INFO("goal in robot frame: (%.2f[m], %.2f[m], %.2f[radian])", goal.x(), goal.y(), goal.z());
  std::vector<State> best_traj;
  best_traj.resize(sim_time_samples_); // 每条轨迹的采样点数
  std::vector<Cost> costs;
  const size_t costs_size = velocity_samples_ * (steer_angle_samples_ + 1); // 轨迹的数量
  costs.reserve(costs_size);
  ROS_INFO("Number of sample trajectories: %zu", costs_size);

  const double velocity_resolution =
      std::max((dynamic_window.max_velocity_ - dynamic_window.min_velocity_) / (velocity_samples_ - 1), DBL_EPSILON);
  const double steer_resolution =
      std::max((dynamic_window.max_steer_angle_ - dynamic_window.min_steer_angle_) / (steer_angle_samples_ - 1), DBL_EPSILON);

  int available_traj_count = 0;
  for (int i = 0; i < velocity_samples_; i++)
  {
    const double v = dynamic_window.min_velocity_ + velocity_resolution * i;
    for (int j = 0; j < steer_angle_samples_; j++)
    {
      std::pair<std::vector<State>, bool> traj;
      double y = dynamic_window.min_steer_angle_ + steer_resolution * j;
      // if (v < slow_velocity_th_)
      //   y = y > 0 ? std::max(y, min_yawrate_) : std::min(y, -min_yawrate_);
      traj.first = generate_trajectory(v, y, true);
      const Cost cost = evaluate_trajectory(traj.first, goal);
      costs.push_back(cost);
      if (cost.obs_cost_ == 1e6)
      {
        traj.second = false;
      }
      else
      {
        traj.second = true;
        available_traj_count++;
      }
      trajectories.push_back(traj);
    }

    if (dynamic_window.min_yawrate_ < 0.0 && 0.0 < dynamic_window.max_yawrate_) // strgith forward
    {
      std::pair<std::vector<State>, bool> traj;
      traj.first = generate_trajectory(v, 0.0, true);
      const Cost cost = evaluate_trajectory(traj.first, goal);
      costs.push_back(cost);
      if (cost.obs_cost_ == 1e6)
      {
        traj.second = false;
      }
      else
      {
        traj.second = true;
        available_traj_count++;
      }
      trajectories.push_back(traj);
    }
  }

  if (available_traj_count == 0)
  {
    ROS_ERROR_THROTTLE(1.0, "No available trajectory");
    best_traj = generate_trajectory(0.0, 0.0, true);
  }
  else
  {
    normalize_costs(costs);
    for (int i = 0; i < costs.size(); i++)
    {
      if (costs[i].obs_cost_ != 1e6)
      {
        costs[i].to_goal_cost_ *= to_goal_cost_gain_;
        costs[i].to_goal_orientation_cost_ *= to_goal_orientation_cost_gain_;
        costs[i].obs_cost_ *= obs_cost_gain_;
        costs[i].speed_cost_ *= speed_cost_gain_;
        costs[i].path_cost_ *= 1.0;
        costs[i].calc_total_cost();
        if (costs[i].total_cost_ < min_cost.total_cost_)
        {
          min_cost = costs[i];
          best_traj = trajectories[i].first;
        }
      }
    }
  }

  ROS_INFO("====best trajectory param ===");
  ROS_INFO_STREAM("(v, y) = (" << best_traj.front().velocity_ << ", " << best_traj.front().yawrate_ << ")");
  min_cost.show();
  ROS_INFO_STREAM("num of trajectories available: " << available_traj_count << " of " << trajectories.size());
  ROS_INFO(" ");

  return best_traj;
}

void DWAPlanner::normalize_costs(std::vector<DWAPlanner::Cost> &costs)
{
  Cost min_cost(1e6, 1e6, 1e6, 1e6, 1e6), max_cost;

  for (const auto &cost : costs)
  {
    if (cost.obs_cost_ != 1e6)
    {
      min_cost.obs_cost_ = std::min(min_cost.obs_cost_, cost.obs_cost_);
      max_cost.obs_cost_ = std::max(max_cost.obs_cost_, cost.obs_cost_);
      min_cost.to_goal_cost_ = std::min(min_cost.to_goal_cost_, cost.to_goal_cost_);
      max_cost.to_goal_cost_ = std::max(max_cost.to_goal_cost_, cost.to_goal_cost_);
      min_cost.to_goal_orientation_cost_ = std::min(min_cost.to_goal_orientation_cost_, cost.to_goal_orientation_cost_);
      max_cost.to_goal_orientation_cost_ = std::max(max_cost.to_goal_orientation_cost_, cost.to_goal_orientation_cost_);
      if (use_speed_cost_)
      {
        min_cost.speed_cost_ = std::min(min_cost.speed_cost_, cost.speed_cost_);
        max_cost.speed_cost_ = std::max(max_cost.speed_cost_, cost.speed_cost_);
      }
    }
  }

  for (auto &cost : costs)
  {
    if (cost.obs_cost_ != 1e6)
    {
      cost.obs_cost_ = (cost.obs_cost_ - min_cost.obs_cost_) / (max_cost.obs_cost_ - min_cost.obs_cost_ + DBL_EPSILON);
      cost.to_goal_cost_ = (cost.to_goal_cost_ - min_cost.to_goal_cost_) /
                           (max_cost.to_goal_cost_ - min_cost.to_goal_cost_ + DBL_EPSILON);
      cost.to_goal_orientation_cost_ = (cost.to_goal_orientation_cost_ - min_cost.to_goal_orientation_cost_) /
                           (max_cost.to_goal_orientation_cost_ - min_cost.to_goal_orientation_cost_ + DBL_EPSILON);
      if (use_speed_cost_)
        cost.speed_cost_ =
            (cost.speed_cost_ - min_cost.speed_cost_) / (max_cost.speed_cost_ - min_cost.speed_cost_ + DBL_EPSILON);
    }
  }
}


bool DWAPlanner::check_collision(const std::vector<State> &traj)
{
  for (const auto &state : traj)
  {
    for (const auto &obs : obs_list_.poses)
    {
      const geometry_msgs::PolygonStamped footprint = ackerman_move_footprint(state, true);
      if (is_inside_of_robot(obs.position, footprint, state))
        return true;
    }
  }

  return false;
}

DWAPlanner::Window DWAPlanner::calc_dynamic_window(void)
{
  Window window;
  window.min_velocity_ = std::max((current_cmd_vel_.linear.x - max_deceleration_ * sim_period_), min_velocity_);
  window.max_velocity_ = std::min((current_cmd_vel_.linear.x + max_acceleration_ * sim_period_), target_velocity_);
  // window.min_yawrate_ = std::max((current_cmd_vel_.angular.z - max_d_yawrate_ * sim_period_), -max_yawrate_);
  // window.max_yawrate_ = std::min((current_cmd_vel_.angular.z + max_d_yawrate_ * sim_period_), max_yawrate_);
  window.min_steer_angle_ = -max_steer_angle_;
  window.max_steer_angle_ = max_steer_angle_;

  ROS_INFO("Dynamic Window:");
  ROS_INFO_STREAM("\tcurent_cm_vel:" << current_cmd_vel_.linear.x << ", " << current_cmd_vel_.angular.z);
  ROS_INFO_STREAM("\tVelocity:");
  ROS_INFO_STREAM("\t\tmax: " << window.max_velocity_);
  ROS_INFO_STREAM("\t\tmin: " << window.min_velocity_);
  ROS_INFO_STREAM("\tSteer Angle:");
  ROS_INFO_STREAM("\t\tmax: " << window.max_steer_angle_);
  ROS_INFO_STREAM("\t\tmin: " << window.min_steer_angle_);
  return window;
}

float DWAPlanner::calc_to_goal_cost(const std::vector<State> &traj, const Eigen::Vector3d &goal)
{
  Eigen::Vector3d last_position(traj.back().x_, traj.back().y_, traj.back().yaw_);
  return (last_position.segment(0, 2) - goal.segment(0, 2)).norm();
}

float DWAPlanner::calc_to_goal_orientation_cost(const std::vector<State> &traj, const Eigen::Vector3d &goal)
{
  Eigen::Vector3d last_position(traj.back().x_, traj.back().y_, traj.back().yaw_);
  return (last_position.segment(2, 1) - goal.segment(2, 1)).norm();
}

float DWAPlanner::calc_obs_cost(const std::vector<State> &traj)
{
  float min_dist = obs_range_;
  for (const auto &state : traj)
  {
    for (const auto &obs : obs_list_.poses)
    {
      // ADD:local_map坐标变换到robot坐标系
      geometry_msgs::PoseStamped obs_in_robot_frame;
      geometry_msgs::PoseStamped obs_in_local_map;
      // obs_in_local_map.header = obs_list_.header;
      obs_in_local_map.header.frame_id = obs_list_.header.frame_id;
      obs_in_local_map.header.stamp = ros::Time(0);
      obs_in_local_map.pose.position = obs.position;
      obs_in_local_map.pose.orientation = obs.orientation;
      try
      {
        listener_.transformPose(robot_frame_, obs_in_local_map, obs_in_robot_frame);
      }
      catch (tf::TransformException &ex)
      {
        ROS_WARN_THROTTLE(1.0, "TF transform failed in create_obs_list: %s", ex.what());
      }
      float dist = calc_dist_from_robot(obs_in_robot_frame.pose.position, state, true);
      if (dist < DBL_EPSILON)
        return 1e6;
      min_dist = std::min(min_dist, dist);
    }
  }
  return obs_range_ - min_dist;
}

float DWAPlanner::calc_speed_cost(const std::vector<State> &traj)
{
  if (!use_speed_cost_)
    return 0.0;
  const Window dynamic_window = calc_dynamic_window();
  return dynamic_window.max_velocity_ - traj.front().velocity_;
}

std::vector<DWAPlanner::State> DWAPlanner::generate_trajectory(const double velocity, const double steer_angle, bool use_ackerman)
{
  if (!use_ackerman)
  {
    return std::vector<State>();
  }
  const double yawrate = velocity * tan(steer_angle) / wheelbase_;
  std::vector<State> trajectory;
  trajectory.resize(sim_time_samples_);
  State state;
  for (int i = 0; i < sim_time_samples_; i++)
  {
    motion(state, velocity, yawrate);
    trajectory[i] = state;
  }
  return trajectory;
}


DWAPlanner::Cost DWAPlanner::evaluate_trajectory(const std::vector<State> &trajectory, const Eigen::Vector3d &goal)
{
  Cost cost;
  cost.to_goal_cost_ = calc_to_goal_cost(trajectory, goal);
  cost.to_goal_orientation_cost_ = calc_to_goal_orientation_cost(trajectory, goal);
  cost.obs_cost_ = calc_obs_cost(trajectory);
  cost.speed_cost_ = calc_speed_cost(trajectory);
  cost.path_cost_ = 0.0; // 删除距离参考路径的距离
  cost.calc_total_cost();
  return cost;
}

geometry_msgs::Point DWAPlanner::calc_intersection(
    const geometry_msgs::Point &obstacle, const State &state, geometry_msgs::PolygonStamped footprint)
{
  for (int i = 0; i < footprint.polygon.points.size(); i++)
  {
    const Eigen::Vector3d vector_A(obstacle.x, obstacle.y, 0.0);
    const Eigen::Vector3d vector_B(state.x_, state.y_, 0.0);
    const Eigen::Vector3d vector_C(footprint.polygon.points[i].x, footprint.polygon.points[i].y, 0.0);
    Eigen::Vector3d vector_D(0.0, 0.0, 0.0);
    if (i != footprint.polygon.points.size() - 1)
      vector_D << footprint.polygon.points[i + 1].x, footprint.polygon.points[i + 1].y, 0.0;
    else
      vector_D << footprint.polygon.points[0].x, footprint.polygon.points[0].y, 0.0;

    const double deno = (vector_B - vector_A).cross(vector_D - vector_C).z();
    if (fabs(deno) < 1e-8) // 加入平行或重合判断
      continue;

    const double s = (vector_C - vector_A).cross(vector_D - vector_C).z() / deno;
    const double t = (vector_B - vector_A).cross(vector_A - vector_C).z() / deno;

    geometry_msgs::Point point;
    point.x = vector_A.x() + s * (vector_B - vector_A).x();
    point.y = vector_A.y() + s * (vector_B - vector_A).y();

    // cross
    if (!(s < 0.0 || 1.0 < s || t < 0.0 || 1.0 < t))
      return point;
  }

  geometry_msgs::Point point;
  point.x = 1e6;
  point.y = 1e6;
  return point;
}

float DWAPlanner::calc_dist_from_robot(const geometry_msgs::Point &obstacle, const State &state, bool use_ackerman)
{
  if (!use_ackerman)
  {
    return 0.0f;
  }
  const geometry_msgs::PolygonStamped footprint = ackerman_move_footprint(state, use_ackerman); // pose polygon after moving in the current state coorinate
  if (is_inside_of_robot(obstacle, footprint, state))
  {
    return 0.0;
  }
  else
  {
    geometry_msgs::Point intersection = calc_intersection(obstacle, state, footprint);
    return hypot((obstacle.x - intersection.x), (obstacle.y - intersection.y));
  }
}

geometry_msgs::PolygonStamped DWAPlanner::ackerman_move_footprint(const State &target_pose, bool use_ackerman)
{
  if (!use_ackerman)
  { 
    return geometry_msgs::PolygonStamped();
  }
  geometry_msgs::PolygonStamped footprint;
  geometry_msgs::Point32 front_left, rear_left, rear_right, front_right;
  front_left.x = wheelbase_+ front_overhang_ + footprint_padding_;
  front_left.y = robot_width_ / 2.0 + footprint_padding_;
  rear_left.x = -(rear_overhang_ + footprint_padding_);
  rear_left.y = robot_width_ / 2.0 + footprint_padding_;
  rear_right.x = -(rear_overhang_ + footprint_padding_);
  rear_right.y = -(robot_width_ / 2.0 + footprint_padding_);
  front_right.x = wheelbase_ + front_overhang_ + footprint_padding_;
  front_right.y = -(robot_width_ / 2.0 + footprint_padding_);

  footprint.polygon.points.push_back(front_left);
  footprint.polygon.points.push_back(rear_left);
  footprint.polygon.points.push_back(rear_right);
  footprint.polygon.points.push_back(front_right);
  footprint.header.stamp = ros::Time::now();

  // add rotation
  for (auto &point : footprint.polygon.points)
  {
    Eigen::VectorXf point_in(2);
    point_in << point.x, point.y;
    Eigen::Matrix2f rot;
    rot = Eigen::Rotation2Df(target_pose.yaw_);
    const Eigen::VectorXf point_out = rot * point_in;

    point.x = point_out.x() + target_pose.x_;
    point.y = point_out.y() + target_pose.y_;
  }
  return footprint;
}

bool DWAPlanner::is_inside_of_robot(
    const geometry_msgs::Point &obstacle, const geometry_msgs::PolygonStamped &footprint, const State &state)
{
  geometry_msgs::Point32 state_point;
  state_point.x = state.x_;
  state_point.y = state.y_;

  for (int i = 0; i < footprint.polygon.points.size(); i++)
  {
    geometry_msgs::Polygon triangle;
    triangle.points.push_back(state_point);
    triangle.points.push_back(footprint.polygon.points[i]);

    if (i != footprint.polygon.points.size() - 1)
      triangle.points.push_back(footprint.polygon.points[i + 1]);
    else
      triangle.points.push_back(footprint.polygon.points[0]);

    if (is_inside_of_triangle(obstacle, triangle))
      return true;
  }

  return false;
}

bool DWAPlanner::is_inside_of_triangle(const geometry_msgs::Point &target_point, const geometry_msgs::Polygon &triangle)
{
  if (triangle.points.size() != 3)
  {
    ROS_ERROR("Not triangle");
    exit(1);
  }

  const Eigen::Vector3d vector_A(triangle.points[0].x, triangle.points[0].y, 0.0);
  const Eigen::Vector3d vector_B(triangle.points[1].x, triangle.points[1].y, 0.0);
  const Eigen::Vector3d vector_C(triangle.points[2].x, triangle.points[2].y, 0.0);
  const Eigen::Vector3d vector_P(target_point.x, target_point.y, 0.0);

  const Eigen::Vector3d vector_AB = vector_B - vector_A;
  const Eigen::Vector3d vector_BP = vector_P - vector_B;
  const Eigen::Vector3d cross1 = vector_AB.cross(vector_BP);

  const Eigen::Vector3d vector_BC = vector_C - vector_B;
  const Eigen::Vector3d vector_CP = vector_P - vector_C;
  const Eigen::Vector3d cross2 = vector_BC.cross(vector_CP);

  const Eigen::Vector3d vector_CA = vector_A - vector_C;
  const Eigen::Vector3d vector_AP = vector_P - vector_A;
  const Eigen::Vector3d cross3 = vector_CA.cross(vector_AP);

  if ((0 < cross1.z() && 0 < cross2.z() && 0 < cross3.z()) || (cross1.z() < 0 && cross2.z() < 0 && cross3.z() < 0))
    return true;
  else
    return false;
}

void DWAPlanner::motion(State &state, const double velocity, const double yawrate)
{
  // Dead Reckon(DR) Algorithm
  const double sim_time_step = predict_time_ / static_cast<double>(sim_time_samples_);
  state.yaw_ += yawrate * sim_time_step;
  state.x_ += velocity * std::cos(state.yaw_) * sim_time_step;
  state.y_ += velocity * std::sin(state.yaw_) * sim_time_step;
  state.velocity_ = velocity;
  state.yawrate_ = yawrate;
}

void DWAPlanner::create_obs_list(const nav_msgs::OccupancyGrid &map)
{
  obs_list_.poses.clear();
  obs_list_.header = map.header;
  // ROS_WARN_THROTTLE(1.0, "Creating obstacle list from local map");
  const double max_search_dist = max_velocity_ * predict_time_ * 3; // 3倍预测时间内的最大搜索距离
  for (float angle = -M_PI; angle <= M_PI; angle += angle_resolution_)
  {
    for (float dist = 0.0; dist <= max_search_dist; dist += map.info.resolution)
    {
      geometry_msgs::PoseStamped pose_in_robot_frame;
      pose_in_robot_frame.header.frame_id = robot_frame_;
      pose_in_robot_frame.header.stamp = ros::Time(0);
      pose_in_robot_frame.pose.position.x = dist * cos(angle);
      pose_in_robot_frame.pose.position.y = dist * sin(angle);
      pose_in_robot_frame.pose.orientation.x = 0.0;
      pose_in_robot_frame.pose.orientation.y = 0.0;
      pose_in_robot_frame.pose.orientation.z = 0.0;
      pose_in_robot_frame.pose.orientation.w = 1.0;
      // ADD:坐标变换到local_map坐标系
      geometry_msgs::PoseStamped pose_in_local_map;
      try
      {
        listener_.transformPose(map.header.frame_id, pose_in_robot_frame, pose_in_local_map);
      }
      catch (tf::TransformException &ex)
      {
        ROS_WARN_THROTTLE(1.0, "TF transform failed in create_obs_list: %s", ex.what());
        continue;
      }
      const int index_x = floor((pose_in_local_map.pose.position.x - map.info.origin.position.x) / map.info.resolution);
      const int index_y = floor((pose_in_local_map.pose.position.y - map.info.origin.position.y) / map.info.resolution);
      if ((0 <= index_x && index_x < map.info.width) && (0 <= index_y && index_y < map.info.height))
      {
        if (map.data[index_x + index_y * map.info.width] >= 90) // TODO:BUG, 栅格地图不一定100才是障碍物
        {
          geometry_msgs::Pose pose;
          pose.position.x = pose_in_local_map.pose.position.x;
          pose.position.y = pose_in_local_map.pose.position.y;
          pose.orientation = pose_in_local_map.pose.orientation;
          obs_list_.poses.push_back(pose);
          break;
        }
      }
    }
  }
  // 发布障碍物点可视化
  visualize_obstacles();
}

// 添加新的可视化函数
void DWAPlanner::visualize_obstacles(void)
{
  visualization_msgs::MarkerArray obstacle_markers;
  
  // 首先发送清除标记
  visualization_msgs::Marker clear_marker;
  clear_marker.header.frame_id = obs_list_.header.frame_id;
  clear_marker.header.stamp = ros::Time::now();
  clear_marker.ns = "obstacle_points";
  clear_marker.action = visualization_msgs::Marker::DELETEALL;
  obstacle_markers.markers.push_back(clear_marker);
  
  // 为每个障碍物点创建标记
  for (size_t i = 0; i < obs_list_.poses.size(); i++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = obs_list_.header.frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "obstacle_points";
    marker.id = i + 1; // 从1开始，避免与清除标记冲突
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    
    // 设置障碍物点的位置
    marker.pose.position.x = obs_list_.poses[i].position.x;
    marker.pose.position.y = obs_list_.poses[i].position.y;
    marker.pose.position.z = 0.1; // 稍微抬高以便可视化
    marker.pose.orientation.w = 1.0;
    
    // 设置标记大小
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    
    // 设置颜色为红色
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.8;
    
    // 设置持续时间（长期显示直到下次更新）
    marker.lifetime = ros::Duration(0); // 0表示永久显示直到被删除
    
    obstacle_markers.markers.push_back(marker);
  }
  
  // 发布标记数组
  obstacle_points_pub_.publish(obstacle_markers);
  
  ROS_INFO_THROTTLE(2.0, "Published %lu obstacle points for visualization", obs_list_.poses.size());
}

visualization_msgs::Marker DWAPlanner::create_marker_msg(
    const int id, const double scale, const std_msgs::ColorRGBA color, const std::vector<State> &trajectory,
    const geometry_msgs::PolygonStamped &footprint)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = robot_frame_;
  marker.header.stamp = ros::Time::now();
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1;
  marker.scale.x = scale;
  marker.color = color;
  marker.color.a = 0.8;
  marker.lifetime = ros::Duration(0.0);

  geometry_msgs::Point p;
  if (footprint.polygon.points.empty())
  {
    for (const auto &point : trajectory)
    {
      p.x = point.x_;
      p.y = point.y_;
      marker.points.push_back(p);
    }
  }
  else
  {
    for (const auto &point : footprint.polygon.points)
    {
      p.x = point.x;
      p.y = point.y;
      marker.points.push_back(p);
    }
    p.x = footprint.polygon.points.front().x;
    p.y = footprint.polygon.points.front().y;
    marker.points.push_back(p);
  }

  return marker;
}

void DWAPlanner::tranform_trajectory_to_path(const std::vector<State> &trajectory){
  auto costmp_frame_id = costmap_.header.frame_id;
  path_.poses.clear();
  path_.header.stamp = ros::Time::now();
  path_.header.frame_id = costmp_frame_id;
  geometry_msgs::PoseStamped pose_in, pose_out;
  pose_in.header.stamp = ros::Time::now();
  pose_in.header.frame_id = robot_frame_;
  for (int i = 0; i < trajectory.size(); i++)
  {
    pose_in.pose.position.x = trajectory[i].x_;
    pose_in.pose.position.y = trajectory[i].y_;
    pose_in.pose.position.z = 0.2; // for visualization
    pose_in.pose.orientation = tf::createQuaternionMsgFromYaw(trajectory[i].yaw_);
    try
    {
      listener_.transformPose(costmp_frame_id, ros::Time(0), pose_in, robot_frame_, pose_out);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("trajectory to path failed: %s", ex.what());
      return;
    }
    path_.poses.push_back(pose_out);
  }
}

void DWAPlanner::visualize_trajectory(const std::vector<State> &trajectory, const ros::Publisher &pub)
{
  std_msgs::ColorRGBA color;
  color.r = 1.0;
  visualization_msgs::Marker v_trajectory = create_marker_msg(0, v_path_width_, color, trajectory);
  pub.publish(v_trajectory);
}

void DWAPlanner::visualize_trajectories(
    const std::vector<std::pair<std::vector<State>, bool>> &trajectories, const ros::Publisher &pub)
{
  visualization_msgs::MarkerArray v_trajectories;
  for (int i = 0; i < trajectories.size(); i++)
  {
    std_msgs::ColorRGBA color;
    if (trajectories[i].second)
    {
      color.g = 1.0;
    }
    else
    {
      color.r = 0.5;
      color.b = 0.5;
    }
    visualization_msgs::Marker v_trajectory = create_marker_msg(i, v_path_width_ * 0.4, color, trajectories[i].first);
    v_trajectories.markers.push_back(v_trajectory);
  }
  pub.publish(v_trajectories);
}

void DWAPlanner::visualize_footprints(const std::vector<State> &trajectory, const ros::Publisher &pub)
{
  std_msgs::ColorRGBA color;
  color.b = 1.0;
  visualization_msgs::MarkerArray v_footprints;
  for (int i = 0; i < trajectory.size(); i++)
  {
    const geometry_msgs::PolygonStamped footprint = ackerman_move_footprint(trajectory[i], true);
    visualization_msgs::Marker v_footprint = create_marker_msg(i, v_path_width_ * 0.2, color, trajectory, footprint);
    v_footprints.markers.push_back(v_footprint);
  }
  pub.publish(v_footprints);
}
