#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <dwa_planner/dwa_planner.h>

class DWANode {
public:
    DWANode() : nh_("~"), dwa_planner_() {
        // 订阅costmap话题
        costmap_sub_ = nh_.subscribe("/local_map", 1, &DWANode::costmapCallback, this);
        // 订阅RViz中的目标点
        goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &DWANode::goalCallback, this);
        // 订阅当前速度
        odom_sub_ = nh_.subscribe("/odom", 1, &DWANode::odomCallback, this);
        // 发布路径
        path_pub_ = nh_.advertise<nav_msgs::Path>("/dwa_path", 1);
        // 发布控制指令
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

        // 初始化标志
        is_costmap_received_ = false;
        is_initialized_ = false;
        ROS_INFO("DWA Node initialized");
    }

private:
    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        costmap_ = *msg;
        //costmap_.header.frame_id = "map"; // 确保costmap的frame_id正确
        is_costmap_received_ = true;
        //ROS_INFO("Received costmap");
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_velocity_ = msg->twist.twist;
        latest_odom_ = *msg;
        if (is_costmap_received_ && !is_initialized_) {
            dwa_planner_.initialize(costmap_, current_velocity_);
            is_initialized_ = true;
            ROS_INFO("DWA Planner initialized");
        }
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal) {
        if (!is_initialized_) {
            ROS_WARN("DWA Planner not initialized yet, cannot plan path");
            return;
        }
        ROS_INFO("Received goal msg.");
        // 获取机器人当前位置（假设从/odom话题获取）
        geometry_msgs::PoseStamped start_pose;
        start_pose.header.frame_id = "map";
        start_pose.header.stamp = ros::Time::now();
        // 假设机器人位置从odom获取，实际需根据具体情况调整
        start_pose.pose = latest_odom_.pose.pose;

        // 调用DWA规划器
        dwa_planner_.initialize(costmap_, current_velocity_);
        bool success = dwa_planner_.makePlan(start_pose, *goal);
        if (success) {
            // 获取规划路径并发布
            nav_msgs::Path path = dwa_planner_.getPath();
            if (path.poses.empty()) {
                ROS_WARN("Received empty path from DWA planner");
                return;
            }
            path.header.stamp = ros::Time::now();
            if (path.header.frame_id.empty()) {
                ROS_WARN("Path header.frame_id is empty, setting to 'map'");
                path.header.frame_id = "map";
            }
            path_pub_.publish(path);

            // 获取控制指令并发布
            // geometry_msgs::Twist cmd_vel = dwa_planner_.getDWA_cmd_vel();
            // cmd_vel_pub_.publish(cmd_vel);
            // ROS_WARN("Path planned and cmd_vel published");
        } else {
            ROS_WARN("Failed to plan path");
        }
    }

    ros::NodeHandle nh_;
    ros::Subscriber costmap_sub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher path_pub_;
    ros::Publisher cmd_vel_pub_;
    nav_msgs::OccupancyGrid costmap_;
    geometry_msgs::Twist current_velocity_;
    nav_msgs::Odometry latest_odom_;
    DWAPlanner dwa_planner_;
    bool is_costmap_received_;
    bool is_initialized_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "dwa_node");
    DWANode dwa_node;
    ros::spin();
    return 0;
}