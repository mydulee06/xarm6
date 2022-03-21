/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Vinman <vinman.cub@gmail.com>
 ============================================================================*/

#ifndef __XARM_PLANNER_H__
#define __XARM_PLANNER_H__

#include <signal.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rcpputils/filesystem_helper.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
// #include <moveit_visual_tools/moveit_visual_tools.h>

#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <xarm_msgs/srv/plan_pose.hpp>
#include <xarm_msgs/srv/plan_joint.hpp>
#include <xarm_msgs/srv/plan_exec.hpp>
#include <xarm_msgs/srv/plan_single_straight.hpp>
#include <xarm_msgs/srv/set_int16.hpp>

#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>

#include <chrono>
#include <memory>


namespace xarm_planner
{
    class XArmPlanner
    {
    public:
        XArmPlanner(const rclcpp::Node::SharedPtr& node, const std::string& group_name);
        ~XArmPlanner() {};

        void setConstraints();
        bool planJointTarget(const std::vector<double>& joint_target);
        bool planPoseTarget(const geometry_msgs::msg::Pose& pose_target);
        bool planPoseTargets(const std::vector<geometry_msgs::msg::Pose>& pose_target_vector);
        bool planCartesianPath(const std::vector<geometry_msgs::msg::Pose>& pose_target_vector);

        bool executePath(bool wait = true);

        void start_manual_mode();
        void stop_manual_mode();
        void get_joint_states();
        void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr states);
        void rm_last_states();
        void go_home();
        void plan_and_write(std::string &directory);
        bool replay_recorded_path(std::string &directory);
        bool replay_recorded_path_camera(std::string &directory);
        void initWriter(std::string &directory);
        void savePath();
        void initReader(std::string &directory);
        bool loadPath();
        int callService(int data_);
    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
        rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedPtr client_send_msg_to_camera_;
        rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedPtr client_set_mode_;
        rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedPtr client_set_state_;
        std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req_set_int16_;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
        moveit::planning_interface::MoveGroupInterface::Plan xarm_plan_;
        template<typename ServiceT, typename SharedRequest = typename ServiceT::Request::SharedPtr>
	    int _call_request(std::shared_ptr<ServiceT> client, SharedRequest req);
        std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> writer_;
        rosbag2_cpp::readers::SequentialReader reader_;
        std::vector<sensor_msgs::msg::JointState> recorded_joint_state_;
    };
}

#endif // __XARM_PLANNER_H__
