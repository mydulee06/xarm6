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
#include <xarm_msgs/msg/robot_msg.hpp>
#include <xarm_msgs/srv/plan_pose.hpp>
#include <xarm_msgs/srv/plan_joint.hpp>
#include <xarm_msgs/srv/plan_exec.hpp>
#include <xarm_msgs/srv/plan_single_straight.hpp>
#include <xarm_msgs/srv/set_int16.hpp>

#include <rapidjson/document.h>
#include <rapidjson/filewritestream.h>
#include <rapidjson/filereadstream.h>
#include <rapidjson/prettywriter.h>
#include <cstdio>

#include <boost/tokenizer.hpp>
#include <math.h>
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

        void go_home();

        void start_manual_mode();
        void get_eef_pose(const int delay);
        void rm_last_states();
        void stop_manual_mode();
        void plan_and_write(std::string &directory);
        bool replay_recorded_path(std::string &directory);
        bool replay_recorded_path_camera(std::string &directory);

        void xarm_states_callback(const xarm_msgs::msg::RobotMsg::SharedPtr states);

        int callService(int data_);
    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<xarm_msgs::msg::RobotMsg>::SharedPtr xarm_state_sub_;
        rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedPtr client_send_msg_to_camera_;
        rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedPtr client_set_mode_;
        rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedPtr client_set_state_;
        std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req_set_int16_;
        std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res_set_int16_;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
        moveit::planning_interface::MoveGroupInterface::Plan xarm_plan_;

        geometry_msgs::msg::Pose home_;
        rapidjson::Document d;
        rapidjson::Value method_arr_;
        rapidjson::Value method_obj_;
        std::string srv_msg_;
        
        template<typename ServiceT, typename SharedRequest = typename ServiceT::Request::SharedPtr>
	    int _call_request(std::shared_ptr<ServiceT> client, SharedRequest req);
    };
}

#endif // __XARM_PLANNER_H__
