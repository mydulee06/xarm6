#ifndef __XARM_OBJECT_H__
#define __XARM_OBJECT_H__

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>

// MoveIt
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h> ///
#include <moveit/planning_scene_monitor/planning_scene_monitor.h> ///

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>

#include <string>

namespace xarm_object
{
    class XArmObject
    {
        public:
            XArmObject(rclcpp::Node::SharedPtr& node);
            //void init(rclcpp::Node::SharedPtr& node);
            void XArmAddCollisionObject(const std::string &object_name, const std::string &resource, geometry_msgs::msg::Pose collision_object_pose);
            void XArmAddCollisionObject(const std::string &object_name, const std::string &resource, geometry_msgs::msg::Pose collision_object_pose, const Eigen::Vector3d &scaling);
            void XArmAddAttachedObject(const std::string &parent_link, const std::string &object_name, const std::string &resource, std::vector<std::string> &touch_links, geometry_msgs::msg::Pose attached_object_pose);
            void XArmAddAttachedObject(const std::string &parent_link, const std::string &object_name, const std::string &resource, std::vector<std::string> &touch_links, geometry_msgs::msg::Pose attached_object_pose, const Eigen::Vector3d &scaling);
            void XArmRemoveCollisionObject(std::string object_id);
            void XArmRemoveAttachedObject(std::string &object_id);
            void XArmAllowCollision(const std::string &link1, const std::string &link2);
            rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr client_get_planning_scene_;

        private:
            rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher_;
            std::shared_ptr<planning_scene::PlanningScene> planning_scene_;
            rclcpp::Node::SharedPtr node_;
    };
}

#endif // __XARM_OBJECT_H__