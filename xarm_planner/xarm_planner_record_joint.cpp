/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Vinman <vinman.cub@gmail.com>
 ============================================================================*/
 
#include "xarm_planner/xarm_planner.h"

#define SERVICE_CALL_FAILED 999

using namespace std::chrono_literals;

namespace xarm_planner
{
const double jump_threshold = 0.0;
const double eef_step = 0.005;
const double maxV_scale_factor = 0.5;
const double maxA_scale_factor = 0.5;

XArmPlanner::XArmPlanner(const rclcpp::Node::SharedPtr& node, const std::string& group_name)
    : node_(node)
{
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, group_name);
    setConstraints();
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>("joint_states", 5, std::bind(&XArmPlanner::joint_states_callback, this, std::placeholders::_1));
    client_send_msg_to_camera_ = node_->create_client<xarm_msgs::srv::SetInt16>("/xarm_camera_socket/send_msg_to_camera");
    client_set_mode_ = node_->create_client<xarm_msgs::srv::SetInt16>("/xarm/set_mode");
    client_set_state_ = node_->create_client<xarm_msgs::srv::SetInt16>("/xarm/set_state");
    // rclcpp::executors::SingleThreadedExecutor executor;
    // executor.add_node(node_);
    // std::thread([&executor]() { executor.spin(); }).detach();
    RCLCPP_INFO(node_->get_logger(), "Planning frame: %s", move_group_->getPlanningFrame().c_str());
    RCLCPP_INFO(node_->get_logger(), "End effector link: %s", move_group_->getEndEffectorLink().c_str());
    RCLCPP_INFO(node_->get_logger(), "Available Planning Groups:");
    std::copy(move_group_->getJointModelGroupNames().begin(), move_group_->getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
}

void XArmPlanner::setConstraints()
{
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions.resize(3);
    box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 1;
    box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 1;
    box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 1;

    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = 0.4;
    box_pose.position.y = 0;
    box_pose.position.z = 0.1;
    box_pose.orientation.w = 1.0;

    moveit_msgs::msg::PositionConstraint position_constraint;
    position_constraint.header.frame_id = "link_base";
    position_constraint.link_name = "link_tcp";
    position_constraint.constraint_region.primitives.push_back(box);
    position_constraint.constraint_region.primitive_poses.push_back(box_pose);

    position_constraint.weight = 1.0;
    moveit_msgs::msg::Constraints path_constraints;
    path_constraints.name = "test_position_constraints";
    path_constraints.position_constraints.push_back(position_constraint);
    move_group_->setPathConstraints(path_constraints);
}

bool XArmPlanner::planJointTarget(const std::vector<double>& joint_target)
{
    move_group_->setMaxVelocityScalingFactor(maxV_scale_factor);
    move_group_->setMaxAccelerationScalingFactor(maxA_scale_factor);
    bool success = move_group_->setJointValueTarget(joint_target);
    if (!success)
        RCLCPP_WARN(node_->get_logger(), "setJointValueTarget: out of bounds");
    success = (move_group_->plan(xarm_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success)
        RCLCPP_ERROR(node_->get_logger(), "planJointTarget: plan failed");
    return success;
}

bool XArmPlanner::planPoseTarget(const geometry_msgs::msg::Pose& pose_target)
{
    move_group_->setMaxVelocityScalingFactor(maxV_scale_factor);
    move_group_->setMaxAccelerationScalingFactor(maxA_scale_factor);
    bool success = move_group_->setPoseTarget(pose_target);
    if (!success)
        RCLCPP_WARN(node_->get_logger(), "setPoseTarget: out of bounds");
    success = (move_group_->plan(xarm_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success)
        RCLCPP_ERROR(node_->get_logger(), "planPoseTarget: plan failed");
    return success;
}

bool XArmPlanner::planPoseTargets(const std::vector<geometry_msgs::msg::Pose>& pose_target_vector)
{
    move_group_->setMaxVelocityScalingFactor(maxV_scale_factor);
    move_group_->setMaxAccelerationScalingFactor(maxA_scale_factor);
    bool success = move_group_->setPoseTargets(pose_target_vector);
    if (!success)
        RCLCPP_WARN(node_->get_logger(), "setPoseTargets: out of bounds");
    success = (move_group_->plan(xarm_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success)
        RCLCPP_ERROR(node_->get_logger(), "planPoseTargets: plan failed");
    return success;
}

bool XArmPlanner::planCartesianPath(const std::vector<geometry_msgs::msg::Pose>& pose_target_vector)
{   
    move_group_->setMaxVelocityScalingFactor(maxV_scale_factor);
    move_group_->setMaxAccelerationScalingFactor(maxA_scale_factor);
    moveit_msgs::msg::RobotTrajectory trajectory;
    
    double fraction = move_group_->computeCartesianPath(pose_target_vector, eef_step, jump_threshold, trajectory);
    bool success = true;
    if(fraction < 0.9) {
        RCLCPP_ERROR(node_->get_logger(), "planCartesianPath: plan failed, fraction=%lf", fraction);
        return false;
    }
    xarm_plan_.trajectory_ = trajectory;
    return true;
}

bool XArmPlanner::executePath(bool wait)
{
    moveit::planning_interface::MoveItErrorCode code;
    if (wait)
        code =  move_group_->execute(xarm_plan_);
    else
        code =  move_group_->asyncExecute(xarm_plan_);
    bool success = (code == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success)
        RCLCPP_ERROR(node_->get_logger(), "executePath: execute failed, wait=%d, MoveItErrorCode=%d", wait, code);
    return success;
}
void XArmPlanner::start_manual_mode()
{
    req_set_int16_ = std::make_shared<xarm_msgs::srv::SetInt16::Request>();
    req_set_int16_->data = 2;
    _call_request(client_set_mode_,req_set_int16_);
    req_set_int16_->data = 0;
    _call_request(client_set_state_,req_set_int16_);
}
void XArmPlanner::stop_manual_mode()
{
    req_set_int16_ = std::make_shared<xarm_msgs::srv::SetInt16::Request>();
    req_set_int16_->data = 1;
    _call_request(client_set_mode_,req_set_int16_);
    req_set_int16_->data = 0;
    _call_request(client_set_state_,req_set_int16_);
}
void XArmPlanner::get_joint_states()
{
    sensor_msgs::msg::JointState joint_state;
    rclcpp::MessageInfo joint_state_info;
    joint_state_sub_->take(joint_state,joint_state_info);
    recorded_joint_state_.push_back(joint_state);
    /*
    std::string pos_str = "[ ";
    for(int i = 0; i < joint_state.position.size(); i++){
        pos_str += std::to_string(joint_state.position[i]); 
        pos_str += " ";
    }
    pos_str += "]";
    RCLCPP_INFO(node_->get_logger(), "Recorded positon: %s", pos_str.c_str());
    RCLCPP_INFO(node_->get_logger(), "Size of recorded_joint_state: %d", recorded_joint_state_.size());
    */
}
void XArmPlanner::joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr states){}
void XArmPlanner::rm_last_states()
{
    recorded_joint_state_.erase(recorded_joint_state_.end());
}
void XArmPlanner::go_home()
{
    geometry_msgs::msg::Pose home;
    home.position.x = 0.4;
	home.position.y = 0;
	home.position.z = 0.1;
	home.orientation.x = 1;
	home.orientation.y = 0;
	home.orientation.z = 0;
	home.orientation.w = 0;
    planPoseTarget(home);
    executePath();
}
void XArmPlanner::plan_and_write(std::string &directory)
{
    go_home();
    initWriter(directory);
    for(int i = 0; i < recorded_joint_state_.size(); i++){
        std::vector<double> joint_target;
        for(int j = 0; j < recorded_joint_state_[i].position.size(); j++){
            joint_target.push_back(recorded_joint_state_[i].position[j]);
        }
        planJointTarget(joint_target);
        executePath();
        savePath();
    }
}
bool XArmPlanner::replay_recorded_path(std::string &directory)
{
    go_home();
    initReader(directory);
    while(loadPath()){
        if(!executePath()) return false;
    }
    return true;
}
bool XArmPlanner::replay_recorded_path_camera(std::string &directory)
{
    if(callService(1) != 1) return false;
    go_home();
    initReader(directory);
    while(loadPath()){
        if(!executePath()) {
            callService(0);
            return false;
        }
        if(callService(2) != 1) return false;
    }
    return true;
}
void XArmPlanner::initWriter(std::string &directory)
{
    auto rosbag_directory = rcpputils::fs::path(directory.c_str());
    rcpputils::fs::remove_all(rosbag_directory);
    if(!rcpputils::fs::is_directory(rosbag_directory)) rcpputils::fs::create_directories(rosbag_directory);
    const rosbag2_cpp::StorageOptions storage_options({directory.c_str(), "sqlite3"});
    const rosbag2_cpp::ConverterOptions converter_options(
        {rmw_get_serialization_format(),
        rmw_get_serialization_format()});
    writer_ = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();

    writer_->open(storage_options, converter_options);

    writer_->create_topic(
        {"savepath",
        "moveit_msgs/msg/RobotTrajectory",
        rmw_get_serialization_format(),
        ""});
}
void XArmPlanner::savePath()
{
    rcutils_time_point_value_t time_stamp;
    if (rcutils_system_time_now(&time_stamp) != RCUTILS_RET_OK) {
        std::cerr << "Error getting current time: " <<
            rcutils_get_error_string().str;
        exit(1);
    }

    auto serializer = rclcpp::Serialization<moveit_msgs::msg::RobotTrajectory>();
    auto serialized_message = rclcpp::SerializedMessage();
    serializer.serialize_message(&xarm_plan_.trajectory_, &serialized_message);

    auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();

    bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
        new rcutils_uint8_array_t,
        [](rcutils_uint8_array_t * msg) {
            auto fini_return = rcutils_uint8_array_fini(msg);
            delete msg;
            if (fini_return != RCUTILS_RET_OK) {
                std::cerr << "Failed to destroy serialized message " <<
                rcutils_get_error_string().str;
            }
        });
    *bag_message->serialized_data = serialized_message.release_rcl_serialized_message();

    bag_message->topic_name = "savepath";
    bag_message->time_stamp = time_stamp;

    writer_->write(bag_message);
}
void XArmPlanner::initReader(std::string &directory)
{
    rosbag2_cpp::StorageOptions storage_options{};
    storage_options.uri = directory.c_str();
    storage_options.storage_id = "sqlite3";

    rosbag2_cpp::ConverterOptions converter_options{};
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    reader_.open(storage_options, converter_options);
}
bool XArmPlanner::loadPath()
{
    rclcpp::Serialization<moveit_msgs::msg::RobotTrajectory> serialization;

    if(reader_.has_next()){
        auto bag_msg = reader_.read_next();
        rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
        serialization.deserialize_message(&serialized_msg, &xarm_plan_.trajectory_);
        return true;
    }
    return false;
}
int XArmPlanner::callService(int data_)
{
    req_set_int16_ = std::make_shared<xarm_msgs::srv::SetInt16::Request>();
    req_set_int16_->data = data_;
    return _call_request(client_send_msg_to_camera_,req_set_int16_);
}
template<typename ServiceT, typename SharedRequest = typename ServiceT::Request::SharedPtr>
int XArmPlanner::_call_request(std::shared_ptr<ServiceT> client, SharedRequest req)
{
    bool is_try_again = false;
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
            exit(1);
        }
        if (!is_try_again) {
            is_try_again = true;
            RCLCPP_WARN(node_->get_logger(), "service %s not available, waiting ...", client->get_service_name());
        }
    }
    auto result_future = client->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node_, result_future) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service %s", client->get_service_name());
        return SERVICE_CALL_FAILED;
    }
    auto res = result_future.get();
    if (res->message.size() != 0)
        RCLCPP_DEBUG(node_->get_logger(), "call service %s, ret=%d, message(%s)", client->get_service_name(), res->ret, res->message.c_str());
    else
        RCLCPP_DEBUG(node_->get_logger(), "call service %s, ret=%d", client->get_service_name(), res->ret);
    return res->ret;
}
}
