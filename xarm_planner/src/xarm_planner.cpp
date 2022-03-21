/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Vinman <vinman.cub@gmail.com>
 ============================================================================*/
 
#include "xarm_planner/xarm_planner.h"

#define SERVICE_CALL_FAILED 999

using namespace std::chrono_literals;

const float PI = 3.141593;

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
    home_.position.x = 0.4;
	home_.position.y = 0;
	home_.position.z = 0.415;
	home_.orientation.x = 1;
	home_.orientation.y = 0;
	home_.orientation.z = 0;
	home_.orientation.w = 0;
    xarm_state_sub_ = node_->create_subscription<xarm_msgs::msg::RobotMsg>("/xarm/xarm_states", 5, std::bind(&XArmPlanner::xarm_states_callback, this, std::placeholders::_1));
    client_send_msg_to_camera_ = node_->create_client<xarm_msgs::srv::SetInt16>("/xarm_camera_socket/send_msg_to_camera");
    client_set_mode_ = node_->create_client<xarm_msgs::srv::SetInt16>("/xarm/set_mode");
    client_set_state_ = node_->create_client<xarm_msgs::srv::SetInt16>("/xarm/set_state");
    
    method_arr_.SetArray();
    method_obj_.SetObject();

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
void XArmPlanner::go_home()
{
    planPoseTarget(home_);
    executePath();
}
void XArmPlanner::start_manual_mode()
{
    req_set_int16_ = std::make_shared<xarm_msgs::srv::SetInt16::Request>();
    req_set_int16_->data = 2;
    _call_request(client_set_mode_,req_set_int16_);
    req_set_int16_->data = 0;
    _call_request(client_set_state_,req_set_int16_);
}
void XArmPlanner::get_eef_pose(const int delay) // 단위 : millisec
{
    xarm_msgs::msg::RobotMsg xarm_state;
    rclcpp::MessageInfo xarm_state_info;
    xarm_state_sub_->take(xarm_state,xarm_state_info);
    
    rapidjson::Value val(rapidjson::kArrayType);
    rapidjson::Value key(rapidjson::kObjectType);

    for (int i = 0; i < 6; i++) val.PushBack(rapidjson::Value().SetDouble(xarm_state.pose[i]), d.GetAllocator());
    
    key.AddMember("pose", val, d.GetAllocator());
    key.AddMember("delay", rapidjson::Value().SetInt(delay), d.GetAllocator());

    method_arr_.PushBack(key, d.GetAllocator());
}

void XArmPlanner::rm_last_states()
{
    method_arr_.PopBack();
}

void XArmPlanner::stop_manual_mode()
{
    req_set_int16_ = std::make_shared<xarm_msgs::srv::SetInt16::Request>();
    req_set_int16_->data = 1;
    _call_request(client_set_mode_,req_set_int16_);
    req_set_int16_->data = 0;
    _call_request(client_set_state_,req_set_int16_);
}

void XArmPlanner::plan_and_write(std::string &method)
{
    go_home();
    tf2::Quaternion temp_quat;
    geometry_msgs::msg::Pose temp_pose;
    for(int i = 0; i < method_arr_.Size(); i++){
        temp_quat.setRPY(method_arr_[i]["pose"][3].GetDouble(), method_arr_[i]["pose"][4].GetDouble(), method_arr_[i]["pose"][5].GetDouble());
        temp_pose.position.x = method_arr_[i]["pose"][0].GetDouble()/1000;
        temp_pose.position.y = method_arr_[i]["pose"][1].GetDouble()/1000;
        temp_pose.position.z = method_arr_[i]["pose"][2].GetDouble()/1000;
        temp_pose.orientation.x = temp_quat[0];
        temp_pose.orientation.y = temp_quat[1];
        temp_pose.orientation.z = temp_quat[2];
        temp_pose.orientation.w = temp_quat[3];
        std::vector<geometry_msgs::msg::Pose> temp_poses = {temp_pose};

        planCartesianPath(temp_poses);
        executePath();
        rclcpp::sleep_for(std::chrono::milliseconds(method_arr_[i]["delay"].GetInt()));
    }
    rapidjson::Value key(method.c_str(), d.GetAllocator());
    method_obj_.AddMember(key, method_arr_, d.GetAllocator());

    method += ".json";
    FILE* fp = fopen(method.c_str(), "wb");

    char writeBuffer[65536];
    rapidjson::FileWriteStream os(fp, writeBuffer, sizeof(writeBuffer));
 
    rapidjson::PrettyWriter<rapidjson::FileWriteStream> fwriter(os);
    method_obj_.Accept(fwriter);

    fclose(fp);
}

bool XArmPlanner::replay_recorded_path(std::string &method)
{
    std::string path = method + ".json";

    FILE* fp;
    if((fp = fopen(path.c_str(), "rb")) == NULL) return false;

    char readBuffer[65536];
    rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    d.ParseStream(is);

    go_home();
    for(int i = 0; i < d[method.c_str()].Size(); i++){
        geometry_msgs::msg::Pose temp_pose;
        tf2::Quaternion pose_q;
        pose_q.setRPY(d[method.c_str()][i]["pose"][3].GetDouble(), d[method.c_str()][i]["pose"][4].GetDouble(), d[method.c_str()][i]["pose"][5].GetDouble());
        temp_pose.position.x = d[method.c_str()][i]["pose"][0].GetDouble()/1000;
        temp_pose.position.y = d[method.c_str()][i]["pose"][1].GetDouble()/1000;
        temp_pose.position.z = d[method.c_str()][i]["pose"][2].GetDouble()/1000;
        temp_pose.orientation.x = pose_q[0];
        temp_pose.orientation.y = pose_q[1];
        temp_pose.orientation.z = pose_q[2];
        temp_pose.orientation.w = pose_q[3];
        std::vector<geometry_msgs::msg::Pose> temp_poses = {temp_pose};
        if(!planCartesianPath(temp_poses)) return false;
        if(!executePath()) return false;
        rclcpp::sleep_for(std::chrono::milliseconds(d[method.c_str()][i]["delay"].GetInt()));
    }
    return true;
}

bool XArmPlanner::replay_recorded_path_camera(std::string &method)
{
    /*
    if(callService(1) != 1) return false;
    go_home();
    //req_set_int16_ = std::make_shared<xarm_msgs::srv::SetInt16::Request>();
    //req_set_int16_->data = 2;
    //_call_request(client_send_msg_to_camera_,req_set_int16_, req_set_int16_);
    //RCLCPP_INFO(node_->get_logger(), "Message from camera : %s", req_set_int16_->message.c_str());
    if(callService(2) != 1) return false;
    float tf[3];
    boost::char_separator<char> sep(",");
    boost::tokenizer<boost::char_separator<char>> tokens(srv_msg_, sep);
    for (const auto& t : tokens) {
        int i = 0;
        tf[i] = std::stof(t);
        i++;
    }
    RCLCPP_INFO(node_->get_logger(), "x : %f, y : %f, theta : %f",tf[0],tf[1],tf[2]);
    tf2::Quaternion rot_q;
    rot_q.setRPY(0,0,tf[2]);
    initReader(method);
    while(loadPose()){
        geometry_msgs::msg::Pose tf_pose;
        tf2::Quaternion pose_q(loaded_pose_.orientation.x,loaded_pose_.orientation.y,loaded_pose_.orientation.z,loaded_pose_.orientation.w);
        pose_q = rot_q*pose_q;
        pose_q.normalize();
        tf_pose.position.x = home_.position.x + cos(tf[2])*(loaded_pose_.position.x - home_.position.x) - sin(tf[2])*(loaded_pose_.position.y - home_.position.y) + tf[0];
        tf_pose.position.y = home_.position.y + sin(tf[2])*(loaded_pose_.position.x - home_.position.x) + cos(tf[2])*(loaded_pose_.position.y - home_.position.y) + tf[1];
        tf_pose.position.z = loaded_pose_.position.z;
        tf_pose.orientation.x = pose_q[0];
        tf_pose.orientation.y = pose_q[1];
        tf_pose.orientation.z = pose_q[2];
        tf_pose.orientation.w = pose_q[3];
        std::vector<geometry_msgs::msg::Pose> temp_poses = {tf_pose};
        if(!planCartesianPath(temp_poses)) {
            callService(0);
            return false;
        }
        if(!executePath()){
            callService(0);
            return false;
        }
        if(callService(3) != 1) return false;
    }
    if(callService(4) != 1) return false;
    return true;
    */
}

void XArmPlanner::xarm_states_callback(const xarm_msgs::msg::RobotMsg::SharedPtr states){}

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
    if (res->message.size() != 0){
        RCLCPP_DEBUG(node_->get_logger(), "call service %s, ret=%d, message(%s)", client->get_service_name(), res->ret, res->message.c_str());
        srv_msg_ = res->message;
    }
    else
        RCLCPP_DEBUG(node_->get_logger(), "call service %s, ret=%d", client->get_service_name(), res->ret);
    return res->ret;
}
}
