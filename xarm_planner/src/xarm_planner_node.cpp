/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Vinman <vinman.cub@gmail.com>
 ============================================================================*/

#include <signal.h>
#include <rclcpp/rclcpp.hpp>
#include "xarm_planner/xarm_planner.h"
#include <xarm_api/xarm_ros_client.h>
#include <std_msgs/msg/bool.hpp>
#include <xarm_msgs/srv/plan_pose.hpp>
#include <xarm_msgs/srv/plan_joint.hpp>
#include <xarm_msgs/srv/plan_exec.hpp>
#include <xarm_msgs/srv/plan_single_straight.hpp>
#include <xarm_msgs/srv/set_int16_str.hpp>
#include <xarm_msgs/msg/cio_state.hpp>

#define BIND_CLS_CB(func) std::bind(func, this, std::placeholders::_1, std::placeholders::_2)

class XArmPlannerRunner
{
public:
    XArmPlannerRunner(rclcpp::Node::SharedPtr& node);
    ~XArmPlannerRunner() {};

private:
    void exec_seq(const xarm_msgs::msg::CIOState::SharedPtr iostate);
    bool do_pose_plan(const std::shared_ptr<xarm_msgs::srv::PlanPose::Request> req, std::shared_ptr<xarm_msgs::srv::PlanPose::Response> res);
    bool do_joint_plan(const std::shared_ptr<xarm_msgs::srv::PlanJoint::Request> req, std::shared_ptr<xarm_msgs::srv::PlanJoint::Response> res);
    bool exec_plan_cb(const std::shared_ptr<xarm_msgs::srv::PlanExec::Request> req, std::shared_ptr<xarm_msgs::srv::PlanExec::Response> res);
    bool record_callback(const std::shared_ptr<xarm_msgs::srv::SetInt16Str::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16Str::Response> res);

private:
    std::string group_name_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Node::SharedPtr client_node_;
    std::shared_ptr<xarm_planner::XArmPlanner> xarm_planner_;
    std::shared_ptr<xarm_planner::XArmPlanner> xarm_client_planner_;
    std::string method_;
    int L_iostate_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr exec_plan_sub_;
    rclcpp::Subscription<xarm_msgs::msg::CIOState>::SharedPtr io_state_sub_;

    rclcpp::Service<xarm_msgs::srv::PlanExec>::SharedPtr exec_plan_server_;
    rclcpp::Service<xarm_msgs::srv::PlanPose>::SharedPtr pose_plan_server_;
    rclcpp::Service<xarm_msgs::srv::PlanJoint>::SharedPtr joint_plan_server_;
    rclcpp::Service<xarm_msgs::srv::PlanSingleStraight>::SharedPtr single_straight_plan_server_;
    rclcpp::Service<xarm_msgs::srv::SetInt16Str>::SharedPtr record_target_point_;
};

XArmPlannerRunner::XArmPlannerRunner(rclcpp::Node::SharedPtr& node)
    : node_(node)
{
    int dof;
    node_->get_parameter_or("DOF", dof, 7);
    group_name_ = "xarm" + std::to_string(dof);

    RCLCPP_INFO(node_->get_logger(), "namespace: %s, group_name: %s", node->get_namespace(), group_name_.c_str());

    xarm_planner_ = std::make_shared<xarm_planner::XArmPlanner>(node_, group_name_);

    io_state_sub_ = node_->create_subscription<xarm_msgs::msg::CIOState>("/xarm/xarm_cgpio_states", 5, std::bind(&XArmPlannerRunner::exec_seq, this, std::placeholders::_1));

    exec_plan_server_ = node_->create_service<xarm_msgs::srv::PlanExec>("xarm_exec_plan", BIND_CLS_CB(&XArmPlannerRunner::exec_plan_cb));
    pose_plan_server_ = node_->create_service<xarm_msgs::srv::PlanPose>("xarm_pose_plan", BIND_CLS_CB(&XArmPlannerRunner::do_pose_plan));
    joint_plan_server_ = node_->create_service<xarm_msgs::srv::PlanJoint>("xarm_joint_plan", BIND_CLS_CB(&XArmPlannerRunner::do_joint_plan));
    record_target_point_ = node_->create_service<xarm_msgs::srv::SetInt16Str>("/xarm/xarm_record", BIND_CLS_CB(&XArmPlannerRunner::record_callback));

    client_node_ = rclcpp::Node::make_shared("xarm_planner_client_node");
}

void XArmPlannerRunner::exec_seq(const xarm_msgs::msg::CIOState::SharedPtr iostate)
{
    int iostate_buffer = iostate->input_digitals[1];
    if(iostate_buffer == 254 && L_iostate_ == 255){
        rclcpp::Node::SharedPtr xarm_node = rclcpp::Node::make_shared("xarm_client_node");;
        xarm_api::XArmROSClient xarm_client;

        xarm_client.init(xarm_node, std::string("xarm"));
        xarm_client.set_cgpio_digital(1, 0);
        rclcpp::sleep_for(std::chrono::milliseconds(100));

        auto xarm_record_executor = std::make_shared<xarm_planner::XArmPlanner>(client_node_, group_name_);
        //xarm_record_executor->replay_recorded_path_camera(method_);
        xarm_record_executor->replay_recorded_path(method_);
        xarm_record_executor.reset();

        xarm_client.set_cgpio_digital(1, 1);
    }
    L_iostate_ = iostate_buffer;
}

bool XArmPlannerRunner::do_pose_plan(const std::shared_ptr<xarm_msgs::srv::PlanPose::Request> req, std::shared_ptr<xarm_msgs::srv::PlanPose::Response> res)
{
    bool success = xarm_planner_->planPoseTarget(req->target);
    res->success = success;
    return success;
}

bool XArmPlannerRunner::do_joint_plan(const std::shared_ptr<xarm_msgs::srv::PlanJoint::Request> req, std::shared_ptr<xarm_msgs::srv::PlanJoint::Response> res)
{
    bool success = xarm_planner_->planJointTarget(req->target);
    res->success = success;
    return success;
}

bool XArmPlannerRunner::exec_plan_cb(const std::shared_ptr<xarm_msgs::srv::PlanExec::Request> req, std::shared_ptr<xarm_msgs::srv::PlanExec::Response> res)
{
    bool success = xarm_planner_->executePath(req->wait);
    res->success = success;
    return success;
}

/**
 * Service "/xarm/xarm_record"의 콜백함수.
 * 매뉴얼 모드로 로봇 메소드 생성, 변경 또는 동작하게 해줌. 
 * 보통 http_server랑 같이 쓰여서 서버에서 들어오는 요청 수행함.
 * req->data의 값에 따라 xarm_planner::XArmPlanner 객체의 함수들 불러옴. xarm_palnner.cpp이랑 같이 보면 이해하기 쉬움.
 * req->data =
 * 1 : Teaching Mode(매뉴얼 모드) 시작, 2 : 현재 상태 저장, 3 : 마지막으로 저장된 상태 삭제, 4 : 매뉴얼 모드 종료
 * 5 : 경로계획, 6 : Teaching Mode(매뉴얼 모드) 리셋, 7 : 메소드 변경, 8 : 카메라와 동작, 9 : 카메라 없이 동작
 * 성공하면 1, 실패하면 0 return.
 */
bool XArmPlannerRunner::record_callback(const std::shared_ptr<xarm_msgs::srv::SetInt16Str::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16Str::Response> res)
{
    res->ret = 0;
    if(req->data == 1){
        xarm_client_planner_ = std::make_shared<xarm_planner::XArmPlanner>(client_node_, group_name_);
        method_ = req->parameter; // method: 메소드 이름. ".json" 붙여서 파일 불러옴
        RCLCPP_INFO(node_->get_logger(), "Manual_mode start");
        xarm_client_planner_->start_manual_mode();
        res->ret = 1;
    } else if(req->data == 7){
        method_ = req->parameter;
        res->ret = 1;
    } else if(req->data == 8){
        auto xarm_record_executor = std::make_shared<xarm_planner::XArmPlanner>(client_node_, group_name_);
        if(xarm_record_executor->replay_recorded_path_camera(method_)) res->ret = 1;
        else res->ret = 0;
        xarm_record_executor.reset();
    } else if(req->data == 9){
        auto xarm_record_executor = std::make_shared<xarm_planner::XArmPlanner>(client_node_, group_name_);
        if(xarm_record_executor->replay_recorded_path(method_)) res->ret = 1;
        else res->ret = 0;
        xarm_record_executor.reset();
    } else {
        if(!xarm_client_planner_) {
            res->ret = 0;
            return res->ret == 1;
        }
        switch(req->data){
            case 2: {
                if(req->parameter.empty()) xarm_client_planner_->get_eef_pose(0);
                else{
                    int delay = std::stoi(req->parameter);
                    xarm_client_planner_->get_eef_pose(delay);
                }
                res->ret = 1;
                break;
            }
            case 3: {
                xarm_client_planner_->rm_last_states();
                res->ret = 1;
                break;
            }
            case 4: {
                xarm_client_planner_->stop_manual_mode();
                res->ret = 1;
                break;
            }
            case 5: {
                xarm_client_planner_->plan_and_write(method_);
                res->ret = 1;
                break;
            }
            case 6: {
                xarm_client_planner_.reset();
                res->ret = 1;
                break;
            }
        }
    }
    return res->ret == 1;
}

void exit_sig_handler(int signum)
{
    fprintf(stderr, "[xarm_planner_node] Ctrl-C caught, exit process...\n");
    exit(-1);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("xarm_planner_node", node_options);
    RCLCPP_INFO(node->get_logger(), "xarm_planner_node start");
    signal(SIGINT, exit_sig_handler);

    XArmPlannerRunner xarm_planner_runner(node);
    
    rclcpp::spin(node);
    rclcpp::shutdown();

    RCLCPP_INFO(node->get_logger(), "xarm_planner_node over");
    return 0;
}