#include "object/xarm_object.h"

namespace xarm_object
{
XArmObject::XArmObject(rclcpp::Node::SharedPtr& node)
  : node_(node)
{
  planning_scene_diff_publisher_ = node_->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);
  while (planning_scene_diff_publisher_->get_subscription_count() < 1)
  {
    rclcpp::sleep_for(std::chrono::milliseconds(500)); // 여기까지 했음!!!
  }
}

void XArmObject::XArmAddCollisionObject(const std::string &object_name, const std::string &resource, geometry_msgs::msg::Pose collision_object_pose)
{
  static const Eigen::Vector3d default_scaling(1.0, 1.0, 1.0);
  XArmAddCollisionObject(object_name, resource, collision_object_pose, default_scaling);
}

void XArmObject::XArmAddCollisionObject(const std::string &object_name, const std::string &resource, geometry_msgs::msg::Pose collision_object_pose, const Eigen::Vector3d &scaling)
{
  RCLCPP_INFO(node_->get_logger(), "Mesh file of collision object is being imported from %s", resource.c_str());
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.id = object_name;
  collision_object.header.frame_id = "world";

  shapes::Mesh *m = shapes::createMeshFromResource(resource, scaling);
  shape_msgs::msg::Mesh collision_object_mesh;
  shapes::ShapeMsg collision_object_mesh_msg;

  shapes::constructMsgFromShape(m, collision_object_mesh_msg);
  collision_object_mesh = boost::get<shape_msgs::msg::Mesh>(collision_object_mesh_msg);

  collision_object.meshes.push_back(collision_object_mesh);
  collision_object.mesh_poses.push_back(collision_object_pose);
  collision_object.operation = collision_object.ADD;
  
  moveit_msgs::msg::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(collision_object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher_->publish(planning_scene);

  RCLCPP_INFO(node_->get_logger(), "충돌 물체(ID : %s)가 로봇 세계에 추가되었습니다.", collision_object.id.c_str());
  RCLCPP_INFO(node_->get_logger(), "Position x = %f", collision_object_pose.position.x);
  RCLCPP_INFO(node_->get_logger(), "Position y = %f", collision_object_pose.position.y);
  RCLCPP_INFO(node_->get_logger(), "Position z = %f", collision_object_pose.position.z);
  RCLCPP_INFO(node_->get_logger(), "Orientation x = %f", collision_object_pose.orientation.x);
  RCLCPP_INFO(node_->get_logger(), "Orientation y = %f", collision_object_pose.orientation.y);
  RCLCPP_INFO(node_->get_logger(), "Orientation z = %f", collision_object_pose.orientation.z);
  RCLCPP_INFO(node_->get_logger(), "Orientation w = %f", collision_object_pose.orientation.w);
}

void XArmObject::XArmAddAttachedObject(const std::string &parent_link, const std::string &object_name, const std::string &resource, std::vector<std::string> &touch_links, geometry_msgs::msg::Pose attached_object_pose){
  static const Eigen::Vector3d default_scaling(1.0, 1.0, 1.0);
  XArmAddAttachedObject(parent_link, object_name, resource, touch_links, attached_object_pose, default_scaling);
}

void XArmObject::XArmAddAttachedObject(const std::string &parent_link, const std::string &object_name, const std::string &resource, std::vector<std::string> &touch_links, geometry_msgs::msg::Pose attached_object_pose, const Eigen::Vector3d &scaling){
  RCLCPP_INFO(node_->get_logger(), "Mesh file of collision object is being imported from %s", resource.c_str());
  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = parent_link;
  attached_object.object.header.frame_id = parent_link;
  attached_object.object.id = object_name;

  shapes::Mesh *m = shapes::createMeshFromResource(resource, scaling);
  shape_msgs::msg::Mesh attached_object_mesh;
  shapes::ShapeMsg attached_object_mesh_msg;

  shapes::constructMsgFromShape(m, attached_object_mesh_msg);
  attached_object_mesh = boost::get<shape_msgs::msg::Mesh>(attached_object_mesh_msg);

  attached_object.object.meshes.push_back(attached_object_mesh);
  attached_object.object.mesh_poses.push_back(attached_object_pose);
  attached_object.object.operation = attached_object.object.ADD;

  attached_object.touch_links = touch_links;

  moveit_msgs::msg::PlanningScene planning_scene;
  planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher_->publish(planning_scene);

  RCLCPP_INFO(node_->get_logger(), "로봇의 end-effector에 %s가 추가되었습니다.", attached_object.object.id.c_str());
  RCLCPP_INFO(node_->get_logger(), "%s로부터의 Position x = %f", parent_link.c_str(), attached_object_pose.position.x);
  RCLCPP_INFO(node_->get_logger(), "%s로부터의 Position y = %f", parent_link.c_str(), attached_object_pose.position.y);
  RCLCPP_INFO(node_->get_logger(), "%s로부터의 Position z = %f", parent_link.c_str(), attached_object_pose.position.z);
  RCLCPP_INFO(node_->get_logger(), "%s로부터의 Orientation x = %f", parent_link.c_str(), attached_object_pose.orientation.x);
  RCLCPP_INFO(node_->get_logger(), "%s로부터의 Orientation y = %f", parent_link.c_str(), attached_object_pose.orientation.y);
  RCLCPP_INFO(node_->get_logger(), "%s로부터의 Orientation z = %f", parent_link.c_str(), attached_object_pose.orientation.z);
  RCLCPP_INFO(node_->get_logger(), "%s로부터의 Orientation w = %f", parent_link.c_str(), attached_object_pose.orientation.w);
}

void XArmObject::XArmRemoveCollisionObject(std::string object_id){
  moveit_msgs::msg::CollisionObject object;
  object.id = object_id;
  object.operation = object.REMOVE;
  
  moveit_msgs::msg::PlanningScene planning_scene;
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(object);
  planning_scene_diff_publisher_->publish(planning_scene);
  RCLCPP_INFO(node_->get_logger(), "충돌 물체(ID : %s)가 로봇 세계에서 제거되었습니다.", object.id.c_str());
}
void XArmObject::XArmRemoveAttachedObject(std::string &object_id){
  moveit_msgs::msg::AttachedCollisionObject detach_object;
  detach_object.object.id = object_id;
  detach_object.link_name = "link6";
  detach_object.object.operation = detach_object.object.REMOVE;

  moveit_msgs::msg::PlanningScene planning_scene;
  planning_scene.robot_state.attached_collision_objects.push_back(detach_object);
  planning_scene_diff_publisher_->publish(planning_scene);
}
void XArmObject::XArmAllowCollision(const std::string &link1, const std::string &link2){
  client_get_planning_scene_ = node_->create_client<moveit_msgs::srv::GetPlanningScene>("get_planning_scene");
  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  request->components.components = request->components.ALLOWED_COLLISION_MATRIX;
    
  bool is_try_again = false;
  while (!client_get_planning_scene_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
      exit(1);
    }
    if (!is_try_again) {
      is_try_again = true;
      RCLCPP_WARN(node_->get_logger(), "service %s not available, waiting ...", client_get_planning_scene_->get_service_name());
    }
  }
  auto result_future = client_get_planning_scene_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, result_future) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to call service %s", client_get_planning_scene_->get_service_name());
    return;
  }
  auto res = result_future.get();
  
  moveit_msgs::msg::PlanningScene planning_scene;
  collision_detection::AllowedCollisionMatrix acm(res->scene.allowed_collision_matrix);
  acm.setEntry(link1, link2, true);
  acm.getMessage(planning_scene.allowed_collision_matrix);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher_->publish(planning_scene);
}
}