#include "object/xarm_object.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("xarm_add_attached_object");
  RCLCPP_INFO(node->get_logger(),"xarm_add_attached_object start");

  auto xarmobject = std::make_shared<xarm_object::XArmObject>(node);

  const std::string parent_link = "link_base";
  const std::string object_name = "desk";
  const std::string resource = "package://add_object/meshes/desk.stl";
  std::vector<std::string> touch_links = std::vector<std::string>{"link_base"};
  geometry_msgs::msg::Pose pos;
  pos.position.x = 0.0;
  pos.position.y = 0.0;
  pos.position.z = 0.0;
  pos.orientation.x = 0.0;
  pos.orientation.y = 0.0;
  pos.orientation.z = 0.0;
  pos.orientation.w = 0.0;

  const Eigen::Vector3d scaling(1, 1, 1);

  xarmobject->XArmAddAttachedObject(parent_link, object_name, resource, touch_links, pos);
  rclcpp::shutdown();
  return 0;
}