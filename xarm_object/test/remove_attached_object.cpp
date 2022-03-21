#include "object/xarm_object.h"

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("xarm_remove_attached_object");
  RCLCPP_INFO(node->get_logger(),"xarm_remove_attached_object start");

  auto xarmobject = std::make_shared<xarm_object::XArmObject>(node);

  xarmobject->XArmRemoveAttachedObject("camera");

  rclcpp::shutdown();
  return 0;
}