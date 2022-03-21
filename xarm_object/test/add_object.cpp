#include "object/xarm_object.h"

void exit_sig_handler(int signum)
{
    fprintf(stderr, "[xarm_driver] Ctrl-C caught, exit process...\n");
    exit(-1);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("xarm_add_object", node_options);
  RCLCPP_INFO(node->get_logger(),"xarm_add_object start");
  
  signal(SIGINT, exit_sig_handler);

  std::string object_name = node->get_parameter("object_name").as_string();
  std::string mesh_file = node->get_parameter("mesh_file").as_string();
  std::vector<double> origin_xyz_rpy = node->get_parameter("origin_xyz_rpy").as_double_array();
  std::vector<double> scaling = node->get_parameter("scaling").as_double_array();

  auto xarmobject = std::make_shared<xarm_object::XArmObject>(node);

  tf2::Quaternion quat;
  quat.setRPY(origin_xyz_rpy[3],origin_xyz_rpy[4],origin_xyz_rpy[5]);
  geometry_msgs::msg::Pose pos;
  pos.position.x = origin_xyz_rpy[0];
  pos.position.y = origin_xyz_rpy[1];
  pos.position.z = origin_xyz_rpy[2];
  pos.orientation.x = 0.0;
  pos.orientation.y = 0.0;
  pos.orientation.z = 0.0;
  pos.orientation.w = 0.0;

  const Eigen::Vector3d scaling_(scaling[0], scaling[1], scaling[2]);
  xarmobject->XArmAddCollisionObject(object_name, mesh_file, pos, scaling_);
  xarmobject->XArmAllowCollision(object_name, "link_base");
  /*
  const std::string object_name_1 = "bottole";
  const std::string resource_1 = "package://add_object/meshes/desk.stl";

  geometry_msgs::msg::Pose pos_1;
  pos.position.x = 0.0;
  pos.position.y = 0.0;
  pos.position.z = 0.0;
  pos.orientation.x = 0.0;
  pos.orientation.y = 0.0;
  pos.orientation.z = 0.0;
  pos.orientation.w = 0.0;

  const Eigen::Vector3d scaling_1(0.1, 0.1, 0.1);

  xarmobject.XArmAddCollisionObject(object_name_1, resource_1, pos_1, scaling_1);
  */
  rclcpp::shutdown();
  return 0;
}