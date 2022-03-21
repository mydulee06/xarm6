#include <string>
#include <signal.h>
#include <rclcpp/rclcpp.hpp>
#include <xarm_api/xarm_ros_client.h>
#include <httplib.h>
#include <rapidjson/rapidjson.h>
#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>

#define SERVICE_CALL_FAILED 999
#define SERVER_IP "118.67.143.184"
#define SERVER_PORT 8000
//#define SERVER_IP "192.168.0.8"
//#define SERVER_PORT 1234

class Register
{
public:
    Register(rclcpp::Node::SharedPtr& node){
        node_ = node;
        node_->get_parameter_or("is_new_company", is_new_company_, false);
        node_->get_parameter_or("is_new_factory", is_new_factory_, false);
        node_->get_parameter_or("company_name", company_, std::string("N/A"));
        node_->get_parameter_or("factory_name", factory_, std::string("N/A"));
        node_->get_parameter_or("FactoryLoc", factory_loc_, std::string("N/A"));
        node_->get_parameter_or("robot_type", robot_type_, std::string("N/A"));
        node_->get_parameter_or("loc_x", loc_x_, float(0.0));
        node_->get_parameter_or("loc_y", loc_y_, float(0.0));
        RCLCPP_INFO(node->get_logger(), "\nCompany name: %s\nFactory name: %s\nFacotry loc: %s\nRobot type: %s\nLoc x: %f\nLoc y: %f"
                , company_.c_str(),factory_.c_str(),factory_loc_.c_str(),robot_type_.c_str(),loc_x_,loc_y_);

        cli_ = std::make_shared<httplib::Client>(SERVER_IP, SERVER_PORT);

        if(is_new_company_) register_company();
        if(is_new_factory_) register_factory();

        xarm_state_sub_ = node->create_subscription<xarm_msgs::msg::RobotMsg>("xarm_states", 5, std::bind(&Register::register_robot, this, std::placeholders::_1));
    }
    
    void register_company(){
        char cp_buf[company_.size()];
        int cp_len = sprintf(cp_buf,"%s",company_.c_str());

        rapidjson::Value cp;
        cp.SetString(cp_buf, cp_len, d.GetAllocator());

        rapidjson::Value body(rapidjson::kObjectType);
        body.AddMember("company_name", cp, d.GetAllocator());

        rapidjson::StringBuffer body_str;
	    rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(body_str);
	    body.Accept(writer);

        httplib::Params company_param{
            { "company_name", company_ }
        };
        if(auto res = cli_->Post("/register/company", body_str.GetString(), "application/json")){
            RCLCPP_INFO(node_->get_logger(), "Status: %d\nBody: %s", res->status, res->body.c_str());
        } else{
            RCLCPP_ERROR(node_->get_logger(), "Error code: %d", res.error());
        }
    }

    void register_factory(){
        char cp_buf[company_.size()];
        char fact_buf[factory_.size()];
        char fact_loc_buf[factory_loc_.size()];

        int cp_len = sprintf(cp_buf,"%s",company_.c_str());
        int fact_len = sprintf(fact_buf,"%s",factory_.c_str());
        int fact_loc_len = sprintf(fact_loc_buf,"%s",factory_loc_.c_str());

        rapidjson::Value data;
        rapidjson::Value body(rapidjson::kObjectType);

        data.SetString(cp_buf, cp_len, d.GetAllocator());
        body.AddMember("company_name", data, d.GetAllocator());

        data.SetString(fact_buf, fact_len, d.GetAllocator());
        body.AddMember("factory_name", data, d.GetAllocator());

        data.SetString(fact_loc_buf, fact_loc_len, d.GetAllocator());
        body.AddMember("FactoryLoc", data, d.GetAllocator());

        rapidjson::StringBuffer body_str;
	    rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(body_str);
	    body.Accept(writer);

        if(auto res = cli_->Post("/register/factory", body_str.GetString(), "application/json")){
            RCLCPP_INFO(node_->get_logger(), "Status: %d\nBody: %s", res->status, res->body.c_str());
        } else{
            RCLCPP_ERROR(node_->get_logger(), "Error code: %d", res.error());
        }
    }
    
    void register_robot(const xarm_msgs::msg::RobotMsg::SharedPtr states){
        char cp_buf[company_.size()];
        char fact_buf[factory_.size()];
        char rb_type[robot_type_.size()];
        char rb_ip[states->robot_ip.size()];
        float loc_x = loc_x_;
        float loc_y = loc_y_;

        int cp_len = sprintf(cp_buf,"%s",company_.c_str());
        int fact_len = sprintf(fact_buf,"%s",factory_.c_str());
        int rb_type_len = sprintf(rb_type,"%s",robot_type_.c_str());
        int rb_ip_len = sprintf(rb_ip,"%s",states->robot_ip.c_str());

        rapidjson::Value data;
        rapidjson::Value body(rapidjson::kObjectType);

        data.SetString(cp_buf, cp_len, d.GetAllocator());
        body.AddMember("company_name", data, d.GetAllocator());

        data.SetString(fact_buf, fact_len, d.GetAllocator());
        body.AddMember("factory_name", data, d.GetAllocator());

        data.SetString(rb_type, rb_type_len, d.GetAllocator());
        body.AddMember("robot_type", data, d.GetAllocator());

        data.SetString(rb_ip, rb_ip_len, d.GetAllocator());
        body.AddMember("robot_ip", data, d.GetAllocator());

        data.SetFloat(loc_x);
        body.AddMember("loc_x", data, d.GetAllocator());

        data.SetFloat(loc_y);
        body.AddMember("loc_x", data, d.GetAllocator());

        rapidjson::StringBuffer body_str;
	    rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(body_str);
	    body.Accept(writer);

        if(auto res = cli_->Post("/register/robot", body_str.GetString(), "application/json")){
            RCLCPP_INFO(node_->get_logger(), "Status: %d\nBody: %s", res->status, res->body.c_str());
        } else{
            RCLCPP_ERROR(node_->get_logger(), "Error code: %d", res.error());
        }
        rclcpp::shutdown();
    }
    
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<xarm_msgs::msg::RobotMsg>::SharedPtr xarm_state_sub_;
    bool is_new_company_;
    bool is_new_factory_;
    std::string company_;
    std::string factory_;
    std::string factory_loc_;
    std::string robot_type_;
    float loc_x_;
    float loc_y_;
    std::shared_ptr<httplib::Client> cli_;
    rapidjson::Document d;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    std::string hw_ns = "xarm";
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("xarm_register", node_options);
    RCLCPP_INFO(node->get_logger(), "xarm_register start");

    Register xarm_register(node);

    rclcpp::spin(node);
    rclcpp::shutdown();
    RCLCPP_INFO(node->get_logger(), "xarm_http_server over");
    return 0;
}