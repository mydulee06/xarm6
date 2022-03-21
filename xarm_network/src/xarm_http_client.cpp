#include <signal.h>
#include <rclcpp/rclcpp.hpp>
#include <xarm_api/xarm_ros_client.h>
#include <httplib.h>
#include <rapidjson/rapidjson.h>
#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>

#define SERVICE_CALL_FAILED 999
#define SERVER_IP "101.101.218.67"
#define SERVER_PORT 8000

class Http_Cli
{
public:
    Http_Cli(rclcpp::Node::SharedPtr& node)
    {
        node_ = node;
        xarm_state_sub_ = node_->create_subscription<xarm_msgs::msg::RobotMsg>("/xarm/xarm_states", 5, std::bind(&Http_Cli::xarm_states_callback, this, std::placeholders::_1));
        cli_ = std::make_shared<httplib::Client>(SERVER_IP, SERVER_PORT);
        if (auto res = cli_->Get("/hi")){
            RCLCPP_INFO(node_->get_logger(),"Status : %d",res->status);
            RCLCPP_INFO(node_->get_logger(),"Body : %s",res->body.c_str());
        } else{
            RCLCPP_INFO(node_->get_logger(),"Error code : %d",res.error());
        }
    }

private:
    void xarm_states_callback(const xarm_msgs::msg::RobotMsg::SharedPtr states)
    {
        char serial[states->serial_number.size()];
        int serial_len = sprintf(serial,"%s",states->serial_number.c_str());
        
        int state = states->state;
        int err = states->err;

        if(state_buf_ != state){
            int att = 0; // 시도 횟수;
            rapidjson::Value data;
            rapidjson::Value body(rapidjson::kObjectType);

            data.SetString(serial, serial_len, d.GetAllocator());
            body.AddMember("serial", data, d.GetAllocator());

            data.SetInt(state);
            body.AddMember("value", data, d.GetAllocator());

            rapidjson::StringBuffer body_str;
	        rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(body_str);
	        body.Accept(writer);
            // 확인하기
            auto res = cli_->Post("/robot/update_data/status", body_str.GetString(), "application/json");

            while(!res && att < 100){
                res = cli_->Post("/robot/update_data/status", body_str.GetString(), "application/json");
                att++;
            }
            if(!res){
                RCLCPP_INFO(node_->get_logger(),"Error code : %d",res.error());
            }
            if(att != 100) state_buf_ = state;
        }

        if(err_buf_ != err){ 
            int att = 0; // 시도 횟수;
            rapidjson::Value data;
            rapidjson::Value body(rapidjson::kObjectType);

            data.SetString(serial, serial_len, d.GetAllocator());
            body.AddMember("serial", data, d.GetAllocator());

            data.SetInt(err);
            body.AddMember("value", data, d.GetAllocator());

            rapidjson::StringBuffer body_str;
	        rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(body_str);
	        body.Accept(writer);

            auto res = cli_->Post("/robot/update_data/error_code", body_str.GetString(), "application/json");

            while(!res && att < 100){
                res = cli_->Post("/robot/update_data/error_code", body_str.GetString(), "application/json");
                att++;
            }
            if(!res){
                RCLCPP_INFO(node_->get_logger(),"Error code : %d",res.error());
            }
            if(att != 100) err_buf_ = err;
        }
    }
    template<typename ServiceT, typename SharedRequest = typename ServiceT::Request::SharedPtr>
    int _call_request(std::shared_ptr<ServiceT> client, SharedRequest req)
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
        }
        else
            RCLCPP_DEBUG(node_->get_logger(), "call service %s, ret=%d", client->get_service_name(), res->ret);
        return res->ret;
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<xarm_msgs::msg::RobotMsg>::SharedPtr xarm_state_sub_;
    std::shared_ptr<httplib::Client> cli_;
    int state_buf_ = -1;
    int err_buf_ = -1;
    rapidjson::Document d;
};

void exit_sig_handler(int signum)
{
    fprintf(stderr, "[xarm_http_server] Ctrl-C caught, exit process...\n");
    exit(-1);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::string hw_ns = "xarm";
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("xarm_http_client");
    RCLCPP_INFO(node->get_logger(), "xarm_http_server start");

    signal(SIGINT, exit_sig_handler);

    Http_Cli http_cli(node);
    
    rclcpp::spin(node);
    rclcpp::shutdown();

    RCLCPP_INFO(rclcpp::get_logger("xarm_http_server"), "xarm_http_server over");
}