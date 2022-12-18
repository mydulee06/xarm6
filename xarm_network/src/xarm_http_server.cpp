#include <signal.h>
#include <rclcpp/rclcpp.hpp>
#include <xarm_api/xarm_ros_client.h>
#include <string.h>
#include <rapidjson/document.h>
#include <rapidjson/filewritestream.h>
#include <rapidjson/filereadstream.h>
#include <rapidjson/prettywriter.h>
#include <httplib.h>
#include <unistd.h>
#include <iostream>
#include <fstream>

#define SERVICE_CALL_FAILED 999

#define CLOUD_IP "101.101.218.67"
#define CLOUD_PORT 8000

#define SERVER_IP "0.0.0.0"
#define SERVER_PORT 8080

void exit_sig_handler(int signum)
{
    fprintf(stderr, "[xarm_http_server] Ctrl-C caught, exit process...\n");
    exit(-1);
}

std::string dump_headers(const httplib::Headers &headers) {
    std::string s;
    char buf[BUFSIZ];

    for (auto it = headers.begin(); it != headers.end(); ++it) {
        const auto &x = *it;
        snprintf(buf, sizeof(buf), "%s: %s\n", x.first.c_str(), x.second.c_str());
        s += buf;
    }

    return s;
}

std::string log(const httplib::Request &req, const httplib::Response &res) {
    std::string s;
    char buf[BUFSIZ];

    s += "================================\n";

    snprintf(buf, sizeof(buf), "%s %s %s", req.method.c_str(),
           req.version.c_str(), req.path.c_str());
    s += buf;

    std::string query;
    for (auto it = req.params.begin(); it != req.params.end(); ++it) {
        const auto &x = *it;
        snprintf(buf, sizeof(buf), "%c%s=%s",
                 (it == req.params.begin()) ? '?' : '&', x.first.c_str(),
                 x.second.c_str());
        query += buf;
    }
    snprintf(buf, sizeof(buf), "%s\n", query.c_str());
    s += buf;

    s += dump_headers(req.headers);

    s += req.body.c_str();

    s += '\n';

    s += "--------------------------------\n";

    snprintf(buf, sizeof(buf), "%d %s\n", res.status, res.version.c_str());
    s += buf;
    s += dump_headers(res.headers);
    s += "\n";

    if (!res.body.empty()) { s += res.body; }

    s += "\n";

    return s;
}

void empty_cb(const xarm_msgs::msg::RobotMsg::SharedPtr states){}

/**
 * 서버에서 들어오는 http 요청에 따라 처리
 * 자세한 내용은 api 참조
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::string hw_ns = "xarm";
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("xarm_http_server");
    
    rclcpp::Subscription<xarm_msgs::msg::RobotMsg>::SharedPtr xarm_state_sub 
        = node->create_subscription<xarm_msgs::msg::RobotMsg>("/xarm/xarm_states", 5, empty_cb);
    

    xarm_msgs::msg::RobotMsg xarm_state;
    rclcpp::MessageInfo xarm_state_info;
    std::string temp_method;
    
    int att = 0;
    while(!xarm_state_sub->take(xarm_state,xarm_state_info) && att < 100){
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        att++;
    }
    if(att == 100){
        RCLCPP_ERROR(node->get_logger(), "Cannot obtain XArm states");
        rclcpp::shutdown();
    }
    std::string robot_serial = xarm_state.serial_number;
    RCLCPP_INFO(node->get_logger(), "Serial number: %s",robot_serial.c_str());

    signal(SIGINT, exit_sig_handler);

    //XArm_Http_Server http_server(node, hw_ns);

    //ROS_Cli ros_cli(node);
    xarm_api::XArmROSClient xarm_client;
    xarm_client.init(node, hw_ns);
    httplib::Server svr;
    rapidjson::Document doc;

    if (!svr.is_valid()) {
        printf("server has an error...\n");
        return -1;
    }

    svr.Get("/hi", [](const httplib::Request & req, httplib::Response &res) {
        res.set_content("Hello World!\n", "text/plain");
    });

    /**
     * 로봇 시리얼 얻는 http api 구현 함수
     */
    svr.Get("/get_serial", [&](const httplib::Request &req, httplib::Response &res){
        res.set_content(robot_serial, "text/plain");
    });

    /**
     * 로봇에 명령어 보내는 api 구현 함수
     * 로봇 serial, command, param이 있는지 확인하고 있으면 시리얼이 이 로봇과 맞는지 확인 후에 명령 실행
     */
    svr.Post("/", [&](const httplib::Request &req, httplib::Response &res){
        //ros_cli.callRecSrv(1, std::string("sequence1"));
        rapidjson::ParseResult Pres = doc.Parse(req.body.c_str());
        if(!Pres){
            res.status = 404;
            res.set_content("{\n  \"msg\": \"Not Json format\"\n}", "application/json");
            return 0;
        }
        if(!doc.HasMember("robot_serial") || !doc.HasMember("robot_command") || !doc.HasMember("robot_param")){
            res.status = 404;
            res.set_content("{\n  \"msg\": \"Not appropriate API\"\n}", "application/json");
            return 0;
        }
        if(strcmp(doc["robot_serial"].GetString(), robot_serial.c_str()) == 0){ // 시리얼 정보 확인
            if(strcmp(doc["robot_command"].GetString(), "manual_start") == 0){
                RCLCPP_INFO(node->get_logger(), "Manual start command received.");
                temp_method = std::string(doc["robot_param"].GetString());
                if(xarm_client.xarm_record(1, temp_method))
                    res.status = 200; 
                else{
                    res.status = 404;
                    res.set_content("{\n  \"msg\": \"Robot internal Error.\"\n}", "application/json");
                }
            } else if(strcmp(doc["robot_command"].GetString(), "get_pos") == 0){
                RCLCPP_INFO(node->get_logger(), "Get position command received.");
                if(xarm_client.xarm_record(2, std::string(doc["robot_param"].GetString())))
                    res.status = 200;
                else{
                    res.status = 404;
                    res.set_content("{\n  \"msg\": \"Robot internal Error.\"\n}", "application/json");
                }
            } else if(strcmp(doc["robot_command"].GetString(), "remove_pos") == 0){
                RCLCPP_INFO(node->get_logger(), "Remove last position command received.");
                if(xarm_client.xarm_record(3))
                    res.status = 200;
                else{
                    res.status = 404;
                    res.set_content("{\n  \"msg\": \"Robot internal Error.\"\n}", "application/json");
                }
            } else if(strcmp(doc["robot_command"].GetString(), "manual_stop") == 0){
                RCLCPP_INFO(node->get_logger(), "Manual stop command received.");
                if(xarm_client.xarm_record(4))
                    res.status = 200;
                else{
                    res.status = 404;
                    res.set_content("{\n  \"msg\": \"Robot internal Error.\"\n}", "application/json");
                }
            } else if(strcmp(doc["robot_command"].GetString(), "plan_save") == 0){
                RCLCPP_INFO(node->get_logger(), "Plan & save command received.");
                if(xarm_client.xarm_record(5))
                    res.status = 200;
                else{
                    res.status = 404;
                    res.set_content("{\n  \"msg\": \"Robot internal Error.\"\n}", "application/json");
                }
            } else if(strcmp(doc["robot_command"].GetString(), "manual_end") == 0){
                RCLCPP_INFO(node->get_logger(), "Manual end command received.");
                
                std::string path = temp_method + ".json";

                FILE* fp;
                if((fp = fopen(path.c_str(), "rb")) == NULL){
                    res.status = 404;
                    res.set_content("{\n  \"msg\": \"Method does not exist.\"\n}", "application/json");
                    return 0;
                }

                char readBuffer[65536];
                rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));

                rapidjson::Document fdoc;
                fdoc.ParseStream(is);

                rapidjson::StringBuffer buffer;
	            rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
	            fdoc.Accept(writer);

                //httplib::Client cli(CLOUD_IP, CLOUD_PORT);
                //cli.Post("/register/method", buffer.GetString(), "application/json");

                res.set_content(buffer.GetString(), "application/json");

                if(xarm_client.xarm_record(6))
                    res.status = 200;
                else{
                    res.status = 404;
                    res.set_content("{\n  \"msg\": \"Robot internal Error.\"\n}", "application/json");
                }
            } else if(strcmp(doc["robot_command"].GetString(), "confirm_method") == 0){
                RCLCPP_INFO(node->get_logger(), "Get method command received.");

                std::string path(doc["robot_param"].GetString());
                path += ".json";

                FILE* fp;
                if((fp = fopen(path.c_str(), "rb")) == NULL){
                    res.status = 404;
                    res.set_content("{\n  \"msg\": \"Method does not exist.\"\n}", "application/json");
                    return 0;
                }

                char readBuffer[65536];
                rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));

                rapidjson::Document fdoc;
                fdoc.ParseStream(is);

                rapidjson::StringBuffer buffer;
	            rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
	            fdoc.Accept(writer);

                //httplib::Client cli(CLOUD_IP, CLOUD_PORT);
                //cli.Post("/register/method", buffer.GetString(), "application/json");

                res.set_content(buffer.GetString(), "application/json");

            } else if(strcmp(doc["robot_command"].GetString(), "set_method") == 0){
                RCLCPP_INFO(node->get_logger(), "Change method command received.");
                std::string method_path = std::string(doc["robot_param"].GetString()) + ".json";
                if(access(method_path.c_str(), F_OK) != 0){
                    res.status = 404;
                    res.set_content("{\n  \"msg\": \"The method does not exists!.\"\n}", "application/json");
                    return 0;
                }
                if(xarm_client.xarm_record(7, std::string(doc["robot_param"].GetString())))
                    res.status = 200;
                else{
                    res.status = 404;
                    res.set_content("{\n  \"msg\": \"Robot internal Error.\"\n}", "application/json");
                }
            } else if(strcmp(doc["robot_command"].GetString(), "get_method") == 0){
                RCLCPP_INFO(node->get_logger(), "Get method command received.");
                rapidjson::Document m_doc;
                rapidjson::ParseResult m_Pres = m_doc.Parse(doc["robot_param"].GetString());
                if(!m_Pres){
                    res.status = 404;
                    res.set_content("{\n  \"msg\": \"Method is not Json format\"\n}", "application/json");
                    return 0;
                }
                //rapidjson::Value::ConstMemberIterator itr = m_doc.MemberBegin()
                std::string method_path = std::string(m_doc.MemberBegin()->name.GetString()) + ".json";
                if(access(method_path.c_str(), F_OK) == 0){
                    res.status = 404;
                    res.set_content("{\n  \"msg\": \"The method already exists!\"\n}", "application/json");
                    return 0;
                }
                std::ofstream file;
                file.open(method_path.c_str());
                std::string method_content = std::string(doc["robot_param"].GetString());
                file.write(method_content.c_str(), method_content.size());
                res.status = 200;
            } else if(strcmp(doc["robot_command"].GetString(), "exec_method") == 0){
                RCLCPP_INFO(node->get_logger(), "Execute method command received.");
                if(xarm_client.xarm_record(8))
                    res.status = 200;
                else{
                    res.status = 404;
                    res.set_content("{\n  \"msg\": \"Robot internal Error.\"\n}", "application/json");
                }
            } else if(strcmp(doc["robot_command"].GetString(), "exec_method_wo_camera") == 0){
                RCLCPP_INFO(node->get_logger(), "Execute method without camera command received.");
                if(xarm_client.xarm_record(9))
                    res.status = 200;
                else{
                    res.status = 404;
                    res.set_content("{\n  \"msg\": \"Robot internal Error.\"\n}", "application/json");
                }
            } else if(strcmp(doc["robot_command"].GetString(), "clear_error") == 0){
                int tmp_state;
                RCLCPP_INFO(node->get_logger(), "Error clear received.");
                xarm_client.clean_error();
                xarm_client.clean_warn();
                xarm_client.motion_enable(true);
                xarm_client.set_mode(1);
                xarm_client.set_state(0);
                xarm_client.get_state(&tmp_state);
                //xarm_client.get_mode(tmp_mode); 있는지 확인해야함
                if(tmp_state == 2) res.status = 200; // 2가 맞는지 확인
                else{
                    res.status = 404;
                    res.set_content("{\n  \"msg\": \"Robot internal Error.\"\n}", "application/json");
                }
            } else{
                res.status = 404;
                res.set_content("{\n  \"msg\": \"Wrong command.\"\n}", "application/json");
            }
        } else{ //시리얼 불일치
            res.status = 404;
            res.set_content("{\n  \"msg\": \"Serial not matched.\"\n}", "application/json");
        }
    });

    //svr.set_error_handler([](const httplib::Request & /*req*/, httplib::Response &res) {
    //    const char *fmt = "<p>Error Status: <span style='color:red;'>%d</span></p>"; //바꿔야함.
    //    char buf[BUFSIZ];
    //    snprintf(buf, sizeof(buf), fmt, res.status);
    //    res.set_content(buf, "text/html");
    //});

    svr.set_logger([&](const httplib::Request &req, const httplib::Response &res) {
        RCLCPP_INFO(node->get_logger(), "\n%s", log(req, res).c_str());
    });
    
    RCLCPP_INFO(node->get_logger(), "xarm_http_server start");

    svr.listen(SERVER_IP,SERVER_PORT);

    RCLCPP_INFO(rclcpp::get_logger("xarm_http_server"), "xarm_http_server over");
}