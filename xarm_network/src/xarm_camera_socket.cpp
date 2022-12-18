#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <xarm_msgs/srv/set_int16.hpp>

void error_handling(const char *message);
#define BIND_CLS_CB(func) std::bind(func, this, std::placeholders::_1, std::placeholders::_2)

typedef struct { 
    char header;
    char CMD[10] = "";
    char data[30] = "";
} packet_t;


class Camera_Socket{
    public:
    void init_service(rclcpp::Node::SharedPtr& node){
        node_ = node;
        hw_node_ = node_->create_sub_node(std::string("xarm_camera_socket"));
        service_send_msg_to_camera =
            hw_node_->create_service<xarm_msgs::srv::SetInt16>("send_msg_to_camera", BIND_CLS_CB(&Camera_Socket::camera_socket));
        RCLCPP_INFO(node_->get_logger(), "Created service: %s", service_send_msg_to_camera->get_service_name());
    }
    
    // 소켓 생성과 연결을 성공하면 1, 소켓 생성을 실패하면 2, 서버 소켓과 연결을 실패하면 3 리턴.
    int init_socket(std::string server_ip, std::string port)
    {
        sock_ = socket(PF_INET, SOCK_STREAM, 0);
        if(sock_ == -1)
            return 2;
	
        // 클라이언트와 마찬가지로 주소정보를 초기화
        struct sockaddr_in serv_addr;
        memset(&serv_addr, 0, sizeof(serv_addr));
        serv_addr.sin_family=AF_INET;
        serv_addr.sin_addr.s_addr=inet_addr(server_ip.c_str());
        serv_addr.sin_port=htons(atoi(port.c_str()));
        if(connect(sock_, (struct sockaddr*)&serv_addr, sizeof(serv_addr))==-1) 
            return 3;
        return 1;
    }

    /**
     * xarm6_planner_node에서 요청하는 request에 따라 비전 카메라와 소켓 통신을 통해 상호 작용
     * request : data : 0 -> 동작 실패, data : 1 -> 동작 완료
     * response : ret : 0 -> 소켓 read 실패, ret : 1 -> 소켓 read 성공
     * 0 : 검사 취소, 1 : 검사 시작, 2 : 얼라인 요청, 3 : 타겟 포인트 비전 검사, 4 : 비전 검사 완료
     */
    bool camera_socket(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res)
    {
        int str_len;
        char msg_from_server[30];
        res->message = "data=" + std::to_string(req->data);

        if (req->data == 0)
        {
            send_packet->header = '@';
            char CMD_[11] = "LOT_CANCEL";
            for (int i = 0; i < 10; i++){
                if (CMD_[i] == '\0') CMD_[i] = ' ';
            }
            char data_[] = " \r";
            strncpy(send_packet->CMD, CMD_, sizeof(CMD_) - 1);
            strncpy(send_packet->data, data_, sizeof(data_));
            write(sock_,send_packet,sizeof(*send_packet));
            RCLCPP_DEBUG(rclcpp::get_logger("xarm_camera_socket"), "send_packet CMD: %s", send_packet->CMD);
            RCLCPP_DEBUG(rclcpp::get_logger("xarm_camera_socket"), "send_packet data: %s", send_packet->data);
        } else if(req->data == 1)
        {
            send_packet->header = '@';
            char CMD_[10] = "LOT_START";
            for (int i = 0; i < 10; i++){
                if (CMD_[i] == '\0') CMD_[i] = ' ';
            }
            char data_[] = "LOTID,45\r";
            strncpy(send_packet->CMD, CMD_, sizeof(CMD_));
            strncpy(send_packet->data, data_, sizeof(data_));
            write(sock_,send_packet,sizeof(*send_packet));
            RCLCPP_DEBUG(rclcpp::get_logger("xarm_camera_socket"), "send_packet CMD: %s", send_packet->CMD);
            RCLCPP_DEBUG(rclcpp::get_logger("xarm_camera_socket"), "send_packet data: %s", send_packet->data);
        } else if(req->data == 2)
        {
            send_packet->header = '@';
            char CMD_[11] = "ALIGN_REQ";
            for (int i = 0; i < 10; i++){
                if (CMD_[i] == '\0') CMD_[i] = ' ';
            }
            char data_[] = " \r";
            str_len=read(sock_, msg_from_server, sizeof(msg_from_server)-1);
            if(str_len==-1){
                res->ret = 0;
                return res->ret == 1;
            }
            char data_from_server[10] = "";
            strncpy(data_from_server, msg_from_server+11,11);
            RCLCPP_INFO(rclcpp::get_logger("xarm_camera_socket"), "Message from server: %s", msg_from_server);
            RCLCPP_INFO(rclcpp::get_logger("xarm_camera_socket"), "Data from server: %s", data_from_server);
            res->message = data_from_server;

        } else if(req->data == 3)
        {
            send_packet->header = '@';
            char CMD_[10] = "MOVE_COMP";
            for (int i = 0; i < 10; i++){
                if (CMD_[i] == '\0') CMD_[i] = ' ';
            }
            char data_[] = " \r";
            strncpy(send_packet->CMD, CMD_, sizeof(CMD_));
            strncpy(send_packet->data, data_, sizeof(data_));
            write(sock_,send_packet,sizeof(*send_packet));
            RCLCPP_DEBUG(rclcpp::get_logger("xarm_camera_socket"), "send_packet CMD: %s", send_packet->CMD);
            RCLCPP_DEBUG(rclcpp::get_logger("xarm_camera_socket"), "send_packet data: %s", send_packet->data);
            str_len=read(sock_, msg_from_server, sizeof(msg_from_server)-1);
            if(str_len==-1){
                res->ret = 0;
                return res->ret == 1;
            }
            char data_from_server[10] = "";
            strncpy(data_from_server, msg_from_server+11,3);
            RCLCPP_DEBUG(rclcpp::get_logger("xarm_camera_socket"), "Message from server: %s", msg_from_server);
            RCLCPP_DEBUG(rclcpp::get_logger("xarm_camera_socket"), "Data from server: %s", data_from_server);
            if(strcmp(data_from_server,"ACK") == 0){
                RCLCPP_INFO(rclcpp::get_logger("xarm_camera_socket"), "Server ACK");
                res->ret = 1;
            } else{
                RCLCPP_INFO(rclcpp::get_logger("xarm_camera_socket"), "Server NAK");
                res->ret = 0;
                return res->ret == 1;
            }
            str_len=read(sock_, msg_from_server, sizeof(msg_from_server)-1);
            if(str_len==-1){
                res->ret = 0;
                return res->ret == 1;
            }
            strncpy(data_from_server, msg_from_server+1,9);
            RCLCPP_DEBUG(rclcpp::get_logger("xarm_camera_socket"), "Message from server: %s", msg_from_server);
            RCLCPP_DEBUG(rclcpp::get_logger("xarm_camera_socket"), "Data from server: %s", data_from_server);
            if(strcmp(data_from_server,"INSP_COMP") == 0){
                RCLCPP_DEBUG(rclcpp::get_logger("xarm_camera_socket"), "Server INSP_COMP");
                //char send_packet_2[] = "#INSP_COMP  \r";
                send_packet->header = '#';
                char CMD_[10] = "INSP_COMP";
                for (int i = 0; i < 10; i++){
                    if (CMD_[i] == '\0') CMD_[i] = ' ';
                }
                char data_[] = "ACK\r";
                strncpy(send_packet->CMD, CMD_, sizeof(CMD_));
                strncpy(send_packet->data, data_, sizeof(data_));
                write(sock_,send_packet,sizeof(*send_packet));
                RCLCPP_DEBUG(rclcpp::get_logger("xarm_camera_socket"), "send_packet CMD: %s", send_packet->CMD);
                RCLCPP_DEBUG(rclcpp::get_logger("xarm_camera_socket"), "send_packet data: %s", send_packet->data);
                res->ret = 1;
            } else{
                RCLCPP_DEBUG(rclcpp::get_logger("xarm_camera_socket"), "Server not INSP_COM");
                res->ret = 0;
            }
            return res->ret == 1;
        } else if(req->data == 4)
        {
            send_packet->header = '@';
            char CMD_[10] = "LOT_END";
            for (int i = 0; i < 10; i++){
                if (CMD_[i] == '\0') CMD_[i] = ' ';
            }
            char data_[] = " \r";
            strncpy(send_packet->CMD, CMD_, sizeof(CMD_));
            strncpy(send_packet->data, data_, sizeof(data_));
            write(sock_,send_packet,sizeof(*send_packet));
            RCLCPP_DEBUG(rclcpp::get_logger("xarm_camera_socket"), "send_packet CMD: %s", send_packet->CMD);
            RCLCPP_DEBUG(rclcpp::get_logger("xarm_camera_socket"), "send_packet data: %s", send_packet->data);
        } 

        str_len=read(sock_, msg_from_server, sizeof(msg_from_server)-1);
        if(str_len==-1){
            res->ret = 0;
        }
        char data_from_server[8] = "";
        strncpy(data_from_server, msg_from_server+11,3);
        RCLCPP_DEBUG(rclcpp::get_logger("xarm_camera_socket"), "Message from server: %s", msg_from_server);
        RCLCPP_DEBUG(rclcpp::get_logger("xarm_camera_socket"), "Data from server: %s", data_from_server);
        if(strcmp(data_from_server,"ACK") == 0){
            RCLCPP_INFO(rclcpp::get_logger("xarm_camera_socket"), "Server ACK");
            res->ret = 1;
        } else{
            RCLCPP_INFO(rclcpp::get_logger("xarm_camera_socket"), "Server NAK");
            res->ret = 0;
        }
        res->ret = 1;
        return res->ret == 1;
    }
    private:
    int sock_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Node::SharedPtr hw_node_;
    rclcpp::Service<xarm_msgs::srv::SetInt16>::SharedPtr service_send_msg_to_camera;
    packet_t *send_packet;
    packet_t *read_packet;
};



int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("xarm_camera_socket",node_options);
    RCLCPP_INFO(rclcpp::get_logger("xarm_camera_socket"), "xarm_camera_socket start");

    std::string server_ip;
    int port;
    node->get_parameter_or("server_ip", server_ip, std::string("192.168.0.77"));
    node->get_parameter_or("port", port, 5000);
    RCLCPP_INFO(rclcpp::get_logger("xarm_camera_socket"), "Server ip : %s", server_ip.c_str());

    Camera_Socket camera_socket;

    switch(camera_socket.init_socket(server_ip, std::to_string(port))){
        case 1:
            RCLCPP_INFO(rclcpp::get_logger("xarm_camera_socket"), "Socket creation and connection are Success!");
            camera_socket.init_service(node);
            break;
        case 2:
            RCLCPP_INFO(rclcpp::get_logger("xarm_camera_socket"), "Socket creation is Failed!");
            break;
        case 3:
            RCLCPP_INFO(rclcpp::get_logger("xarm_camera_socket"), "Socket connection is Failed!");
            break;
    };

    rclcpp::spin(node);
    rclcpp::shutdown();
    RCLCPP_INFO(node->get_logger(), "xarm_camera_socket over");
    return 0;
}

void error_handling(const char *message)
{
    fputs(message, stderr);
    fputc('\n', stderr);
    exit(1);
}