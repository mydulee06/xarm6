# ROS2 Architecture Description

## Dependencies

도커 허브 이미지에 이미 다 깔려있긴 한대 혹시 몰라 언급합니다.

[RapidJson](https://github.com/Tencent/rapidjson): C++ Json 라이브러리
[cpp-httplib](https://github.com/yhirose/cpp-httplib): C++ http 라이브러리
[Paho_mqtt_cpp](https://github.com/eclipse/paho.mqtt.cpp): C++ mqtt 라이브러리

## Introduction

제가 한 작업들은 기본적으로 ros2_xarm 레포지토리를 기반으로
1. 로봇 moveit scene에 충돌 물체 추가
2. 로봇팔 hand-teaching 추가
3. 서버와 http, mqtt 통신

입니다. 일단은 제가 한 작업들 한게 중요하니깐 그거 위주로 설명하고 필요하면 전반적인 설명 덧붙이겠습니다.

## Node Description

아래 명령어를 실행하면

    ros2 launch xarm_planner xarm6_planner_realmove_wo_rviz.launch.py robot_ip:=192.168.1.222 report_type:='rich' server_ip:='192.168.0.77' port:=5000 object_name:="desk" mesh_file:="package://xarm_object/meshes/desk.stl"

node가 동시에 실행되는데, 중요한 node들 몇가지만 뽑아 설명하자면 다음과 같습니다.

* [/xarm_driver_node](https://github.com/mydulee06/xarm6/blob/main/xarm_api/src/xarm_driver_node.cpp): xarm6 구동에 필요한 기본적인 드라이버 실행하는 node. Ex) 로봇 상태, 에러코드, ip 등등 publish.
* [/xarm_planner_node](https://github.com/mydulee06/xarm6/blob/main/xarm_planner/src/xarm_planner_node.cpp): moveit api를 이용하여 로봇팔이 동작하도록 하는 node.
* [/add_object](https://github.com/mydulee06/xarm6/blob/main/xarm_object/src/xarm_object.cpp): 로봇팔이 동작하는 scene에 물체 추가하는 node (물체를 피해서 로봇 팔이 움직임(planning)).
* [/xarm_camera_socket](https://github.com/mydulee06/xarm6/blob/main/xarm_network/src/xarm_camera_socket.cpp): 비전 카메라와 소켓 통신하는 node.
* [/xarm_http_server](https://github.com/mydulee06/xarm6/blob/main/xarm_network/src/xarm_http_server.cpp): 서버와 통신할 때 서버 역할을 하는 node.
* [/xarm_http_client](https://github.com/mydulee06/xarm6/blob/main/xarm_network/src/xarm_http_client.cpp): 서버와 통신할 때 클라이언트 역할을 하는 node.
* [/xarm_mqtt_pub](https://github.com/mydulee06/xarm6/blob/main/xarm_network/src/xarm_mqtt_pub.cpp): 서버에 로봇팔의 전류, 전압을 mqtt로 쏴주는 node.

## Topic, Service Description

서로 다른 node가 통신할 때 쓰는 ROS 기법이 topic(publisher, subscriber)이랑 service(service, client)인데 각 node에 구현되어 있습니다. 아래는 각 node에 있는 topic과 service의 간단한 설명입니다.

* /xarm_driver_node
    * /xarm/xarm_states(publisher): 로봇의 상태, 에러코드, 시리얼 넘버 등등을 publish.
    
* /xarm_planner_node
    * /xarm/xarm_record(service): 로봇 타겟 위치 지정해 줄 때 사용하는 service.

* /xarm_camera_socket
    * /send_msg_to_camera(publisher): 로봇 동작에 따라 카메라에 socket 통신하는 service.

* /xarm_http_server
    * 따로 publisher나 service는 없고 서버(사용자 측)에서 들어오는 요청을 다른 node에 client 형태로 요청합니다.

* /xarm_http_client
    * 역시나 따로 publisher나 service는 없고 로봇의 상태코드와 에러코드를 /xarm/xarm_states으로부터 읽어서 서버에 http로 쏴줍니다.

* /xarm_mqtt_pub
    * 마찬가지로 publisher나 service는 없고 로봇의 전류와 전압을 서버에 쏴줍니다.

## Detailed Description

자세한 코드 기술은 아래의 코드 파일들 안에 자바독 형태로 적어놓았습니다.

* xarm_planner/src/xarm_planner_node.cpp
* xarm_planner/src/xarm_planner.cpp
* xarm_network/src/xarm_camera_socket.cpp
* xarm_network/src/xarm_http_server.cpp
* xarm_network/src/xarm_http_client.cpp
* xarm_network/src/xarm_mqtt_pub.cpp
