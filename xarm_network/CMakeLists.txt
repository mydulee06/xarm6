cmake_minimum_required(VERSION 3.5)

project(xarm_network)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic
    -Wno-sign-compare
    -Wno-unused-parameter 
    -Wno-unused-variable
  )
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(rclcpp REQUIRED)
find_package(xarm_msgs REQUIRED)
find_package(PahoMqttCpp REQUIRED)
find_package(RapidJSON REQUIRED)
find_package(xarm_api REQUIRED)
find_package(CURL REQUIRED)

set(dependencies "rclcpp" "xarm_msgs" "xarm_api")

include_directories(
  include
  ${xarm_sdk_INCLUDE_DIRS}
  ${ament_INCLUDE_DIRS}
  ${CURL_INCLUDE_DIR}
)

add_executable(xarm_mqtt_pub 
  src/xarm_mqtt_pub.cpp
)

ament_target_dependencies(xarm_mqtt_pub
  ${dependencies}
)

target_link_libraries(xarm_mqtt_pub
  ${xarm_sdk_LIBRARIES}
  ${xarm_api_LIBRARIES}
  ${ament_LIBRARIES}
  paho-mqttpp3
  paho-mqtt3as
)

add_executable(xarm_http_server 
  src/xarm_http_server.cpp
)

ament_target_dependencies(xarm_http_server
  ${dependencies}
)

target_link_libraries(xarm_http_server
  ${xarm_sdk_LIBRARIES}
  ${xarm_api_LIBRARIES}
  ${ament_LIBRARIES}
  ${CURL_LIBRARIES}
)

add_executable(xarm_http_client 
  src/xarm_http_client.cpp
)

ament_target_dependencies(xarm_http_client
  ${dependencies}
)

target_link_libraries(xarm_http_client
  ${xarm_sdk_LIBRARIES}
  ${xarm_api_LIBRARIES}
  ${ament_LIBRARIES}
)

add_executable(xarm_camera_socket
  src/xarm_camera_socket.cpp
)

ament_target_dependencies(xarm_camera_socket
  ${dependencies}
)

target_link_libraries(xarm_camera_socket
  ${xarm_sdk_LIBRARIES}
  ${xarm_api_LIBRARIES}
  ${ament_LIBRARIES}
)

add_executable(xarm_register 
  src/xarm_register.cpp
)

ament_target_dependencies(xarm_register
  ${dependencies}
)

target_link_libraries(xarm_register
  ${ament_LIBRARIES}
)


install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS 
  xarm_mqtt_pub
  xarm_http_server
  xarm_http_client
  xarm_camera_socket
  xarm_register
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

#install(DIRECTORY
#  launch
#  meshes
#  DESTINATION share/${PROJECT_NAME}/
#)

ament_package()