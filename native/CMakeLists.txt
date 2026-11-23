cmake_minimum_required(VERSION 3.16)
project(web_control_native LANGUAGES CXX)

find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(action_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lio_sam_hesai REQUIRED)
find_package(keyboard_handler REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
find_path(JSON_INCLUDE_DIRECTORY nlohmann/json.hpp REQUIRED)

add_library(web_control_terrain STATIC terrain.cpp cloud_response.cpp)
target_compile_features(web_control_terrain PUBLIC cxx_std_17)
target_include_directories(web_control_terrain PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR} ${JSON_INCLUDE_DIRECTORY})
target_link_libraries(web_control_terrain PUBLIC ${visualization_msgs_TARGETS})

add_library(web_control_services STATIC catalog.cpp diagnostics.cpp managed_processes.cpp robot_control.cpp)
target_compile_features(web_control_services PUBLIC cxx_std_17)
target_include_directories(web_control_services PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR} ${JSON_INCLUDE_DIRECTORY})
target_link_libraries(web_control_services PUBLIC
  rclcpp::rclcpp rosbag2_cpp::rosbag2_cpp rosbag2_transport::rosbag2_transport
  yaml-cpp Threads::Threads keyboard_handler::keyboard_handler ${action_msgs_TARGETS} ${diagnostic_msgs_TARGETS}
  ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${sensor_msgs_TARGETS})

add_executable(web_control_cpp web_control.cpp http_server.cpp scene.cpp system_status.cpp tf_tree.cpp)
target_compile_definitions(web_control_cpp PRIVATE
  WEB_CONTROL_STATIC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../static")
target_link_libraries(web_control_cpp PRIVATE
  web_control_terrain web_control_services Boost::system ament_index_cpp::ament_index_cpp
  ${lio_sam_hesai_TARGETS} ${rcl_interfaces_TARGETS} ${tf2_msgs_TARGETS})

include(CTest)
if(BUILD_TESTING)
  add_executable(terrain_test terrain_test.cpp)
  target_link_libraries(terrain_test PRIVATE web_control_terrain)
  add_test(NAME terrain_conversion COMMAND terrain_test)
  foreach(component IN ITEMS catalog diagnostics managed_processes robot_control)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${component}_test.cpp")
      add_executable(${component}_test ${component}_test.cpp)
      target_link_libraries(${component}_test PRIVATE web_control_services)
      add_test(NAME ${component} COMMAND ${component}_test)
    endif()
  endforeach()
endif()
