cmake_minimum_required(VERSION 3.16)
project(web_control LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
include(CTest)
add_subdirectory(native)

set_target_properties(web_control_cpp PROPERTIES OUTPUT_NAME web_control_node)
install(TARGETS web_control_cpp RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch static DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(FILES THIRD_PARTY_NOTICES.md DESTINATION share/${PROJECT_NAME})

ament_package()
