cmake_minimum_required(VERSION 3.8)
project(cf_adapters)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)

add_executable(world_octomap src/world_octomap.cpp)
target_link_libraries(world_octomap ${OCTOMAP_LIBRARIES})
target_include_directories(world_octomap PRIVATE ${OCTOMAP_INCLUDE_DIRS})
ament_target_dependencies(world_octomap rclcpp octomap_msgs)

install(TARGETS world_octomap DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/mavros_relay.py scripts/stereo_eval.py scripts/tracking_point.py scripts/droan_bridge.py DESTINATION lib/${PROJECT_NAME})

ament_package()
