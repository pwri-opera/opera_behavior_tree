cmake_minimum_required(VERSION 3.8)
project(opera_behavior_tree CXX)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(com3_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(opera_bt_msgs REQUIRED)

nav2_package()

set(library_name ${PROJECT_NAME})

# add_library(${library_name} SHARED
#   src/behavior_tree_engine.cpp
# )
# target_include_directories(${library_name}
#   PUBLIC
#   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
# target_link_libraries(${library_name} PUBLIC
#   ${action_msgs_TARGETS}
#   behaviortree_cpp::behaviortree_cpp
#   ${geometry_msgs_TARGETS}
#   ${nav_msgs_TARGETS}
#   ${nav2_msgs_TARGETS}
#   nav2_util::nav2_util_core
#   rclcpp::rclcpp
#   rclcpp_action::rclcpp_action
#   rclcpp_lifecycle::rclcpp_lifecycle
#   ${sensor_msgs_TARGETS}
#   tf2::tf2
#   tf2_ros::tf2_ros
# )

# add_library(nav2_wait_action_bt_node SHARED plugins/action/wait_action.cpp)
# list(APPEND plugin_libs nav2_wait_action_bt_node)


add_library(enable_route_switcher_bt_node SHARED plugins/action/enable_route_switcher_node.cpp)
list(APPEND plugin_libs enable_route_switcher_bt_node)

add_library(check_safety_bt_node SHARED plugins/action/check_safety_node.cpp)
list(APPEND plugin_libs check_safety_bt_node)

# add_library(get_pose_from_path_with_area_node SHARED plugins/action/get_pose_from_apth_with_area.cpp)
# list(APPEND plugin_libs get_pose_from_path_with_area_node)

add_library(get_area_node SHARED plugins/action/get_area_node.cpp)
list(APPEND plugin_libs get_area_node)


add_library(align_path_orientations_bt_node SHARED plugins/action/align_path_orientations.cpp)
list(APPEND plugin_libs align_path_orientations_bt_node)

add_library(set_path_goal_orientation_bt_node SHARED plugins/action/set_path_goal_orientation.cpp)
list(APPEND plugin_libs set_path_goal_orientation_bt_node)

add_library(set_dump_angle_bt_node SHARED plugins/action/set_dump_angle_action.cpp)
add_library(navigate_to_pose_goal_bt_node SHARED plugins/action/navigate_to_pose_goal_action.cpp)
list(APPEND plugin_libs navigate_to_pose_goal_bt_node)
add_library(set_swing_angle_bt_node SHARED plugins/action/set_swing_angle_action.cpp)
list(APPEND plugin_libs set_dump_angle_bt_node set_swing_angle_bt_node)

foreach(bt_plugin ${plugin_libs})
  target_include_directories(${bt_plugin}
    PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
  target_link_libraries(${bt_plugin} PUBLIC
    behaviortree_cpp::behaviortree_cpp
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${nav2_msgs_TARGETS}
    nav2_behavior_tree::nav2_behavior_tree
    nav2_util::nav2_util_core
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    rclcpp_lifecycle::rclcpp_lifecycle
    ${sensor_msgs_TARGETS}
    tf2::tf2
    tf2_ros::tf2_ros
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS}
    ${opera_bt_msgs_TARGETS}
    ${com3_msgs_TARGETS}
  )
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(run_angle_tree src/run_angle_tree.cpp)
target_link_libraries(run_angle_tree PRIVATE
  behaviortree_cpp::behaviortree_cpp rclcpp::rclcpp ament_index_cpp::ament_index_cpp)
install(TARGETS run_angle_tree DESTINATION lib/${PROJECT_NAME})

install(TARGETS ${plugin_libs}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# # we will embed the list of plugin names inside a header file
# set(GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/gen)
# configure_file(plugins_list.hpp.in ${GENERATED_DIR}/plugins_list.hpp)

# add_executable(generate_nav2_tree_nodes_xml src/generate_nav2_tree_nodes_xml.cpp)
# target_link_libraries(generate_nav2_tree_nodes_xml PRIVATE
#   behaviortree_cpp::behaviortree_cpp
#   nav2_util::nav2_util_core
# )
# # allow generate_nav2_tree_nodes_xml to find plugins_list.hpp
# target_include_directories(generate_nav2_tree_nodes_xml PRIVATE ${GENERATED_DIR})
# install(TARGETS generate_nav2_tree_nodes_xml DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(FILES nav2_tree_nodes.xml DESTINATION share/${PROJECT_NAME})
# install(FILES ${GENERATED_DIR}/plugins_list.hpp DESTINATION include/${PROJECT_NAME}/${PROJECT_NAME})


ament_export_include_directories(
  include/${PROJECT_NAME}
)

ament_export_libraries(${library_name})

ament_export_dependencies(
  action_msgs
  com3_msgs
  behaviortree_cpp
  geometry_msgs
  nav2_behavior_tree
  nav2_common
  nav2_msgs
  nav2_util
  nav_msgs
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  opera_bt_msgs
)
ament_export_targets(${library_name})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_align_path_orientations test/test_align_path_orientations.cpp)
  target_link_libraries(test_align_path_orientations align_path_orientations_bt_node)
  target_compile_definitions(test_align_path_orientations PRIVATE
    ALIGN_PLUGIN_PATH="$<TARGET_FILE:align_path_orientations_bt_node>")
  ament_add_gtest(test_align_path_route_entry test/test_align_path_route_entry.cpp)
  target_link_libraries(test_align_path_route_entry align_path_orientations_bt_node)
  target_compile_definitions(test_align_path_route_entry PRIVATE
    ENTRY_PLUGIN_PATH="$<TARGET_FILE:align_path_orientations_bt_node>")
  ament_add_gtest(test_set_path_goal_orientation test/test_set_path_goal_orientation.cpp)
  target_link_libraries(test_set_path_goal_orientation set_path_goal_orientation_bt_node)
  target_compile_definitions(test_set_path_goal_orientation PRIVATE
    GOAL_PLUGIN_PATH="$<TARGET_FILE:set_path_goal_orientation_bt_node>")
endif()

ament_package()
