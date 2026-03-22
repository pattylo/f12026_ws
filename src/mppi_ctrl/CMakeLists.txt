cmake_minimum_required(VERSION 3.8)
project(mppi_ctrl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(visualization_msgs REQUIRED)
# find_package(bezier_traj_msgs REQUIRED)
# find_package(osqp REQUIRED)
# find_package(OsqpEigen REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

# EXECUTABLES HERE
###################################################################

# scout_traj_server
# add_executable(scout_traj_server_node
# 	src/scout_traj_server_node.cpp
#     src/scout_traj_server.cpp
    
#     src/bernstein.cpp
#     src/scout_traj_gen.cpp
# )
# ament_target_dependencies(scout_traj_server_node
# 	rclcpp 
# 	geometry_msgs
# 	sensor_msgs
# 	mavros_msgs
# 	visualization_msgs
#     bezier_traj_msgs
# )
# target_link_libraries(scout_traj_server_node
# 	osqp::osqp
# 	OsqpEigen::OsqpEigen
# )
# install(TARGETS scout_traj_server_node DESTINATION lib/${PROJECT_NAME})

# scout_sim_server
add_executable(servo_rc
	src/servo_rc.cpp
)
ament_target_dependencies(servo_rc
	rclcpp 
	geometry_msgs
	sensor_msgs
	mavros_msgs
	visualization_msgs
    # bezier_traj_msgs
)
install(TARGETS servo_rc DESTINATION lib/${PROJECT_NAME})

# rate_ctrl
add_executable(rate_ctrl
	src/rate_ctrl.cpp
)
ament_target_dependencies(rate_ctrl
	rclcpp 
	geometry_msgs
	sensor_msgs
	mavros_msgs
	visualization_msgs
    # bezier_traj_msgs
)
install(TARGETS rate_ctrl DESTINATION lib/${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  script/servo_rc.py
  script/servo_test.py
  DESTINATION lib/${PROJECT_NAME}
)

###################################################################
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()