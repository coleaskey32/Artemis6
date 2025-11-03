cmake_minimum_required(VERSION 3.8)
project(robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files and config
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install Webots worlds, URDF, PROTO, and controllers
install(DIRECTORY
  worlds
  urdf
  protos
  controllers
  DESTINATION share/${PROJECT_NAME}/
)

# Install ROS2 base controller script
install(PROGRAMS
  scripts/base_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME base_controller
)

# Install square drive script
install(PROGRAMS
  scripts/square_drive.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME square_drive
)

# Install arm controller script
install(PROGRAMS
  scripts/arm_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME arm_controller
)

# Install pose controller script
install(PROGRAMS
  scripts/pose_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pose_controller
)

# Install joint state publisher script
install(PROGRAMS
  scripts/joint_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME joint_state_publisher
)

# Install arm teleoperation script
install(PROGRAMS
  scripts/arm_teleop.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME arm_teleop
)

# Install IK solver script
install(PROGRAMS
  scripts/ik_solver.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ik_solver
)

# Install pick and place demo script
install(PROGRAMS
  scripts/pick_and_place_demo.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pick_demo
)

# ------------------------------------------------------------
# Install led status script
install(PROGRAMS
  scripts/radar_project/led_status.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME led_status
)

# Install lidar monitor script
install(PROGRAMS
  scripts/radar_project/lidar_monitor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME lidar_monitor
)
# ------------------------------------------------------------

# Make controller executable
install(PROGRAMS
  controllers/robot_controller/robot_controller
  controllers/robot_controller/robot_controller.py
  DESTINATION share/${PROJECT_NAME}/controllers/robot_controller
)

ament_package()
