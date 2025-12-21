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

# ============================================================================
# CONFIGURATION AND RESOURCE FILES
# ============================================================================

# Install launch files and config
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
  USE_SOURCE_PERMISSIONS  # Preserve executable permissions for scripts
)

# Install Webots worlds, URDF, PROTO, and controllers
install(DIRECTORY
  worlds
  urdf
  protos
  controllers
  DESTINATION share/${PROJECT_NAME}/
)

# ============================================================================
# REAL ROBOT NODES (Hardware/Production)
# ============================================================================

# Base controller - main robot drive controller
install(PROGRAMS
  ros_nodes/base_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME base_controller
)

# Arm controller - robot arm control
install(PROGRAMS
  ros_nodes/arm_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME arm_controller
)

# IK solver - inverse kinematics
install(PROGRAMS
  ros_nodes/ik_solver.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ik_solver
)

# Keyboard teleop - manual robot control
install(PROGRAMS
  ros_nodes/keyboard_control.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME keyboard_control
)

# IK test script
install(PROGRAMS
  ros_nodes/test_ik.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME test_ik
)

# Pick and place demo
install(PROGRAMS
  ros_nodes/pick_place_demo.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pick_place_demo
)

# ============================================================================
# EXAMPLE PROJECTS / DEMOS
# ============================================================================

# --- Square Drive Demo ---
install(PROGRAMS
  ros_nodes/square_drive.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME square_drive
)

# --- Pick and Place Demo ---
install(PROGRAMS
  example_projects/ros_nodes/pick_and_place_demo.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pick_demo
)

# --- Proximity Alert Demo ---
# Distance monitor - reads distance sensor and publishes safety status
install(PROGRAMS
  example_projects/proximity_alert_demo/ros_nodes/lidar_monitor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME lidar_monitor
)

# LED status controller - controls LEDs based on safety status
install(PROGRAMS
  example_projects/proximity_alert_demo/ros_nodes/led_status.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME led_status
)

# ============================================================================
# WEBOTS CONTROLLER EXECUTABLES
# ============================================================================

# Make controller executable
install(PROGRAMS
  controllers/robot_controller/robot_controller
  controllers/robot_controller/robot_controller.py
  DESTINATION share/${PROJECT_NAME}/controllers/robot_controller
)

ament_package()
