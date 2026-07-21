cmake_minimum_required(VERSION 3.8)
project(joyrebot_gazebo_sim)

find_package(ament_cmake REQUIRED)

# ── C++ test node ──────────────────────────────────────────────────────────────
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(test_joints src/test_joints.cpp)
ament_target_dependencies(test_joints rclcpp std_msgs sensor_msgs)

install(TARGETS test_joints
  DESTINATION lib/${PROJECT_NAME}
)

# ── Simulation assets ──────────────────────────────────────────────────────────
install(DIRECTORY config launch meshes urdf worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

# ── Tests ─────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_model test/test_model.py)
endif()

ament_package()
