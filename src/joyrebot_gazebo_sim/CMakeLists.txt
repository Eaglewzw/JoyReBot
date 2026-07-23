cmake_minimum_required(VERSION 3.8)
project(joyrebot_gazebo_sim)

find_package(ament_cmake REQUIRED)

# ── C++ test node ──────────────────────────────────────────────────────────────
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(joint_self_check src/joint_self_check.cpp)
target_include_directories(joint_self_check PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(joint_self_check rclcpp std_msgs sensor_msgs)

install(TARGETS joint_self_check
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
