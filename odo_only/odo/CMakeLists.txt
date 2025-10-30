cmake_minimum_required(VERSION 3.8)
project(odo)

# C++17 사용
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 컴파일 옵션
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 필수 ROS2 패키지
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# include 디렉토리 설정 (필요 시)
include_directories(include)

### 🔹 노드 1: odo_node
add_executable(odo_node src/odo_node.cpp)
ament_target_dependencies(odo_node
  rclcpp std_msgs nav_msgs sensor_msgs tf2 tf2_geometry_msgs tf2_ros
)

### 🔹 노드 2: sensor_node
add_executable(sensor_node src/sensor_node.cpp)
ament_target_dependencies(sensor_node
  rclcpp std_msgs sensor_msgs
)

### 🔹 노드 3: twist_input_publisher
add_executable(twist_input_publisher src/twist_input_publisher.cpp)
ament_target_dependencies(twist_input_publisher
  rclcpp std_msgs nav_msgs tf2 tf2_geometry_msgs
)

# 노드 설치
install(TARGETS
  odo_node
  sensor_node
  twist_input_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# launch 파일 설치 (있다면)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
