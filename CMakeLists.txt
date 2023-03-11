cmake_minimum_required(VERSION 3.5) #cmake 최소 버전
project(camera_ros2) #프로젝트명

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic) #디버깅 옵션
endif()

# find dependencies
find_package(ament_cmake REQUIRED) #패키지 찾기
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include) #해더파일 찾기

add_executable(camera_publisher1 src/camera_publisher1.cpp src/camera.cpp) #실행시킬 실행파일 기술
ament_target_dependencies(camera_publisher1 rclcpp sensor_msgs std_msgs OpenCV) #의존성있는 파일 기술

add_executable(camera_publisher2 src/camera_publisher2.cpp src/camera.cpp) #실행시킬 실행파일 기술
ament_target_dependencies(camera_publisher2 rclcpp sensor_msgs std_msgs OpenCV) #의존성있는 파일 기술

add_executable(camera_subscriber1 src/camera_subscriber1.cpp  src/camera.cpp) #실행시킬 실행파일 기술
ament_target_dependencies(camera_subscriber1 rclcpp sensor_msgs std_msgs OpenCV)#의존성있는 파일 기술

add_executable(camera_subscriber2 src/camera_subscriber2.cpp src/camera.cpp) #실행시킬 실행파일 기술
ament_target_dependencies(camera_subscriber2 rclcpp sensor_msgs std_msgs OpenCV) #의존성있는 파일 기술

install(TARGETS
  camera_publisher1
  camera_publisher2
  camera_subscriber1
  camera_subscriber2
  DESTINATION lib/${PROJECT_NAME})  #ros2_ws/install/camera_ros2/lib 아래의 실행파일위치 기술
                                    #실행파일의 위치를 찾기 위해

if(BUILD_TESTING)  
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
