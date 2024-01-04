cmake_minimum_required(VERSION 3.5)
project(ros2_orb_slam)


# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV 4.2)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(image_transport REQUIRED)
find_package(Pangolin REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
set(ORB_SLAM3_DIR /ORB_SLAM3)

include_directories(
    Eigen
    ORB_SLAM3
    ORB_SLAM3/include
    ORB_SLAM3/include/CameraModels
    ORB_SLAM3/Thirdparty/Sophus)

set(LIBS 
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    ${image_transport_LIBRARIES}
    ${tf2_ros_LIBRARIES}
    ${tf2_LIBRARIES}
    ${CMAKE_SOURCE_DIR}/ORB_SLAM3/lib/libORB_SLAM3.so
    ${CMAKE_SOURCE_DIR}/ORB_SLAM3/Thirdparty/DBoW2/lib/libDBoW2.so
    ${CMAKE_SOURCE_DIR}/ORB_SLAM3/Thirdparty/g2o/lib/libg2o.so
    -lboost_serialization
    -lcrypto)

add_executable(slam src/rgbd.cpp src/rgbd-slam-node.cpp)
add_executable(mono src/mono.cpp src/mono-slam-node.cpp)
add_executable(monoIN src/mono_In.cpp)
target_link_libraries(slam ${LIBS})
target_link_libraries(mono ${LIBS})
target_link_libraries(monoIN ${LIBS})
ament_target_dependencies(slam rclcpp sensor_msgs cv_bridge message_filters tf2_ros tf2 image_transport Pangolin)
ament_target_dependencies(mono rclcpp sensor_msgs cv_bridge message_filters tf2_ros tf2 image_transport Pangolin)
ament_target_dependencies(monoIN rclcpp sensor_msgs cv_bridge message_filters tf2_ros tf2 image_transport Pangolin)

install(TARGETS slam
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS mono
  DESTINATION lib/${PROJECT_NAME})
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
