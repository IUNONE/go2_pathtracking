cmake_minimum_required(VERSION 3.5  tf2_ros
  tf2_msgs
  geometry_msgs
)
project(go2_pathtracking)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


include_directories(include include/common include/nlohmann)
link_directories(src)

set (
  DEPENDENCY_LIST
  unitree_go
  unitree_api
  rclcpp
  std_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# reader
add_executable(read_low_state src/reader/read_low_state.cpp)
add_executable(read_motion_state src/reader/read_motion_state.cpp)
add_executable(read_wireless_controller src/reader/read_wireless_controller.cpp)
ament_target_dependencies(read_low_state ${DEPENDENCY_LIST})
ament_target_dependencies(read_motion_state ${DEPENDENCY_LIST})
ament_target_dependencies(read_wireless_controller ${DEPENDENCY_LIST})

# go2
add_executable(pathtracking_node src/go2/pathtracking_node.cpp src/common/go2_move_client.cpp)
# target_compile_features(pathtracking_node PRIVATE cxx_std_20)
add_executable(test_node src/test_track_node.cpp)

ament_target_dependencies(pathtracking_node ${DEPENDENCY_LIST})
target_link_libraries(pathtracking_node Eigen3::Eigen)

ament_target_dependencies(test_node
  rclcpp
  geometry_msgs
  nav_msgs
)



install(TARGETS 
        read_low_state
        read_motion_state
        read_wireless_controller
        pathtracking_node
        test_node
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
