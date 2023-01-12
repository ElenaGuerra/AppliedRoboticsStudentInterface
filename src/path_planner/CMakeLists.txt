cmake_minimum_required(VERSION 3.8)
project(path_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(obstacles_msgs REQUIRED)
#find_package(moveit_visual_tools REQUIRED)
find_package(graph_msgs REQUIRED)
#find_package(cmath REQUIRED)
#find_package(random REQUIRED)
#find_package(iostream REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
#set(clipper_DIR "/usr/local/lib/cmake")
find_package(clipper_library REQUIRED)


##roadmapPublisher
add_executable(roadmapPublisher src/roadmap_publisher.cpp)
ament_target_dependencies(roadmapPublisher clipper_library rclcpp obstacles_msgs graph_msgs)

# install targets
install(TARGETS
  roadmapPublisher
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()