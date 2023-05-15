cmake_minimum_required(VERSION 3.8)
project(registration_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(registration_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

add_library( registration SHARED src/registration_server.cpp src/registration_client.cpp)
ament_target_dependencies( registration rclcpp registration_msgs geometry_msgs Eigen3 tf2 tf2_ros)

add_executable( reg_svr src/registration_node.cpp )
target_link_libraries( reg_svr registration )
ament_target_dependencies( reg_svr rclcpp registration_msgs Eigen3 tf2 tf2_ros)

add_executable( reg_cli src/reg_client_example_node.cpp )
target_link_libraries( reg_cli registration )
ament_target_dependencies( reg_cli rclcpp registration_msgs Eigen3 )

install( DIRECTORY include/ DESTINATION include )
install( TARGETS registration LIBRARY DESTINATION lib )
install( TARGETS reg_svr reg_cli DESTINATION lib/${PROJECT_NAME} )

ament_export_include_directories(include)
ament_export_libraries(registration)

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
