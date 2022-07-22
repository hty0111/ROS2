cmake_minimum_required(VERSION 3.8)
project(learning_service_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(learning_interface REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(person_info_server src/person_info_server.cpp)
ament_target_dependencies(person_info_server rclcpp learning_interface)

add_executable(person_info_client src/person_info_client.cpp)
ament_target_dependencies(person_info_client rclcpp learning_interface)

install(TARGETS
        person_info_server
        person_info_client
        DESTINATION lib/${PROJECT_NAME}
        )

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
