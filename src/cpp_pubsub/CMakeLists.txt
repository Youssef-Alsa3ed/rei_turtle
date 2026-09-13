cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# OpenCv package
find_package(OpenCV REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DrawShape.srv"
)
# executables
# add_executable(talker src/publisher.cpp)
# ament_target_dependencies(talker rclcpp std_msgs)

# add_executable(listener src/subscriber.cpp)
# ament_target_dependencies(listener rclcpp std_msgs)

add_executable(turtlecontrol src/turtlecontrol.cpp src/graphics/shape.cpp)
ament_target_dependencies(turtlecontrol rclcpp std_msgs geometry_msgs turtlesim std_srvs)
rosidl_target_interfaces(turtlecontrol ${PROJECT_NAME} "rosidl_typesupport_cpp")

#glm 
add_subdirectory(vendor/glm)

target_link_libraries(turtlecontrol ${OpenCV_LIBS} glm::glm)




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


install(TARGETS
  turtlecontrol
  DESTINATION lib/${PROJECT_NAME})

ament_package()
