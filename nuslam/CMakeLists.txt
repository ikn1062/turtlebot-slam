cmake_minimum_required(VERSION 3.8)
project(nuslam)

find_package(ament_cmake REQUIRED)
find_package(turtlelib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Armadillo REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nuturtle_control REQUIRED)

add_library(${PROJECT_NAME} src/ekf_slam.cpp src/circle_library.cpp)

target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>
    ${ARMADILLO_INCLUDE_DIRS})

add_executable(slam src/slam.cpp)
ament_target_dependencies(slam rclcpp std_msgs std_srvs
                          geometry_msgs sensor_msgs
                          nuturtlebot_msgs nav_msgs tf2_ros
                          tf2 visualization_msgs nuturtle_control
                          Armadillo)
target_link_libraries(slam ${rclcpp_LIBRARIES})
target_link_libraries(slam ${std_msgs_LIBRARIES})
target_link_libraries(slam ${ARMADILLO_LIBRARIES})
target_link_libraries(slam ${PROJECT_NAME})
target_link_libraries(slam turtlelib::turtlelib "${cpp_typesupport_target}")

add_executable(landmarks src/landmarks.cpp)
ament_target_dependencies(landmarks rclcpp std_msgs std_srvs
                          geometry_msgs sensor_msgs
                          nuturtlebot_msgs nav_msgs tf2_ros
                          tf2 visualization_msgs nuturtle_control
                          Armadillo)
target_link_libraries(landmarks ${rclcpp_LIBRARIES})
target_link_libraries(landmarks ${std_msgs_LIBRARIES})
target_link_libraries(landmarks ${ARMADILLO_LIBRARIES})
target_link_libraries(landmarks ${PROJECT_NAME})
target_link_libraries(landmarks turtlelib::turtlelib "${cpp_typesupport_target}")
target_link_libraries(${PROJECT_NAME} turtlelib::turtlelib "${cpp_typesupport_target}")

install(TARGETS
  slam
  landmarks
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch src config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

# enable C++ 17
target_compile_features(${PROJECT_NAME}  PUBLIC cxx_std_17)

# warnings are your friend!
target_compile_options(${PROJECT_NAME}  PUBLIC -Wall -Wextra -Wpedantic)



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

# CMake also has the ability to generate doxygen documentation
if(NOT CMAKE_CROSSCOMPILING)
# Use the cmake testing functionality. A test is just an executable.
# We use the catch2 unit testing framework
find_package(Catch2 3 REQUIRED)

include(CTest)
add_executable(circle_test tests/circle_tests.cpp)
target_link_libraries(circle_test nuslam Catch2::Catch2WithMain ${ARMADILLO_LIBRARIES})
add_test(NAME Test_of_Circle_Alg COMMAND circle_test)
endif()
