cmake_minimum_required(VERSION 3.8)
project(imu_simulator_package)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
        include/imu_simulator_package
        ${EIGEN3_INCLUDE_DIR}
        ${YAML_CPP_INCLUDE_DIR})

set(HEADER_FILES
        include/imu_simulator_package/imu_simulator.h)

add_library(imu_simulator SHARED
        src/imu_simulator.cpp
        ${HEADER_FILES})

# add imu_simulator_node
add_executable(${PROJECT_NAME}_node src/imu_simulator_node.cpp)

ament_target_dependencies(${PROJECT_NAME}_node PUBLIC
        rclcpp
        std_msgs
        sensor_msgs
        nav_msgs
        geometry_msgs
        diagnostic_msgs
        tf2_ros
        Eigen3)

target_link_libraries(${PROJECT_NAME}_node PUBLIC imu_simulator)

target_include_directories(${PROJECT_NAME}_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_compile_features(${PROJECT_NAME}_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS
        ${PROJECT_NAME}_node
        DESTINATION
        lib/${PROJECT_NAME})

# add csv_test_executable
add_executable(imu_simulator_static_csv src/imu_simulator_static_csv.cpp)

ament_target_dependencies(imu_simulator_static_csv PUBLIC
        Eigen3)

target_link_libraries(imu_simulator_static_csv PUBLIC imu_simulator)

target_include_directories(imu_simulator_static_csv PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_compile_features(imu_simulator_static_csv PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS
        imu_simulator_static_csv
        DESTINATION lib/${PROJECT_NAME})

install(TARGETS imu_simulator
        EXPORT imu_simulator
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

# add odometry test publisher
add_executable(ground_truth_test_publisher_node src/ground_truth_test_publisher_node.cpp)

ament_target_dependencies(ground_truth_test_publisher_node PUBLIC
        rclcpp
        nav_msgs
        geometry_msgs
        tf2_ros
        Eigen3)

target_link_libraries(ground_truth_test_publisher_node PUBLIC imu_simulator)

target_include_directories(ground_truth_test_publisher_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_compile_features(ground_truth_test_publisher_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS
        ground_truth_test_publisher_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        launch
        config
        DESTINATION
        share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips uncrustify (source code formatting)
  # set(ament_cmake_uncrustify_FOUND TRUE)

  find_package(ament_cmake_gtest REQUIRED)

  set(TEST_FILES
      test/main.cpp
      test/imu_simulator_test.cpp)

  ament_add_gtest(${PROJECT_NAME}_test ${TEST_FILES})

  target_link_libraries(${PROJECT_NAME}_test imu_simulator)

  install(TARGETS
          ${PROJECT_NAME}_test
          DESTINATION lib/${PROJECT_NAME})

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)

  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify)

  # enforce linters and static code analyzers defined in ament_lint_common package
  ament_lint_auto_find_test_dependencies()

  # uncomment to include uncrustify explicitly
  # find_package(ament_cmake_uncrustify)
  # ament_uncrustify(CONFIG_FILE "./uncrustify.cfg" TESTNAME "custom_uncrustify")

endif()

ament_package()