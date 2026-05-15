cmake_minimum_required(VERSION 3.8)
project(caddy_ai2_ros2_control_sensors_sbg_ig500n)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(hardware_interface QUIET)
find_package(pluginlib QUIET)

include_directories(/usr/local/include/sbgCom)
find_library(SBG_LIB NAMES sbgCom PATHS /usr/local/lib)

if(SBG_LIB)
  # --- Standalone ROS 2 node ---
  add_executable(sbg_node code/src/sbg_node.cpp)
  ament_target_dependencies(sbg_node rclcpp sensor_msgs tf2)
  target_compile_features(sbg_node PUBLIC c_std_99 cxx_std_17)
  target_link_libraries(sbg_node ${SBG_LIB})

  install(TARGETS sbg_node
    DESTINATION lib/${PROJECT_NAME})

  # --- ros2_control hardware interface (plugin) ---
  if(hardware_interface_FOUND AND pluginlib_FOUND)
    add_library(sbg_hardware_interface SHARED
      code/src/sbg_hardware_interface.cpp)
    target_compile_features(sbg_hardware_interface PUBLIC cxx_std_17)
    ament_target_dependencies(sbg_hardware_interface
      hardware_interface pluginlib rclcpp tf2)
    target_link_libraries(sbg_hardware_interface ${SBG_LIB})

    pluginlib_export_plugin_description_file(
      hardware_interface sbg_hardware_interface_plugin.xml)

    install(TARGETS sbg_hardware_interface
      DESTINATION lib)
    install(FILES sbg_hardware_interface_plugin.xml
      DESTINATION share/${PROJECT_NAME})
  else()
    message(STATUS "hardware_interface/pluginlib not found — "
                   "sbg_hardware_interface plugin will not be built.")
  endif()
else()
  message(WARNING "sbgCom library not found — no hardware targets will be built. "
                  "Install the SBG SDK to /usr/local.")
endif()

install(DIRECTORY config description
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
