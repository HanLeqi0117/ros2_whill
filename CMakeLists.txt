cmake_minimum_required(VERSION 3.5)
project(ros2_whill)

#add_compile_options(-std=c++11)
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(rosidl_default_generators REQUIRED)

ament_auto_find_build_dependencies()

# Build Interfaces
# declare the message files to generate code for
set(interface_files
   "msg/WhillModelC.msg"
   "msg/WhillSpeedProfile.msg"
   "srv/SetBatteryVoltageOut.srv"
   "srv/SetPower.srv"
   "srv/SetSpeedProfile.srv"
 )

rosidl_generate_interfaces(${PROJECT_NAME}
   ${interface_files}
   
)
ament_export_dependencies(rosidl_default_runtime)

ament_auto_find_build_dependencies()

include_directories(include)

# ament_auto_add_library(${PROJECT_NAME}_component SHARED
#   src/${PROJECT_NAME}_component.cpp)
# rclcpp_components_register_nodes(${PROJECT_NAME}_component "WhillController")
# target_compile_options(${PROJECT_NAME}_component PUBLIC -Wall)

ament_auto_add_executable(${PROJECT_NAME}_node src/main.cpp src/whill_modelc/com_whill.cpp src/whill_modelc/uart.cpp src/odom.cpp)
ament_auto_add_executable(whill_joy src/whill_joy)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(whill_joy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
rosidl_target_interfaces(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  rosidl_typesupport_cpp
)
rosidl_target_interfaces(whill_joy
  ${PROJECT_NAME}
  rosidl_typesupport_cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  params
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_auto_package()