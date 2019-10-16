cmake_minimum_required(VERSION 3.5)
project(ublox)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rtcm_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)

set(JSON_BuildTests OFF CACHE INTERNAL "")
#add_subdirectory(json)
add_library(UbxGpsNode SHARED
  ubxgpsnode.cpp
  ubxcfgmessage.cpp
  ubxmessage.cpp
  ubxreceivemessage.cpp
  ubxserialhandler.cpp
  messages/ubxackack.cpp
  messages/ubxacknak.cpp
  messages/ubxnaveoe.cpp
  messages/ubxnavhighpresgeo.cpp
  messages/ubxnavpvt.cpp
  messages/ubxnavratecfg.cpp
  messages/ubxnavstatus.cpp
  messages/ubxusbmsgcfg.cpp
  messages/ubxusbprotcfg.cpp
)




rclcpp_components_register_nodes(UbxGpsNode "UbxGpsNode")

target_compile_definitions(UbxGpsNode
  PRIVATE "MINIMAL_COMPOSITION_DLL")

ament_target_dependencies(UbxGpsNode
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "rtcm_msgs"
  "lifecycle_msgs"
  "rclcpp_lifecycle"
)
add_subdirectory(../../common_src/ ../install/commonLib/lib/)
target_include_directories(UbxGpsNode PRIVATE
  "./"
  "../../common_src/"
  "../../common_src/json/include"
)

target_link_libraries(UbxGpsNode
OrionCommon
)


if(BUILD_TESTING)
  # Include linting tests
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(ubx_test test/ubx_unit_test.cpp)
  target_link_libraries(ubx_test UbxGpsNode)
endif()

# Executable for easy single-node debugging
add_executable(UbxGpsNode_exec
  exec_entry.cpp)

ament_target_dependencies(UbxGpsNode_exec
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
)

target_include_directories(UbxGpsNode_exec PRIVATE
  "./"
)

target_link_libraries(UbxGpsNode_exec
  UbxGpsNode
)
install(TARGETS
UbxGpsNode
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  UbxGpsNode_exec
  DESTINATION lib/${PROJECT_NAME})



ament_package()
