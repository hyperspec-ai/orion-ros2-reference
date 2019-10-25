cmake_minimum_required(VERSION 3.5)
project(rtcm)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(rtcm_msgs_DIR ../../install/rtcm_msgs/share/rtcm_msgs/cmake)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rtcm_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)

add_library( RtcmNode SHARED
  rtcmnode.cpp
  ntripclient.cpp
  socket.cpp
  base64/base64.cpp 
)
target_include_directories(RtcmNode PRIVATE  
  "./"
  
)
rclcpp_components_register_nodes(RtcmNode "rtcm::RtcmNode")
target_compile_definitions(RtcmNode
  PRIVATE "MINIMAL_COMPOSITION_DLL")

ament_target_dependencies(RtcmNode
  "rclcpp"
  "rclcpp_components"
  "rtcm_msgs"
  "lifecycle_msgs"
  "rclcpp_lifecycle"
  )

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
    )
endif()

# Executable for easy single-node debugging


add_executable(RtcmNode_exec
  exec_entry.cpp)

target_include_directories(RtcmNode_exec PRIVATE
  "./"
)

target_link_libraries(RtcmNode_exec
  RtcmNode
)

ament_target_dependencies(RtcmNode_exec
  "rclcpp"
  "rclcpp_components"
  "rtcm_msgs"
  "RtcmNode"
)


install(TARGETS
  RtcmNode
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  RtcmNode_exec
  DESTINATION lib/${PROJECT_NAME})
ament_package()
