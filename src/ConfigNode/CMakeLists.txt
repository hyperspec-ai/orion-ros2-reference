cmake_minimum_required(VERSION 3.5)
project(config)
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
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rcutils REQUIRED)
find_package(CURL REQUIRED)
find_package(LibArchive REQUIRED)
find_library(PAHO_CPP_LIB NAMES libpaho-mqttpp3.so REQUIRED)
find_library(PAHO_C_LIB NAMES libpaho-mqtt3a.so REQUIRED)



add_library( ConfigNode SHARED
confignode.cpp
configurationhandler.cpp
executioncontext.cpp
executionhandler.cpp
packagedownloader.cpp
persistancehandler.cpp
mqttbackend.cpp
lifecyclemanager.cpp
  
)
IF(CMAKE_CROSSCOMPILING)
  target_include_directories(ConfigNode PRIVATE  
    "./"
    "../../common_src/"
    "../../common_src/json/include"
    SYSTEM "${CMAKE_SYSROOT}/usr/local/include"
  )  
ELSE(CMAKE_CROSSCOMPILING)
  target_include_directories(ConfigNode PRIVATE  
    "./"
    "../../common_src/"
    "../../common_src/json/include"
    "/usr/local/include"
  )  
ENDIF(CMAKE_CROSSCOMPILING)



add_subdirectory(../../common_src/ ../install/commonLib/lib/)

target_link_libraries(ConfigNode
${LibArchive_LIBRARIES}
${CURL_LIBRARIES}
${PAHO_CPP_LIB}
${PAHO_C_LIB}
OrionCommon
)

rclcpp_components_register_nodes(ConfigNode "config::ConfigNode")
target_compile_definitions(ConfigNode
  PRIVATE "MINIMAL_COMPOSITION_DLL")

ament_target_dependencies(ConfigNode
  "class_loader"
  "rclcpp"
  "rclcpp_components"
  "rclcpp_lifecycle"
  "lifecycle_msgs"
  
  )

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
    )
endif()

# Executable for easy single-node debugging


add_executable(ConfigNode_exec
  exec_entry.cpp)


IF(CMAKE_CROSSCOMPILING)
  target_include_directories(ConfigNode_exec PRIVATE  
    "./"
    "../../common_src/"
    "../../common_src/json/include"
    SYSTEM PUBLIC "/opt/sysroot/usr/local/include"
  )  
ELSE(CMAKE_CROSSCOMPILING)
  target_include_directories(ConfigNode_exec PRIVATE  
    "./"
    "../../common_src/"
    "../../common_src/json/include"
    "/usr/local/include"
  )  
ENDIF(CMAKE_CROSSCOMPILING)

target_link_libraries(ConfigNode_exec
ConfigNode
${LibArchive_LIBRARIES}
)

ament_target_dependencies(ConfigNode_exec
  "rclcpp"
  "rclcpp_components"  
  "ConfigNode"  
)


install(TARGETS
ConfigNode
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
ConfigNode_exec
  DESTINATION lib/${PROJECT_NAME})
ament_package()
