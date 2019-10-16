cmake_minimum_required(VERSION 3.5)
project(commonLib)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(JSON_BuildTests OFF CACHE INTERNAL "")
add_subdirectory(json)

add_library( OrionCommon
routeparser.cpp  
)
target_include_directories(OrionCommon PRIVATE  
  "./"
  "./json/include"
)
target_link_libraries(OrionCommon
nlohmann_json::nlohmann_json
)
install(TARGETS
OrionCommon
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

