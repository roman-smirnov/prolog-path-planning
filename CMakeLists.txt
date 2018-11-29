cmake_minimum_required(VERSION 3.12)
project(prolog-path-planning)

set(CMAKE_CXX_STANDARD 17)

set(SOURCES src/main.cpp src/network.cpp src/network.h src/json.hpp src/spline.h src/map.cpp src/map.h src/plan.cpp src/plan.h src/trajectory.cpp src/trajectory.h)

# MACOS SPECIFIC
if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    include_directories(/usr/local/include)
    include_directories(/usr/local/opt/openssl/include)
    link_directories(/usr/local/lib)
    link_directories(/usr/local/opt/openssl/lib)
    link_directories(/usr/local/Cellar/libuv/1.23.0/lib)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")


add_executable(path_planning ${SOURCES})
target_link_libraries(path_planning z ssl uv uWS)