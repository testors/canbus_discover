cmake_minimum_required(VERSION 3.10)
project(canbus_discover C OBJC)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Compiler flags
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -O2)
endif()

# Include paths
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Source files
set(SOURCES
    main.m
    elm327.c
    stn2255.c
    can_discover.c
)

# Executable
add_executable(canbus_discover ${SOURCES})

# macOS frameworks
target_link_libraries(canbus_discover
    "-framework CoreBluetooth"
    "-framework IOBluetooth"
    "-framework Foundation"
    m
)

message(STATUS "CAN Bus Discover Build Configuration")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C Compiler: ${CMAKE_C_COMPILER}")
