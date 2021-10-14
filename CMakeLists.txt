cmake_minimum_required(VERSION 3.20)

set(PROJECT_NAME ASTAR)

add_subdirectory(source)
add_subdirectory(edit)

set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT edit)
