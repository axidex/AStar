cmake_minimum_required(VERSION 3.20)

set(ASTAR_PROJECT_NAME source)

set(ASTAR_PUBLIC_INCLUDE
    AStar.hpp
)


set(ASTAR_PRIVATE_SRC
    AStar.cpp
)

set(ASTAR_ALL_SOURCES
    ${ASTAR_PUBLIC_INCLUDE}
    ${ASTAR_PRIVATE_SRC}
)

add_library(${ASTAR_PROJECT_NAME} STATIC
    ${ASTAR_ALL_SOURCES}
)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES
    ${ASTAR_ALL_SOURCES}
)

target_include_directories(${ASTAR_PROJECT_NAME} PUBLIC include)
target_include_directories(${ASTAR_PROJECT_NAME} PRIVATE src)
target_compile_features(${ASTAR_PROJECT_NAME} PUBLIC cxx_std_17)










