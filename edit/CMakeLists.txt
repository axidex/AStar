cmake_minimum_required(VERSION 3.20)

set(EDITOR_PROJECT_NAME edit)

add_executable(${EDITOR_PROJECT_NAME}
    main.cpp
)

target_link_libraries(${EDITOR_PROJECT_NAME} source)
target_compile_features(${EDITOR_PROJECT_NAME} PUBLIC cxx_std_17)

set_target_properties(${EDITOR_PROJECT_NAME} PROPERTIES RUNTIME_OUTPIT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/)