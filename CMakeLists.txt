cmake_minimum_required(VERSION 3.10)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE "debug")
set(ProjectName "project_name")

project("${ProjectName}" VERSION 0.1)
set(CMAKE_CXX_STANDARD 23)

find_package(Python COMPONENTS Interpreter Development REQUIRED)
add_subdirectory(dependencies)

message(STATUS "Python_EXECUTABLE: ${Python_EXECUTABLE}")
message(STATUS "Python_INCLUDE_DIRS: ${Python_INCLUDE_DIRS}")
message(STATUS "Python_VERSION: ${Python_VERSION}")

add_library(esjson_lib STATIC
    # headers

    # sources
    src/main.cpp
)
target_link_libraries(esjson_lib PUBLIC MidnightMath)
target_include_directories(esjson_lib PUBLIC include)

nanobind_add_module(esjson src/python_bindings.cpp)
target_link_libraries(esjson PRIVATE esjson_lib)
