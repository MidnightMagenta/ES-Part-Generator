cmake_minimum_required(VERSION 3.10)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(ProjectName "project_name")

project("${ProjectName}" VERSION 0.1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_OSX_DEPLOYMENT_TARGET
    "13.3" CACHE STRING
    "Minimum macOS deployment version"
)

add_subdirectory(dependencies)

add_library(esjson_lib STATIC
    # headers
    include/objects.h
    include/object_tree.h
    include/scene_builder.h
    # sources
    src/object_tree.cpp
    src/objects.cpp
    src/scene_builder.cpp
)
target_link_libraries(esjson_lib PUBLIC
    nlohmann_json::nlohmann_json
)
target_include_directories(esjson_lib PUBLIC include)

add_executable(esjson_test
    src/main.cpp
)
target_link_libraries(esjson_test PRIVATE esjson_lib)

if(DEFINED SKBUILD)
    find_package(Python 3.9 COMPONENTS Interpreter Development.Module REQUIRED)

    add_subdirectory(dependencies/submodules/nanobind)

    nanobind_add_module(_esjson src/python_bindings.cpp)

    target_link_libraries(_esjson PRIVATE esjson_lib)

    install(TARGETS _esjson LIBRARY DESTINATION esjson)
endif()
