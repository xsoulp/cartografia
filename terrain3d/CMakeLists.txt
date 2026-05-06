cmake_minimum_required(VERSION 3.16)
project(terrain3d LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PCL 1.14 REQUIRED COMPONENTS common io kdtree visualization)

add_definitions(${PCL_DEFINITIONS})

add_executable(terrain_blanket src/terrain_blanket.cpp)
target_link_libraries(terrain_blanket PRIVATE ${PCL_LIBRARIES})
target_include_directories(terrain_blanket PRIVATE ${PCL_INCLUDE_DIRS})

add_executable(terrain_viewer src/terrain_viewer.cpp)
target_link_libraries(terrain_viewer PRIVATE ${PCL_LIBRARIES})
target_include_directories(terrain_viewer PRIVATE ${PCL_INCLUDE_DIRS})
