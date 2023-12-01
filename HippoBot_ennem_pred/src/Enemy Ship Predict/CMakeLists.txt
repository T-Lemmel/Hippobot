cmake_minimum_required(VERSION 3.8)
project(enemy_predictor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(PCL REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

install(PROGRAMS scripts/enemy_ship_predictor.py
        DESTINATION lib/${PROJECT_NAME})

ament_auto_package()
