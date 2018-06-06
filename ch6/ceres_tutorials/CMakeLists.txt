cmake_minimum_required(VERSION 3.2.0)
project(ceres_project)

set(CMAKE_CXX_STANDARD 11)

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# 寻找Ceres库并添加它的头文件
find_package( Ceres REQUIRED )
include_directories( ${CERES_INCLUDE_DIRS} )

add_executable(ceres_project main.cpp)
target_link_libraries(ceres_project ${CERES_LIBRARIES})

add_executable(powell powell.cc)
target_link_libraries(powell ${CERES_LIBRARIES})

add_executable(curve_fitting curve_fitting.cc)
target_link_libraries(curve_fitting ${CERES_LIBRARIES})
