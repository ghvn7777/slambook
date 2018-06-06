cmake_minimum_required( VERSION 2.8 )
project( ceres_curve_fitting )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# 添加 cmake 模块以使用 ceres 库
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# 寻找 Ceres 库并添加它的头文件
find_package( Ceres REQUIRED )
include_directories( ${CERES_INCLUDE_DIRS} )

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_DIRS} )

add_executable( curve_fitting1 main.cpp )
# 与 Ceres 和 OpenCV 链接
target_link_libraries(curve_fitting1 ${CERES_LIBRARIES} ${OpenCV_LIBS})

add_executable(curve_fitting2 curve_fitting.cpp)
target_link_libraries(curve_fitting2 ${CERES_LIBRARIES} ${OpenCV_LIBS})