cmake_minimum_required(VERSION 3.0)
project(reflection_intensity_map VERSION 1.1.0 LANGUAGES CXX)
# ビルドフラグの設定
set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")
# インクルードパスの追加
if(WIN32)
  include_directories(C:/local/include)
  link_directories(C:/local/lib)
else()
  include_directories(/usr/local/include)
  link_directories(/usr/local/lib)
endif()
# ビルドに関する設定
find_package(Boost 1.65.0 REQUIRED COMPONENTS filesystem system)
set(EXECUTABLE "reflection_intensity_map")
set(DEPENDENT_LIBS opencv_core;opencv_highgui;opencv_imgcodecs;opencv_imgproc ${Boost_LIBRARIES})
add_executable(${EXECUTABLE} src/${EXECUTABLE}.cpp)
target_link_libraries(${EXECUTABLE} ${DEPENDENT_LIBS})