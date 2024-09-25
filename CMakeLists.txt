cmake_minimum_required(VERSION 3.10)
set(CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(RM)
#-------coloer
string(ASCII 27 Esc)
set(R "${Esc}[0;31m")   #红色
#Esc[0m 
set(E "${Esc}[m" )      #结束颜色设置
set(B "${Esc}[1;34m")   #蓝色高亮
set(G "${Esc}[1;32m")   #绿色高亮
set(RB "${Esc}[1;31;40m") #红色字体黑色背景
#-------
set(PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR})# 定义一个宏来表示当前工程文件夹目录
message("${B}当前工程文件夹目录: ${PROJECT_DIR}${E}")
#以上工程相关配置--------------------------------
#查找第三方包
find_package(OpenCV 4 REQUIRED)
#以下头文件-----------------
include_directories(/usr/include/) #os
include_directories(${PROJECT_DIR}/include/mind/) #迈德+=
include_directories(${OPENCV_INCLUDE_DIRS})
#C文件路径变量----------------
add_executable(RM_exe ${PROJECT_DIR}/src/main.cpp)#创建exe
set(MY_EXECUTABLE RM_exe)
#将库和头文件加入--
#--
find_library(OS_LIBRARY_libMVSDK libMVSDK.so PATHS /lib)
if(OS_LIBRARY_libMVSDK)
    target_link_libraries(${MY_EXECUTABLE} ${OS_LIBRARY_libMVSDK})
    message("-- ${G}Library of libMVSDK found${E}")
else()
    message(FATAL_ERROR "${R}Library of libMVSDK not found${E}")
endif()

target_link_libraries(${MY_EXECUTABLE} ${OpenCV_LIBS})
