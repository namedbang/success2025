cmake_minimum_required(VERSION 3.10)
set(CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(RM)

enable_language(CUDA)#激活CUDA语言支持，CUDA时可选
set(CMAKE_CUDA_ARCHITECTURES 72)
include(CheckLanguage)
check_language(CUDA)
if(CMAKE_CUDA_COMPILER)
    message(STATUS "Find CUDA !")
else()
    message(FATAL_ERROR "Not found CUDA!")
endif()
# CUDA
find_package(CUDA REQUIRED)
# TensorRT
set(TensorRT_INCLUDE_DIRS /usr/include /usr/include/aarch-linux-gnu)
set(TensorRT_LIBRARIES /usr/lib/aarch64-linux-gnu)
set(TensorRT_DIR /usr/src/tensorrt/)
message(STATUS "TensorRT Libs: \n\n${TensorRT_LIBRARIES}\n")
message(STATUS "TensorRT Headers: \n${TensorRT_INCLUDE_DIRS}\n")
list(APPEND INCLUDE_DIRS
        ${TensorRT_INCLUDE_DIRS}
        include
        )
list(APPEND ALL_LIBS
        ${TensorRT_LIBRARIES}
        )
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
find_package(CUDA REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PkgConfig REQUIRED)
find_package(jetson-utils REQUIRED)
find_package(JetsonGPIO)
pkg_check_modules(GST REQUIRED gstreamer-1.0)
pkg_check_modules(GSTAPP REQUIRED gstreamer-app-1.0)
#以下头文件-----------------
include_directories(/usr/include/) #os
include_directories(${PROJECT_DIR}/include/mind/) #迈德+=
include_directories(${OPENCV_INCLUDE_DIRS} ${GST_INCLUDE_DIRS} ${GSTAPP_INCLUDE_DIRS})
include_directories(${PROJECT_DIR}/src/app/api)
include_directories(${PROJECT_DIR}/src/hardware/api)
include_directories(${PROJECT_DIR}/src/process)
include_directories(${PROJECT_DIR}/src/cuda)
include_directories(${PROJECT_DIR}/src/hardware/uart)
include_directories(${PROJECT_DIR}/src/hardware/can)
include_directories(${PROJECT_DIR}/src/utils)
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${PROJECT_NAME} PUBLIC ${ALL_LIBS})
#C文件路径变量----------------
set(SOURCE_FILES_HC "")  
# 使用 file(GLOB_RECURSE) 命令递归查找所有 .cpp 和 .h 文件  
file(GLOB_RECURSE SOURCE_FILES_HC "${PROJECT_DIR}/src/*.cpp" "${PROJECT_DIR}/src/*.cu" "${PROJECT_DIR}/src/*.c")  

add_executable(RM_exe   ${PROJECT_DIR}/src/main.cpp 
                        ${SOURCE_FILES_HC}
                                                    )#创建exe
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
target_link_libraries(${MY_EXECUTABLE} jetson-utils ${CUDA_LIBRARIES} JetsonGPIO::JetsonGPIO)
target_link_libraries(${MY_EXECUTABLE} ${OpenCV_LIBS} ${GST_LIBRARIES} ${GSTAPP_LIBRARIES})
target_link_libraries (${MY_EXECUTABLE} Eigen3::Eigen)
target_link_libraries (${MY_EXECUTABLE} nvinfer nvinfer_plugin cudart)