<<<<<<< HEAD
# CMake 最低版本号要求
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
=======
﻿cmake_minimum_required(VERSION 2.8)
>>>>>>> de1bc78909c106f4a899e157e1ac750c98aa19bf

project(realsense-grabbox)

set(CMAKE_BUILD_TYPE "${CMAKE_BUILD_TYPE} -std=Release")

include(CheckCXXCompilerFlag)

CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

include_directories(asp/inc)
include_directories(uart/linux) 
include_directories(fastcluster)
include_directories(logging)

FILE(GLOB sourcefiles "asp/src/*.c" "uart/linux/*.cpp" "fastcluster/*.cpp"  "*.cpp" "*.c")

add_executable(realsense-grabbox ${sourcefiles})

# library
set(DEPENDENCIES realsense)
find_package(OpenCV 2 REQUIRED)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")

<<<<<<< HEAD
# FIND_PACKAGE(OpenCV 3 REQUIRED)
INCLUDE(/usr/local/share/OpenCV/OpenCVConfig.cmake)

message( STATUS "OpenCV library status:" )              # 输出一下得到的变量
message( STATUS "version: ${OpenCV_VERSION}" )
message( STATUS "libraries: ${OpenCV_LIBS}" )
message( STATUS "include path: ${OpenCV_INCLUDE_DIRS}")

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
ENDIF(WIN32)


# 查找当前目录下的所有源文件
# 并将名称保存到 DIR_LIB_SRCS 变量
AUX_SOURCE_DIRECTORY(. DIR_SRCS)

# 包含子目录
ADD_SUBDIRECTORY(source)

# 指定生成目标
ADD_EXECUTABLE(realsense-grabbox ${DIR_SRCS})

# 链接文件
TARGET_LINK_LIBRARIES(realsense-grabbox ${OpenCV_LIBS} source_lib ${DEPENDENCIES})

# 设置可执行文件生成路径
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
=======
target_link_libraries(realsense-grabbox pthread ${OpenCV_LIBS} ${DEPENDENCIES})
>>>>>>> de1bc78909c106f4a899e157e1ac750c98aa19bf
