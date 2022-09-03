cmake_minimum_required(VERSION 3.16)


set(CMAKE_CXX_STANDARD 17)

find_package(casadi REQUIRED)   # 引入包，需要安装到系统环境才有

##set(CMAKE_CXX_COMPILER  "clang++" ) # 指定编译器
##如果要生成debug版本
#SET(CMAKE_BUILD_TYPE "Debug")
##如果要生成release版本
##SET(CMAKE_BUILD_TYPE "Release")
##添加的支持debug和release支持
#SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
##SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -Os -Wall -g")

find_package(Eigen3 REQUIRED)

add_library(MPs
    include/directCollocationSolver.h
    include/common.h
    src/directCollocationSolver.cpp
)

#include(CheckIPOSupported)
#check_ipo_supported(RESULT _IsIPOSupported)
#if(_IsIPOSupported)
#    message(STATUS "Turn on INTERPROCEDURAL_OPTIMIZATION")
#    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
#endif()


#option(BUILD_SHARED_LIBS   "Build as shared library." OFF)
#option(BUILD_EXAMPLES      "Build example programs."   OFF)
#option(BUILD_TESTS         "Build test programs."      OFF)
#option(BUILD_DOCS          "Build documentation."      OFF)

#include_directories(
#        ${CMAKE_CURRENT_SOURCE_DIR}/include
#)

# Grab headers from the include directory.
target_include_directories(MPs PUBLIC include)

# Grab headers from this src directory. Don't share them with clients.
target_include_directories(MPs PRIVATE src)

#LINK_DIRECTORIES(/lib/x86_64-linux-gnu)
# Set options for the compiler.
target_compile_options(MPs PRIVATE -Wall)

target_link_libraries(MPs
        Eigen3::Eigen
        casadi
        )


