cmake_minimum_required(VERSION 3.5)

project(3dplan VERSION 0.1 LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#如果要生成debug版本
SET(CMAKE_BUILD_TYPE "Debug")
#如果要生成release版本
#SET(CMAKE_BUILD_TYPE "Release")
#添加的支持debug和release支持
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
#SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")



find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets LinguistTools DataVisualization OpenGL)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets LinguistTools DataVisualization OpenGL)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(Python3 COMPONENTS NumPy qt-material)
find_package(sbpl REQUIRED)

#find_package(opengl REQUIRED)

include_directories(    # 包含路径后，可以直接引入头文件名字而不需要路径了
    ${catkin_INCLUDE_DIRS}
    data
    plot3d
    include
    planner
)

add_subdirectory(spiral_constraint)

find_package(Eigen3 REQUIRED)
#find_package(Matplot++ REQUIRED)

include_directories()

set(TS_FILES 3dplan_zh_CN.ts)

set(PROJECT_SOURCES
        main.cpp
        mainwindow.cpp
        include/mainwindow.h
        include/json.hpp
        include/utils.hpp
        src/utils.cpp
        data/readtiff.cpp
        data/readtiff.h
        data/readprimitives.cpp
        data/readprimitives.h
        plot3d/surfacegraph.cpp
        plot3d/surfacegraph.h
        plot3d/QPlot3D.cpp
        plot3d/QPlot3D.h
        planner/planner.h
        planner/planner.cpp

#        environment/environment_navxyztheta.cpp
#        environment/environment_navxyztheta.h

        mainwindow.ui
        ${TS_FILES}
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(3dplan
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
# Define target properties for Android with Qt 6 as:
#    set_property(TARGET 3dplan APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation

    qt_create_translation(QM_FILES ${CMAKE_SOURCE_DIR} ${TS_FILES})
else()
    if(ANDROID)
        add_library(3dplan SHARED
            ${PROJECT_SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(3dplan
            ${PROJECT_SOURCES}
        )
    endif()

    qt5_create_translation(QM_FILES ${CMAKE_SOURCE_DIR} ${TS_FILES})
endif()

target_link_libraries(3dplan PRIVATE Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::DataVisualization
    Qt${QT_VERSION_MAJOR}::OpenGL
    -lGL
    gdal MPs sbpl
    Eigen3::Eigen
#    Matplot++::matplot
    Python3::Python
    Python3::NumPy
#    Python3::qt-material
)

set_target_properties(3dplan PROPERTIES
    MACOSX_BUNDLE_GUI_IDENTIFIER my.example.com
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

install(TARGETS 3dplan
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(3dplan)
endif()
