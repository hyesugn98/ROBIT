##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(rescue2019)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS qt_build roscpp
    roslib
    cv_bridge
    message_runtime
    message_generation
    image_transport
    std_msgs
    )
include_directories(${catkin_INCLUDE_DIRS})
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything.
catkin_package()

set (OpenCV_INClUDE_DIRS
 /usr/local/include
 /usr/local/include/opencv2
 )

set(OpenCV_LIB_DIR
    /usr/local/lib
    )

set(Opencv_LIBS
    opencv_core
    opencv_highgui
    opencv_imgcodecs
    opencv_videoio
    opencv_imgproc
    opencv_features2d
    opencv_xfeatures2d
    opencv_flann
    opencv_calib3d
    )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES tledak_vision2
    CATKIN_DEPENDS image_transport roscpp roslib
    DEPENDS)

##############################################################################
# Qt Environment
##############################################################################
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
#SET(CMAKE_AUTOUIC ON)

find_package(Qt5 COMPONENTS Core Gui Widgets PrintSupport Network)
##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/rescue2019/*.hpp)
QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})

include_directories(
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5Multimedia_INCLUDE_DIRS}
    ${Qt5Network_INCLUDE_DIRS}
    )

add_definitions( -std=c++11 -fPIC)
##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)
set( QT_SOURCES
    src/main.cpp
    src/main_window.cpp
    src/qcustomplot.cpp
    src/qnode.cpp
    src/udpsocket.cpp

    include/rescue2019/qcustomplot.h
    include/rescue2019/main_window.hpp
    include/rescue2019/qnode.hpp
    include/rescue2019/udpsocket.hpp
    include/rescue2019/main_window.hpp
    )
##############################################################################
# Binaries
##############################################################################

add_executable(rescue2019 ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})

target_link_libraries(rescue2019 ${catkin_LIBRARIES} ${Opencv_LIBS}
    Qt5::Core
    Qt5::Widgets
    Qt5::PrintSupport
    Qt5::Multimedia
    Qt5::Network)
install(TARGETS rescue2019 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
